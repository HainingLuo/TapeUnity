using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using System.IO;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Obi;
// using RosImage = RosMessageTypes.Sensor.ImageMsg;
using RosBool = RosMessageTypes.Std.BoolMsg;
using RosInt = RosMessageTypes.Std.Int8Msg;
using RosImage = RosMessageTypes.Sensor.CompressedImageMsg;
using RosPose = RosMessageTypes.Geometry.PoseMsg;
using RosPoint = RosMessageTypes.Geometry.PointMsg;
using RosFloatArray = RosMessageTypes.Std.Float32MultiArrayMsg;
using RosPoseArray = RosMessageTypes.Geometry.PoseArrayMsg;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;
using RosMessageTypes.Tape;


public class PhysicsEstimation : MonoBehaviour
{
    // This class creates a set of ropes with various obi parameters,
    // and runs ROS servers to predict the rope states from the request gripper motions.

    // env variables
    public int numEnvs=9;
    private int numParams=3;
    public float envSpacing = 3.5f;
    private List<GameObject> envs=new List<GameObject>();

    // rope variables
    private List<ObiRope> ropes=new List<ObiRope>();
    private List<string> ropeNames;
    private ObiSolver obiSolver;
    private ObiFixedUpdater obiUpdater;
    public Material ropeMaterial;
    public float ropeLength=1.0f;
    public float ropeRadius=0.03f;
    public float ropeResolution=0.2f;
    private int ropeColliderFilterEnd = ObiUtils.MakeFilter((1 << 0) | (1 << 1) | (1 << 3), 1);
    private int ropeColliderFilter = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything ^ (1 << 2), 0);
    public int ropeNumGroups=10;
    // private List<Vector3> gravity_centres = new List<Vector3>();
    public int ropePooledParticles=0;
    public float ropeStretchCompliance=0.0f;
    public float ropeStretchingScale=1.0f;
    public float ropeBendCompliance=0.0f;
    public float ropeMaxBending=0.05f;
    public float ropeMass=0.05f;
    public float ropeDamping=0.0f;
    public int obiSubsteps=4;
    public float obiGravity=-9.8f;
    public float obiDamping=0.99f;
    public bool ropeSelfCollision=true;
    private List<List<int>> ropeParticleIds = new List<List<int>>(); // particle ids of each group
    private List<List<int>> ropeControlledIds = new List<List<int>>(); // controlled particle ids of each group

    // gripper variables
    private List<GameObject> leftGrippers=new List<GameObject>();
    private List<GameObject> rightGrippers=new List<GameObject>();
    private List<Vector3> leftGrippersTargets=new List<Vector3>();
    private List<Vector3> rightGrippersTargets=new List<Vector3>();
    private List<ObiParticleAttachment> leftGripperAttachments = new List<ObiParticleAttachment>();
    private List<ObiParticleAttachment> rightGripperAttachments = new List<ObiParticleAttachment>();
    // private ObiParticleAttachment leftGripperAttachment;
    // private ObiParticleAttachment rightGripperAttachment;
    public float gripperTranslationSpeed=0.01f; // mm


    // ROS variables
    private ROSConnection rosConnector;
    private string predServiceName = "/unity_predict";

    private bool initialised=false;
    private bool predicting=false;
    private int predCounter=0;
    public int predTime=10;


    void Start()
    {
        // Create a set of envs
        int envRows = (int)Mathf.Ceil(Mathf.Sqrt(numEnvs));
        for (int i=0; i<envRows; i++) {
            for (int j=0; j<envRows; j++) {
                int idx = i*envRows+j;
                if (idx>=numEnvs) break;
                GameObject env = new GameObject("env"+idx.ToString());
                env.transform.position = 
                        new Vector3((j-envRows/2)*envSpacing, 0, -(i-envRows/2)*envSpacing);
                envs.Add(env);
                // add a plane
                // GameObject plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
                GameObject plane = GameObject.CreatePrimitive(PrimitiveType.Cube);
                plane.transform.position = env.transform.position-Vector3.up*0.05f;
                // plane.transform.localScale = new Vector3(0.3f, 1.0f, 0.3f);
                plane.transform.localScale = new Vector3(3f, 0.1f, 3f);
                plane.transform.parent = env.transform;
                plane.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Black");
                plane.AddComponent<ObiCollider>();
            }
        }

        // Create a SimPredBatch server
        rosConnector = ROSConnection.GetOrCreateInstance();
        rosConnector.ImplementService<SimPredBatchRequest, SimPredBatchResponse>(predServiceName, predict);
        
    }

    void initialiate(List<Vector3> gripperPoses, List<Vector3> particlePositions, List<float> ropeParams) {
        
        // Create a set of grippers
        for (int i=0; i<numEnvs; i++) {
            GameObject leftGripper = GameObject.CreatePrimitive(PrimitiveType.Cube);
            leftGripper.transform.localScale = Vector3.zero;
            leftGripper.transform.parent = envs[i].transform;
            leftGripper.GetComponent<MeshRenderer>().enabled = false;
            leftGripper.name = "leftGripper"+i.ToString();
            leftGrippers.Add(leftGripper);
            GameObject rightGripper = GameObject.CreatePrimitive(PrimitiveType.Cube);
            rightGripper.transform.localScale = Vector3.zero;
            rightGripper.transform.parent = envs[i].transform;
            rightGripper.GetComponent<MeshRenderer>().enabled = false;
            rightGripper.name = "rightGripper"+i.ToString();
            rightGrippers.Add(rightGripper);
        }

        // Create a set of ropes
        // store the starting time
        float startTime = Time.realtimeSinceStartup;
        // for (int i=0; i<numEnvs; i++) {
        //     // move points to the env location
        //     // List<Vector3> temp = new List<Vector3>(points);
        //     // for (int j=0; j<temp.Count; j++) {
        //     //     temp[j] += envs[i].transform.position;
        //     // }
        //     List<Vector3> temp = new List<Vector3>();
        //     foreach (var p in particlePositions) {
        //         temp.Add(envs[i].transform.TransformPoint(p));
        //     }
        //     // create a rope
        //     ObiRope rope = Generators.Rope(
        //         points:temp,
        //         material:ropeMaterial,
        //         collider_filter:ropeColliderFilter,
        //         collider_filter_end:ropeColliderFilterEnd,
        //         rope_radius:ropeRadius,
        //         resolution:ropeResolution,
        //         pooled_particles:ropePooledParticles,
        //         name:"rope"+i.ToString(),
        //         stretch_compliance:ropeStretchCompliance,
        //         stretching_scale:ropeStretchingScale,
        //         bend_compliance:ropeBendCompliance,
        //         max_bending:ropeMaxBending,
        //         mass:ropeMass,
        //         damping:ropeDamping,
        //         substeps:obiSubsteps,
        //         self_collision:ropeSelfCollision
        //     );
        //     rope.gameObject.name += " "+i.ToString();
        //     ropes.Add(rope);
        // }


        ObiRope ropeTemplate = Generators.Rope(
            points:particlePositions,
            material:ropeMaterial,
            collider_filter:ropeColliderFilter,
            collider_filter_end:ropeColliderFilterEnd,
            rope_radius:ropeRadius,
            resolution:ropeResolution,
            pooled_particles:ropePooledParticles,
            name:"",
            stretch_compliance:ropeStretchCompliance,
            stretching_scale:ropeStretchingScale,
            bend_compliance:ropeBendCompliance,
            max_bending:ropeMaxBending,
            mass:ropeMass,
            damping:ropeDamping,
            substeps:obiSubsteps,
            self_collision:ropeSelfCollision);
        GameObject ropeObject = ropeTemplate.gameObject;

        for (int i=0; i<numEnvs; i++) {
            // move points to the env location
            // List<Vector3> temp = new List<Vector3>(points);
            // for (int j=0; j<temp.Count; j++) {
            //     temp[j] += envs[i].transform.position;
            // }
            // List<Vector3> temp = new List<Vector3>();
            // foreach (var p in particlePositions) {
            //     temp.Add(envs[i].transform.TransformPoint(p));
            // }
            // copy a rope object
            var newRopeObj = Instantiate(ropeObject);
            newRopeObj.transform.position = envs[i].transform.position;
            ropes.Add(newRopeObj.GetComponent<ObiRope>());
            newRopeObj.name += " "+i.ToString();
        }

        // // create ropes
        // ropes = Generators.Ropes(
        //     points: particlePositions,
        //     material:ropeMaterial,
        //     collider_filter:ropeColliderFilter,
        //     collider_filter_end:ropeColliderFilterEnd,
        //     rope_radius:ropeRadius,
        //     resolution:ropeResolution,
        //     pooled_particles:ropePooledParticles,
        //     stretch_compliance:ropeStretchCompliance,
        //     stretching_scale:ropeStretchingScale,
        //     bend_compliance:ropeBendCompliance,
        //     max_bending:ropeMaxBending,
        //     mass:ropeMass,
        //     damping:ropeDamping,
        //     substeps:obiSubsteps,
        //     self_collision:ropeSelfCollision,
        //     env_locations: envs.Select(e => e.transform.position).ToList()
        // );

        // print time used
        Debug.Log("Elapsed time: " + (Time.realtimeSinceStartup - startTime) + "s");

        for (int i=0; i<numEnvs; i++) {
            // log the particle ids
            List<int> particleIds = new List<int>();
            List<int> controlledIds = new List<int>();
            ObiRope rope = ropes[i];
            // // number of elements is 1 less than the number of groups
            // foreach(var element in rope.elements) {
            //     controlledIds.Add(element.particle1);
            // }
            // controlledIds.Add(rope.elements[rope.elements.Count-1].particle2);
            // ropeControlledIds.Add(controlledIds);

            foreach (var group in rope.blueprint.groups) {
                foreach (int id in group.particleIndices)
                    particleIds.Add(rope.solverIndices[id]);
                controlledIds.Add(rope.solverIndices[group.particleIndices[0]]);
            }
            ropeParticleIds.Add(particleIds);
            ropeControlledIds.Add(controlledIds);

            // move grippers
            // leftGrippers[i].transform.position = temp[0];
            // rightGrippers[i].transform.position = temp[temp.Count-1];
            leftGrippers[i].transform.position = envs[i].transform.TransformPoint(particlePositions[0]);
            rightGrippers[i].transform.position = envs[i].transform.TransformPoint(particlePositions[particlePositions.Count-1]);
            leftGrippersTargets.Add(leftGrippers[i].transform.position);
            rightGrippersTargets.Add(rightGrippers[i].transform.position);

            // add gripper attachments
            ObiParticleAttachment leftGripperAttachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            // TODO change attached group id
            leftGripperAttachment.particleGroup = rope.blueprint.groups[0];
            leftGripperAttachment.target = leftGrippers[i].transform;
            leftGripperAttachment.enabled = true;
            leftGripperAttachments.Add(leftGripperAttachment);

            ObiParticleAttachment rightGripperAttachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            // TODO change attached group id
            rightGripperAttachment.particleGroup = rope.blueprint.groups[rope.blueprint.groups.Count-1];
            rightGripperAttachment.target = rightGrippers[i].transform;
            rightGripperAttachment.enabled = false;
            rightGripperAttachments.Add(rightGripperAttachment);

        }
        Debug.Log("Generated "+numEnvs+" "+ropes[0].restLength+"m long Obi Rope.");

        // calculate the time it took to generate the ropes
        float elapsedTime = Time.realtimeSinceStartup - startTime;
        Debug.Log("Elapsed time: " + elapsedTime);

        // update obi parameters
        GameObject solverObject = GameObject.Find("Obi Solver");
        obiSolver = solverObject.GetComponent<ObiSolver>();
        obiSolver.gravity = new Vector3(0,obiGravity,0);
        obiSolver.parameters.damping = obiDamping;
        obiSolver.PushSolverParameters();
        obiUpdater = solverObject.GetComponent<ObiFixedUpdater>();
        if (obiUpdater==null) // in case the updater is not added
            obiUpdater = solverObject.AddComponent<ObiFixedUpdater>();
            if (obiUpdater.solvers.Count==0)
                obiUpdater.solvers.Add(obiSolver); // add the solver to the updater:
        obiUpdater.substeps = obiSubsteps;

        initialised = true;
    }

    async Task<SimPredBatchResponse> predict(SimPredBatchRequest request) {
        SimPredBatchResponse response = new SimPredBatchResponse();
        if (!initialised) {
            // initialiate(request.gripper_poses_prev.poses.Select(p => p.position.From<FLU>()).ToList(),
            //             request.states_prev.poses.Select(p => p.position.From<FLU>()).ToList(),
            //             request.parameters.data.ToList());
            initialiate(request.gripper_poses_prev.poses.Select(p => PointMsg2Vector3(p.position)).ToList(),
                        request.states_prev.poses.Select(p => PointMsg2Vector3(p.position)).ToList(),
                        request.parameters.data.ToList());
        }
        else {
            for (int i=0; i<numEnvs; i++) {
                // change the simulation parameters
                ropes[i].bendCompliance = request.parameters.data[i*numParams];
                ropes[i].stretchCompliance = request.parameters.data[i*numParams+1];
                // ropes[i].damping = request.parameters.data[i*numParams+2];

                // reset rope states
                for (int j=0; j<ropeControlledIds[i].Count; j++) {
                    // TODO do the states from real rope obey sim constraints?
                    // var pos = envs[i].transform.TransformPoint(request.states_prev.poses[j].position.From<FLU>());
                    var pos = envs[i].transform.TransformPoint(PointMsg2Vector3(request.states_prev.poses[j].position));
                     obiSolver.positions[ropeControlledIds[i][j]] = 
                obiSolver.prevPositions[ropeControlledIds[i][j]] = 
                obiSolver.renderablePositions[ropeControlledIds[i][j]] = 
                obiSolver.startPositions[ropeControlledIds[i][j]] = pos;
                    obiSolver.velocities[ropeControlledIds[i][j]] = Vector4.zero;
                    obiSolver.angularVelocities[ropeControlledIds[i][j]] = Vector4.zero;

                obiSolver.invMasses[ropeControlledIds[i][j]] = 0; // stops particles from moving
                //     int id = ropes[i].solverIndices[ropes[i].blueprint.groups[j].particleIndices[0]];
                //     obiSolver.positions[id] = 
                // obiSolver.prevPositions[id] = 
                // obiSolver.renderablePositions[id] = 
                // obiSolver.startPositions[id] = pos;
                //     obiSolver.velocities[id] = Vector4.zero;
                //     obiSolver.angularVelocities[id] = Vector4.zero;
                }

                // read gripper states and positions
                // leftGrippers[i].transform.position = envs[i].transform.TransformPoint(request.gripper_poses_prev.poses[0].position.From<FLU>());
                // leftGrippersTargets[i] = envs[i].transform.TransformPoint(request.gripper_poses_curr.poses[0].position.From<FLU>());
                leftGrippers[i].transform.position = envs[i].transform.TransformPoint(PointMsg2Vector3(request.gripper_poses_prev.poses[0].position));
                leftGrippersTargets[i] = envs[i].transform.TransformPoint(PointMsg2Vector3(request.gripper_poses_curr.poses[0].position));
                //TODO snap gripper to the rope
                // int graspedGroup = FindGraspedParticleGroup(leftGrippers[i].transform.position, request.gripper_states.data[0], ropes[i], ropeControlledIds[i]);
                int graspedGroup = 0;
                leftGripperAttachments[i].particleGroup = ropes[i].blueprint.groups[graspedGroup];
                // leftGripperAttachments[i].enabled = request.gripper_states.data[0]<=ropeRadius;
            }

            // run the simulation
            predicting = true;
            while (predicting)
                await Task.Yield();

    while(!Input.GetKeyDown(KeyCode.Space))
    {
                await Task.Yield();
    }

            // return the rope states
            // for (int i=0; i<numEnvs; i++) {

            // }
        }
        return response;
    }

    private int FindGraspedParticleGroup(Vector3 gripperPos, float gripperOpening, ObiRope rope, List<int> particleIds) {
        int id = -1; // -1 for no particle
        if (gripperOpening>0.1f) return id;
        float minDist = Mathf.Infinity;
        // for (int i=0; i<states.Count; i++) {
        //     float dist = Vector3.Distance(states[i], gripperPos);
        for (int i=0; i<particleIds.Count; i++) {
            float dist = Vector3.Distance(rope.solver.positions[particleIds[i]], gripperPos);
            if (dist<minDist) {
                minDist = dist;
                id = i;
            }
        }
        return id;
    }

    List<Vector3> generate_init_poses(float angle, float rope_length, int num_control_points, float rope_radius, Vector3 rope_centre) {
        angle = angle*Mathf.Deg2Rad;
        List<Vector3> points = new List<Vector3>();
        Vector3 point = new Vector3(-rope_length/2*Mathf.Sin(angle), rope_radius, -rope_length/2*Mathf.Cos(angle))+rope_centre;
        float step = rope_length/num_control_points;
        points.Add(point);
        for (int i=0; i<num_control_points; i++) {
            point += new Vector3(step*Mathf.Sin(angle), 0.0f, step*Mathf.Cos(angle))+rope_centre;
            points.Add(point);
        }
        return points;
    }


    // Update is called once per frame
    void Update()
    {
        if (!initialised) return;
        if (predicting) {
            // reset particle invMasses
            for (int i=0; i<numEnvs; i++) {
                for (int j=0; j<ropeControlledIds[i].Count; j++) {
                    obiSolver.invMasses[ropeControlledIds[i][j]] = 1/ropeMass;
                }
            }
            // move grippers to the target positions
            for (int i=0; i<numEnvs; i++) {
                leftGrippers[i].transform.position = Vector3.MoveTowards(leftGrippers[i].transform.position, leftGrippersTargets[i], gripperTranslationSpeed);
                rightGrippers[i].transform.position = Vector3.MoveTowards(rightGrippers[i].transform.position, rightGrippersTargets[i], gripperTranslationSpeed);
            }
            predCounter++;
        }
        if (predCounter>predTime) {
            predicting = false;
            predCounter = 0;
        }
        
    }
    public static Vector3 PointMsg2Vector3(RosPoint v) => new Vector3((float)v.x, (float)v.y, (float)v.z);
}
