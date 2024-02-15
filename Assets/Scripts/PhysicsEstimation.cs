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
    public float envSpacing = 3.5f;
    private List<GameObject> envs=new List<GameObject>();

    // rope variables
    private List<ObiRope> ropes=new List<ObiRope>();
    private List<string> ropeNames;
    private List<Vector3> ropeCentres=new List<Vector3>();
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

    // gripper variables
    private List<GameObject> leftGrippers=new List<GameObject>();
    private List<GameObject> rightGrippers=new List<GameObject>();
    // private List<ObiParticleAttachment> gripperAttachments = new List<ObiParticleAttachment>();
    private ObiParticleAttachment leftGripperAttachment;
    private ObiParticleAttachment rightGripperAttachment;


    // ROS variables
    private ROSConnection rosConnector;
    private string predServiceName = "/unity_predict";




    void Start()
    {
        // Create a set of envs
        int envRows = (int)Mathf.Sqrt(numEnvs);
        for (int i=0; i<envRows; i++) {
            for (int j=0; j<envRows; j++) {
                int idx = i*envRows+j;
                if (idx>=numEnvs) break;
                GameObject env = new GameObject("env"+idx.ToString());
                env.transform.position = 
                        new Vector3((j-envRows/2)*envSpacing, 0, -(i-envRows/2)*envSpacing);
                envs.Add(env);
                // add a plane
                GameObject plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
                plane.transform.position = env.transform.position;
                plane.transform.localScale = new Vector3(0.3f, 1.0f, 0.3f);
                plane.transform.parent = env.transform;
                plane.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Black");
                plane.AddComponent<ObiCollider>();
            }
        }
        
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
        List<Vector3> points = generate_init_poses(90, ropeLength, ropeNumGroups, ropeRadius, Vector3.zero);
        for (int i=0; i<numEnvs; i++) {
            // move points to the env location
            List<Vector3> temp = new List<Vector3>(points);
            for (int j=0; j<temp.Count; j++) {
                temp[j] += envs[i].transform.position;
            }
            // create a rope
            ObiRope rope = Generators.Rope(
                points:temp,
                material:ropeMaterial,
                collider_filter:ropeColliderFilter,
                collider_filter_end:ropeColliderFilterEnd,
                rope_radius:ropeRadius,
                resolution:ropeResolution,
                pooled_particles:ropePooledParticles,
                name:"rope"+i.ToString(),
                stretch_compliance:ropeStretchCompliance,
                stretching_scale:ropeStretchingScale,
                bend_compliance:ropeBendCompliance,
                max_bending:ropeMaxBending,
                mass:ropeMass,
                damping:ropeDamping,
                substeps:obiSubsteps,
                self_collision:ropeSelfCollision
            );
            rope.gameObject.name += " "+i.ToString();
            ropes.Add(rope);

            // // log the particle ids
            // foreach (var group in rope.blueprint.groups) {
            //     List<int> ids = new List<int>();
            //     foreach (int id in group.particleIndices)
            //         ids.Add(id);
            //     ropeParticleIds.Add(ids);
            // }

            // move grippers
            leftGrippers[i].transform.position = temp[0];
            rightGrippers[i].transform.position = temp[temp.Count-1];

            // add gripper attachments
            leftGripperAttachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            // TODO change attached group id
            leftGripperAttachment.particleGroup = rope.blueprint.groups[0];
            leftGripperAttachment.target = leftGrippers[i].transform;
            rightGripperAttachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            // TODO change attached group id
            rightGripperAttachment.particleGroup = rope.blueprint.groups[rope.blueprint.groups.Count-1];
            rightGripperAttachment.target = rightGrippers[i].transform;

        }
        Debug.Log("Generated "+numEnvs+" "+ropes[0].restLength/10+"m long Obi Rope.");

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

        // Create a SimPredBatch server
        rosConnector = ROSConnection.GetOrCreateInstance();
        rosConnector.ImplementService<SimPredBatchRequest, SimPredBatchResponse>(predServiceName, predict);
        
    }

    SimPredBatchResponse predict(SimPredBatchRequest request) {
        // change the simulation parameters

        // reset rope states

        // read gripper states and positions

        // run the simulation

        // return the rope states
        SimPredBatchResponse response = new SimPredBatchResponse();
        return response;
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
        
    }
}
