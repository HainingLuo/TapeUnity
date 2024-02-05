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
using RosPoseArray = RosMessageTypes.Geometry.PoseArrayMsg;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;
using RosMessageTypes.Tape;



public class Main : MonoBehaviour
{    
    // Unity params
    public float timeScale = 1.0f;
    public float timeStep = 0.02f;

    // obi rope
    private ObiRope rope;
    public string ropeName="";
    public Material ropeMaterial;
    public float ropeLength=13.8f;
    public float ropeRadius=0.03f;
    public float ropeResolution=0.2f;
    private int ropeColliderFilterEnd = ObiUtils.MakeFilter((1 << 0) | (1 << 1) | (1 << 3), 1);
    private int ropeColliderFilter = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything ^ (1 << 2), 0);
    // public int numControlPoints=10;
    // private List<Vector3> gravity_centres = new List<Vector3>();
    public int ropePooledParticles=0;
    public float ropeStretchCompliance=0.0f;
    public float ropeStretchingScale=0.0f;
    public float ropeBendCompliance=0.0f;
    public float ropeMaxBending=0.05f;
    public float ropeMass=0.05f;
    public float ropeDamping=0.0f;
    public int obiSubsteps=4;
    public float obiGravity=-9.8f;
    public float obiDamping=0.99f;
    public bool ropeSelfCollision=true;
    private ObiFixedUpdater updater;
    private List<ObiParticleAttachment> gripperAttachments = new List<ObiParticleAttachment>();

    // states
    private List<Vector3> states = new List<Vector3>(); // position of all particles
    private List<Vector3> statesEst = new List<Vector3>(); // tape estimated position of all particles
    private List<List<int>> particleIds = new List<List<int>>(); // particle ids of each group
    private GameObject stateMarkers;
    private List<GameObject> stateMarkersList = new List<GameObject>();
    public float adjustSpeed=1f; // mm

    // grippers
    private List<GameObject> grippersList = new List<GameObject>();
    private GameObject grippers;
    private List<Vector3> gripperTargets = new List<Vector3>();
    private List<Quaternion> gripperTargetOrients = new List<Quaternion>();
    private List<int> graspedParticles = new List<int>(); // id of particles grasped by grippers
    private List<bool> gripperGrasping = new List<bool>(); // whether grippers are grasping
    public float gripperTranslationSpeed=0.01f; // mm
    public float gripperRotationSpeed=0.1f; // deg
    private List<GameObject> gripperMarkers = new List<GameObject>();

    // ROS
    private ROSConnection rosConnector;
    private RosPoseArray poseArrayMsg = new RosPoseArray();
    public string predServiceName = "/unity_predict";
    public string adjustServiceName = "/unity_adjust";
    public string resetServiceName = "/unity_reset";
    
    // cuz im lazy
    private RosQuaternion emptyRosQuat;
    // // DEFINE SIMULATION PARAMETERS
    // private float publish_rate_control = 0;
    // private float update_time_step;
    // private int stepControl = 0;
    // private int stepTime = 20; // time = stepTime*update timestep (default:0.02s)
    // private Vector3 action_active;

    // System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
    // // public KeyCode screenshotKey;
    // WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();

    // Vector3 grasp_pt_pos = new Vector3(0,0,0);
    // Vector2 pixel_action = new Vector2(0,0);
    // Vector2 pixel_g = new Vector2(0,0);
    // Vector2 pixel_t = new Vector2(0,0);
    // Vector3 point_g = new Vector3(0,0,0);
    // Vector3 point_t = new Vector3(0,0,0);
    // int yaw_g = 0;
    // int action_yaw = 0;
    
    // Transform transform;

    private bool initialised=true;
    private bool adjusting=false;
    private int adjustCounter=0;
    public int adjustTime=10;
    private bool predicting=false;
    private int predictCounter=0;
    public int predictTime=10;


    async void Start()
    {
        // SET ROS CONNECTION
        rosConnector = ROSConnection.instance;
        rosConnector.ImplementService<SimPredRequest, SimPredResponse>(predServiceName, Predict);
        rosConnector.ImplementService<SimAdjustRequest, SimAdjustResponse>(adjustServiceName, Adjust);
        rosConnector.ImplementService<SimResetRequest, SimResetResponse>(resetServiceName, Reset);
        // TODO: (temp solution) add eyelet for testing only
        rosConnector.Subscribe<RosPoseArray>("/eyelet_poses", tempEyeletCallback);

        // initialise fake grippers
        grippers = new GameObject("Grippers");

        // cuz im lazy
        emptyRosQuat = new RosQuaternion();
        emptyRosQuat.x = 0;
        emptyRosQuat.y = 0;
        emptyRosQuat.z = 0;
        emptyRosQuat.w = 0;

        // SETUP SIMULATION PARAMETERS
        Time.timeScale = timeScale;
        Time.fixedDeltaTime = timeStep;
        // update_time_step = Time.fixedDeltaTime;particle_positions
        // Initialise grasping point
        // int gsp_pt_ptc_id = rope.blueprint.groups[grasp_pt].particleIndices[0];
        // point_g = (Vector3)rope.solver.positions[gsp_pt_ptc_id];

    }


    //////////// NOT SUPPOSED TO BE HERE ////////////
    private void tempEyeletCallback(RosPoseArray msg) {
        Debug.Log("Received eyelet poses!");

        // generate eyelets
        List<Transform> eyelets = new List<Transform>();
        GameObject eyelets_object = new GameObject("Eyelets");
        for (int i=0; i<msg.poses.Count(); i++) {
            int row = i/2;
            GameObject eyelet = createCapsuleEyelet("Eyelet "+i.ToString(), 
                                msg.poses[i].position.From<FLU>(),
                                msg.poses[i].orientation.From<FLU>(),
                                Vector3.one*0.06f,
                                eyelets_object.transform
                                );
            eyelets.Add(eyelet.transform);
        }

    }

    GameObject createCapsuleEyelet(string name, Vector3 position, Quaternion rotation, Vector3 scale, Transform parent) {
        // generate capsule eyelet
        GameObject eyelet = new GameObject("Eyelet"+name);
        int numCapsules=36;
        for (int i=0; i<numCapsules; i++) {
            GameObject capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            capsule.name = "Capsule"+i.ToString();
            capsule.transform.position = new Vector3(Mathf.Cos(360/numCapsules*i)*0.1f, Mathf.Sin(360/numCapsules*i)*0.1f, 0);
            capsule.transform.rotation = Quaternion.Euler(90, 0, 0); 
            capsule.transform.localScale = new Vector3(0.05f,0.05f,0.05f);
            capsule.transform.parent = eyelet.transform;
            ObiCollider collider = capsule.AddComponent<ObiCollider>();
        }
        eyelet.transform.position = position;
        eyelet.transform.rotation = rotation;
        eyelet.transform.localScale = scale;
        eyelet.transform.parent = parent;
        return eyelet;
    }
    /////////////////////////////////////////////////

    private async Task<SimResetResponse> Reset(SimResetRequest request) 
    {
        Debug.Log("Resetting the environment...");
        initialised = false;
        // prepare the response
        SimResetResponse response = new SimResetResponse();

        // destroy the rope and grippers
        if (initialised) {
            Destroy(rope.gameObject);
            rope = null;
            states.Clear();
            statesEst.Clear();
            particleIds.Clear();
            foreach (GameObject g in grippersList)
                Destroy(g);
            grippersList.Clear();
            foreach (ObiParticleAttachment attachment in gripperAttachments)
                Destroy(attachment);
            gripperAttachments.Clear();
            graspedParticles.Clear();
            gripperTargets.Clear();
            gripperTargetOrients.Clear();
            foreach (GameObject marker in gripperMarkers)
                Destroy(marker);
            gripperMarkers.Clear();
            foreach (GameObject marker in stateMarkersList)
                Destroy(marker);
            Destroy(stateMarkers);
            stateMarkersList.Clear();
        }

        // read the request
        stateMarkers = new GameObject("StateMarkers");
        for (int i=0; i<request.states_est.poses.Count(); i++) {
            RosPose pose = request.states_est.poses[i];
            states.Add(pose.position.From<FLU>());
            statesEst.Add(pose.position.From<FLU>());

            // add markers
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Marker"+i.ToString();
            marker.transform.position = states[i];
            marker.transform.localScale = Vector3.one*0.01f;
            marker.transform.parent = stateMarkers.transform;
            marker.GetComponent<SphereCollider>().enabled = false;
            marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker2");
            stateMarkersList.Add(marker);
        }
        ropeLength = request.rope_length.data;
        // ropeRadius = request.rope_radius.data;
        // generate initail rope config
        // List<Vector3> points = generate_init_poses(0, ropeLength, states.Count(), ropeRadius);

        // generate a new rope and grippers
        List<Vector3> points = new List<Vector3>(states);
        // add gripper positions to the rope
        points.Insert(0, request.gripper_poses.poses[0].position.From<FLU>());
        points.Add(request.gripper_poses.poses[1].position.From<FLU>());
        rope = Generators.Rope(
            points:points,
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
            self_collision:ropeSelfCollision
            );
        Debug.Log("Generated "+rope.restLength/10+"m long Obi Rope.");

        // update obi parameters
        rope.gameObject.transform.localScale = new Vector3(1f,1f,1f);
        rope.solver.gravity = new Vector3(0,obiGravity,0);
        rope.solver.parameters.damping = obiDamping;
        rope.solver.PushSolverParameters();
        ObiFixedUpdater updater = rope.gameObject.transform.parent.gameObject.GetComponent<ObiFixedUpdater>();
        if (updater==null)
            updater = rope.gameObject.transform.parent.gameObject.AddComponent<ObiFixedUpdater>();
            if (updater.solvers.Count==0)
                updater.solvers.Add(rope.solver); // add the solver to the updater:
        updater.substeps = obiSubsteps;

        // log the particle ids
        foreach (var group in rope.blueprint.groups) {
            List<int> ids = new List<int>();
            foreach (int id in group.particleIndices)
                ids.Add(id);
            particleIds.Add(ids);
        }
        // particleIds.RemoveAt(0); // remove the first gripper particle
        // particleIds.RemoveAt(particleIds.Count-1); // remove the last gripper particle
        // particleIds.Add(rope.blueprint.groups.First().particleIndices); // add the first gripper particle

        // add grippers
        for (int i=0; i<request.gripper_poses.poses.Count(); i++) {
            RosPose pose = request.gripper_poses.poses[i];
            GameObject gripper = new GameObject("Gripper"+i.ToString());
            gripper.transform.position = pose.position.From<FLU>();
            gripper.transform.rotation = pose.orientation.From<FLU>();
            gripper.transform.parent = grippers.transform;
            grippersList.Add(gripper);
            // add gripper targets
            gripperTargets.Add(gripper.transform.position);
            gripperTargetOrients.Add(gripper.transform.rotation);
            // add gripper attachment
            ObiParticleAttachment attachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            attachment.target = gripper.transform;
            // find the particle that should be attached to the gripper
            int id = FindGraspedParticleGroup(gripper.transform.position, request.gripper_states.data[i]);
            graspedParticles.Add(id);
            if (id>=0) {
                attachment.enabled = true;
                attachment.particleGroup = rope.blueprint.groups[id];
            }
            else
                attachment.enabled = false;
            gripperAttachments.Add(attachment);

            // add markers
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            marker.name = gripper.name+"Marker";
            marker.transform.position = gripper.transform.position;
            marker.transform.localScale = Vector3.one*0.01f;
            marker.transform.parent = grippers.transform;
            marker.GetComponent<BoxCollider>().enabled = false;
            marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker3");
            gripperMarkers.Add(marker);

        }
        initialised = true;

        // initial adjustment
        adjusting = true;
        while (adjusting)
            await Task.Yield();

        Debug.Log("Environment reset!");
        return response;
    }

    // List<Vector3> generate_init_poses(float angle, float rope_length, int num_control_points, float cable_radius) {
    //     List<Vector3> points = new List<Vector3>();
    //     Vector3 point = new Vector3(-rope_length/2*Mathf.Sin(angle)+2.5f, cable_radius, -rope_length/2*Mathf.Cos(angle));
    //     float step = rope_length/num_control_points;
    //     points.Add(point);
    //     for (int i=0; i<num_control_points; i++) {
    //         point += new Vector3(step*Mathf.Sin(angle), 0.0f, step*Mathf.Cos(angle));
    //         points.Add(point);
    //     }
    //     return points;
    // }
    
    private async Task<SimAdjustResponse> Adjust(SimAdjustRequest request)
    {
        Debug.Log("Adjusting the environment...");
        // TODO: check validity of the request

        // prepare the response
        SimAdjustResponse response = new SimAdjustResponse();
        
        // check if rope has been initialised
        if (!initialised) {
            Debug.Log("Environment not initialised yet!");
            return response;
        }

        // update state estimation
        statesEst.Clear();
        for (int i=0; i<request.states_est.poses.Count(); i++) {
            RosPose pose = request.states_est.poses[i];
            statesEst.Add(pose.position.From<FLU>());
            // update markers
            stateMarkersList[i].transform.position = statesEst[i];
        }

        // update the rope
        adjusting = true;
        while (adjusting)
            await Task.Yield();

        // generate the response
        List<RosPose> poseList = new List<RosPose>();
        for(int i=0; i<states.Count; i++) {
            // update the rope states
            states[i] = rope.solver.positions[particleIds[i+1][0]];
            poseList.Add(new RosPose(states[i].To<FLU>(), emptyRosQuat));
        }
        response.states_sim.poses = poseList.ToArray();
        
        // stepControl = 0;
        // stepTime = 10;
        // grasp_pt = 0;
        // int gsp_pt_ptc_id = rope.blueprint.groups[grasp_pt].particleIndices[0];
        // point_g = (Vector3)rope.solver.positions[gsp_pt_ptc_id];
        // action_active = new Vector3(0,0,0);
        // action_yaw = 0;
        // while (stepControl <= stepTime) {
        //     await Task.Yield();
        // }
        // response.success = new RosBool(true);
        Debug.Log("Environment adjusted!");
        return response;
    }

    private int FindGraspedParticleGroup(Vector3 gripperPos, float gripperOpening) {
        int id = -1; // -1 for no particle
        if (gripperOpening>0.1f) return id;
        float minDist = Mathf.Infinity;
        // for (int i=0; i<states.Count; i++) {
        //     float dist = Vector3.Distance(states[i], gripperPos);
        for (int i=0; i<rope.blueprint.groups.Count(); i++) {
            float dist = Vector3.Distance(rope.solver.positions[particleIds[i][0]], gripperPos);
            if (dist<minDist) {
                minDist = dist;
                id = i;
            }
        }
        return id;
    }

    private async Task<SimPredResponse> Predict(SimPredRequest request)
    {
        Debug.Log("Predicting the environment...");
        // prepare the response
        SimPredResponse response = new SimPredResponse();
        
        // check if rope has been initialised
        if (!initialised) {
            Debug.Log("Environment not initialised yet!");
            return response;
        }

        // update gripper targets
        for (int i=0; i<request.gripper_poses.poses.Count(); i++) {
            RosPose pose = request.gripper_poses.poses[i];
            gripperTargets[i] = pose.position.From<FLU>();
            gripperTargetOrients[i] = pose.orientation.From<FLU>();
            // update gripper speed to guarantee reaching the target in prediction time
            // gripperTranslationSpeed = Vector3.Distance(grippersList[i].transform.position, gripperTargets[i])/(predictTime-1);
            // gripperRotationSpeed = Quaternion.Angle(grippersList[i].transform.rotation, gripperTargetOrients[i])/(predictTime-1);
            gripperMarkers[i].transform.position = gripperTargets[i];
            // TODO grasped particles only change when gripper states change
            // graspedParticles[i] = FindGraspedParticleGroup(gripperTargets[i], request.gripper_states.data[i]);
        }

        // start prediction
        predicting = true;
        while (predicting)
            await Task.Yield();

        // generate the response
        List<RosPose> poseList = new List<RosPose>();
        for(int i=0; i<states.Count; i++) {
            // update the rope states
            states[i] = rope.solver.positions[particleIds[i+1][0]];
            poseList.Add(new RosPose(states[i].To<FLU>(), emptyRosQuat));
        }
        response.states_pred.poses = poseList.ToArray();

        
        // gripper.SetActive(true);  
        // // check if request is empty
        // if (request == null) return response;

        // // start processing the request 
        // pixel_g = new Vector2((int)request.pick.position.x, (int)request.pick.position.y);
        // pixel_t = new Vector2((int)request.place.position.x, (int)request.place.position.y);
        // pixel_action = pixel_t-pixel_g;

        // // generate image a and check the position of start and ending points
        // update_action();

        // // compute the active action
        // action_active = point_t - point_g;
        // yaw_g = (int)request.pick.orientation.z;
        // action_yaw = (int)request.place.orientation.z - yaw_g;
        // float action_length = Vector3.Distance(point_g, point_t);
        // // Debug.Log(action_length);
        // stepControl = 0;
        // updater.enabled = true;
        // stepTime = Mathf.Max(System.Convert.ToInt32(action_length/0.001f), 1);

        // set_gripper_to(grasp_pt);
        // // wait for defined step time
        // while (stepControl <= stepTime) {
        //     // Debug.Log("lets see "+cnt.ToString());
        //     // cnt++;
        //     await Task.Yield();
        // }

        // // generate image b 
        // // snapshot(); 
        // gripper.SetActive(false);  

        // response.success = true;
        Debug.Log("Environment predicted!");
        return response;
    }

    private void FixedUpdate()
    {
        if (!initialised) return;
        if (adjusting) {
            // add gravity to corresponding particles
            var particle_velocities = rope.solver.velocities.AsNativeArray<float4>();
            var particle_positions = rope.solver.positions.AsNativeArray<float4>();
            for (int i=0; i<statesEst.Count; i++) {
                float3 centre = statesEst[i];
                // // set particle positions to the estimated states
                // particle_positions[particleIds[i+1][0]] = new float4(centre, 1);

                // generate velocity for the control particles
                var vel = particle_velocities[particleIds[i+1][0]];
                vel.xyz += math.normalizesafe(centre - particle_positions[particleIds[i+1][0]].xyz) * math.distance(centre, particle_positions[particleIds[i+1][0]].xyz) * adjustSpeed * Time.deltaTime;
                // vel.xyz += math.normalizesafe(centre - particle_positions[particleIds[i+1][0]].xyz) * adjustSpeed * Time.deltaTime;
                particle_velocities[particleIds[i+1][0]] = vel;

                // loop through corresponding particle element
                // foreach (int id in particleIds[i]) {
                //     var vel = particle_velocities[id];
                //     vel.xyz += math.normalizesafe(centre - particle_positions[id].xyz) * math.distance(centre, particle_positions[id].xyz) * adjustSpeed * Time.deltaTime;
                //     particle_velocities[id] = vel;
                // }
            }
            adjustCounter++;
        }
        if (adjustCounter>adjustTime) {
            adjusting = false;
            adjustCounter = 0;
        }

        if (predicting) {
        
        float distance = Vector3.Distance(grippersList[0].transform.position, gripperTargets[0]) + 
                        Vector3.Distance(grippersList[1].transform.position, gripperTargets[1]);
        if (distance<0.01f) {
            predicting = false;
            predictCounter = 0;
            return;
        }
            // enable attachments
            if (predictCounter==0) {
                for (int i=0; i<gripperAttachments.Count; i++) {
                    if (graspedParticles[i]>=0) {
                        gripperAttachments[i].enabled = true;
                        gripperAttachments[i].particleGroup = rope.blueprint.groups[graspedParticles[i]];
                    }
                    else
                        gripperAttachments[i].enabled = false;
                }  
            }
            // move grippers gradually to targets
            for (int i=0; i<grippersList.Count; i++) {
                grippersList[i].transform.position = Vector3.MoveTowards(grippersList[i].transform.position, gripperTargets[i], gripperTranslationSpeed);
                grippersList[i].transform.rotation = Quaternion.RotateTowards(grippersList[i].transform.rotation, gripperTargetOrients[i], gripperRotationSpeed);
            }
            predictCounter++;
        }
        // if (predictCounter>predictTime) {
        //     predicting = false;
        //     predictCounter = 0;
        // }
    }

    // private void update_action() {
    //     // find target grasping point
    //     point_g = camera.ViewportToWorldPoint(new Vector3(pixel_g.x/pm.resolution_w, (pm.resolution_h-pixel_g.y)/pm.resolution_h, camera.transform.position.y-pm.cable_radius));
    //     float min_dist = Mathf.Infinity;
    //     for (int i=0; i<rope.blueprint.groups.Count; i++) {
    //         for (int j=0; j<rope.blueprint.groups[i].particleIndices.Count; j++) {
    //             int id = rope.blueprint.groups[i].particleIndices[j];
    //             float dist = Vector3.Distance((Vector3)rope.solver.positions[id], point_g);
    //             if (dist<min_dist) {
    //                 min_dist = dist;
    //                 grasp_pt = i;
    //             }
    //         }
    //     }

    //     // Get grasp point position in image space
    //     int gsp_pt_ptc_id = rope.blueprint.groups[grasp_pt].particleIndices[0];
    //     point_g = (Vector3)rope.solver.positions[gsp_pt_ptc_id]; // reset the grasping point to particle position
    //     grasp_pt_pos = point_g;
    //     grasp_pt_pos = camera.WorldToViewportPoint(grasp_pt_pos); // transfer to image space

    //     // // Get the yaw of the grasp point
    //     // Vector3 prev_pt_pos, next_pt_pos;
    //     // if (gsp_pt_ptc_id>0) {
    //     //     int prev_pt_ptc_id = gsp_pt_ptc_id-1;
    //     //     prev_pt_pos = (Vector3)rope.solver.positions[prev_pt_ptc_id];
    //     // }
    //     // else
    //     //     prev_pt_pos = grasp_pt_pos;

    //     // if (gsp_pt_ptc_id<rope.solver.positions.Count()-1) {
    //     //     int next_pt_ptc_id = gsp_pt_ptc_id+1;
    //     //     next_pt_pos = (Vector3)rope.solver.positions[next_pt_ptc_id];
    //     // }
    //     // else
    //     //     next_pt_pos = grasp_pt_pos;

    //     // float theta_g = Mathf.Atan((next_pt_pos.z-prev_pt_pos.z) / (next_pt_pos.x-prev_pt_pos.x)) * Mathf.Rad2Deg;
    //     // yaw_g = System.Convert.ToInt32(-theta_g);

    //     // get the end point in 3D
    //     pixel_g = new Vector2(grasp_pt_pos.x*pm.resolution_w, pm.resolution_h-grasp_pt_pos.y*pm.resolution_h);
    //     // Debug.Log("pixel_g: " + point_g.ToString("F4"));
    //     // Debug.Log(camera.ViewportToWorldPoint(new Vector3(pixel_g.x/pm.resolution_w, (pm.resolution_h-pixel_g.y)/pm.resolution_h, camera.transform.position.y-pm.cable_radius)).ToString("F4"));
    //     point_t = camera.ViewportToWorldPoint(new Vector3(pixel_t.x/pm.resolution_w, (pm.resolution_h-pixel_t.y)/pm.resolution_h, camera.transform.position.y-pm.cable_radius));

    //     // Debug.Log("==============================");
    //     // Debug.Log("2D distance "+ Vector2.Distance(pixel_t, pixel_g));
    //     // Debug.Log("3D distance "+ Vector3.Distance(point_t, point_g));

    // }

    // private void generate()
    // // IEnumerator generate(List<Vector3> points)
    // {
    //     Debug.Log("initialised Obi Rope.");
    //     // Generate obi rope
    //     List<Vector3> points = new List<Vector3>();
    //     // int init_pos_index = Random.Range(0, pm.init_poses.Count);
    //     int init_pos_index = 0; // fix position index
    //     // int init_pos_index = 2;
    //     yaw_g = -init_pos_index*45;
    //     points = pm.init_poses[init_pos_index];
    //     rope = Generators.Rope(
    //         points, 
    //         rope_mat, 
    //         pm.cable_radius, 
    //         pm.pooled_particles, 
    //         " Test",
    //         pm.stretch_compliance, 
    //         pm.stretching_scale, 
    //         pm.bend_compliance, 
    //         pm.max_bending, 
    //         pm.ropeMass,
    //         pm.ropeDamping);
    //     // rope = Generators.Rope(pm.init_pos, rope_mat, pm.cable_radius, pm.pooled_particles, " Test", 0.00f, 0.65f, 0.00f, 0.04f, 0.5f);
    //     rope.gameObject.transform.localScale = new Vector3(0.5f,0.5f,0.5f);

    //     // ObiRope rope1 = Generators.Rope(points, rope_mat, pm.cable_radius, pm.pooled_particles, " Test", 0.00f, 0.65f, 0.00f, 0.04f, 0.5f);
    //     // rope1.gameObject.transform.localScale = new Vector3(0.5f,0.5f,0.5f);

    //     updater = rope.gameObject.transform.parent.gameObject.AddComponent<ObiFixedUpdater>();
    //     updater.substeps = pm.updater_substeps;
    //     updater.solvers.Add(rope.gameObject.transform.parent.gameObject.GetComponent<ObiSolver>()); // add the solver to the updater:
        
    //     // Store initial positions
    //     foreach (Vector3 position in rope.solver.positions)
    //         states_initial.Add(position);

    //     attachment = rope.gameObject.AddComponent<ObiParticleAttachment>();

    //     gripper = new GameObject("Gripper");
    //     GameObject finger_1 = GameObject.CreatePrimitive(PrimitiveType.Cube);
    //     finger_1.name = "Finger1";
    //     finger_1.transform.localScale = new Vector3(0.01f,0.02f,0.005f);
    //     finger_1.GetComponent<MeshRenderer>().material = Resources.Load("Red", typeof(Material)) as Material;
    //     finger_1.AddComponent<ObiCollider>();
    //     finger_1.transform.position = new Vector3(0,0,-pm.cable_radius);
    //     finger_1.transform.parent = gripper.transform;

    //     GameObject finger_2 = GameObject.CreatePrimitive(PrimitiveType.Cube);
    //     finger_2.name = "Finger2";
    //     finger_2.transform.localScale = new Vector3(0.01f,0.02f,0.005f);
    //     finger_2.GetComponent<MeshRenderer>().material = Resources.Load("Red", typeof(Material)) as Material;
    //     finger_2.AddComponent<ObiCollider>();
    //     finger_2.transform.position = new Vector3(0,0,pm.cable_radius);
    //     finger_2.transform.parent = gripper.transform;
    //     set_gripper_to(0);
        
    //     // ADD A MONITORING CAMERA
    //     GameObject cameraObject = new GameObject("Camera", typeof(Camera));
    //     cameraObject.transform.position = pm.camera_position;
    //     cameraObject.transform.rotation = pm.camera_rotation;
    //     camera = cameraObject.GetComponent<Camera>();
    //     camera.targetDisplay = 4; // Render to display 5
    //     rt = new RenderTexture(pm.resolution_w, pm.resolution_h, 24);
    //     camera.targetTexture = rt;
    //     RenderTexture.active = rt;

    //     Debug.Log("Obi Rope initialised.");
    // }

    // public void set_gripper_to(int grasp_pt) {
    //     var group = rope.blueprint.groups[grasp_pt];
    //     int index = group.particleIndices[0];
    //     // cube.transform.position = rope.solver.transform.InverseTransformPoint(rope.solver.positions[index]);
    //     gripper.transform.position = rope.solver.positions[index];
    //     gripper.transform.rotation = Quaternion.Euler(0, yaw_g, 0); // Quaternion.Euler works in degrees
    //     attachment.target = gripper.transform;
    //     attachment.particleGroup = rope.blueprint.groups[grasp_pt];
    // }

    // public void move_gripper(Vector3 action, int yaw_delta) {        
    //     gripper.transform.position = point_g + action;
    //     gripper.transform.rotation = Quaternion.Euler(0, yaw_g+yaw_delta, 0);
    // }

    // private async Task<bool> WaitForParam() {
    //     while (pm.getting_param == true)
    //         await Task.Yield();
    //     return true;
    // }
}
