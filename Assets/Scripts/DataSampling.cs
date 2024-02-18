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



public class DataSampling : MonoBehaviour
{    
    // Unity params
    public float timeScale = 1.0f;
    public float timeStep = 0.02f;

    // obi rope
    private ObiRope rope;
    public string ropeName="";
    public Material ropeMaterial;
    public float ropeLength=13.8f;
    public Vector3 ropeCentre=new Vector3(0,0,0);
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
    private ObiFixedUpdater updater;
    // private List<ObiParticleAttachment> gripperAttachments = new List<ObiParticleAttachment>();
    private ObiParticleAttachment leftGripperAttachment;
    private ObiParticleAttachment rightGripperAttachment;
    private List<List<int>> ropeParticleIds = new List<List<int>>(); // particle ids of each group

    // grippers
    public float gripperHeight=0.138f;
    public GameObject gripperLeft;
    public GameObject gripperRight;
    private bool gripperLeftClosed=false;
    private bool gripperRightClosed=false;
    public float gripperTranslationSpeed=0.01f; // mm
    public float gripperRotationSpeed=0.1f; // deg

    // ROS
    private ROSConnection rosConnector;
    private RosPoseArray ropeStateMsg = new RosPoseArray();
    private RosPoseArray gripperPoseMsg = new RosPoseArray();
    private RosFloatArray gripperStateMsg = new RosFloatArray();
    
    // publish parameters
    public int publishRate = 10;
    private float publishRateControl = 0;
    private float publishInterval = 0;

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

    private bool initialised=false;


    async void Start()
    {
        // SET ROS CONNECTION
        rosConnector = ROSConnection.instance;
        // register groundtruth publishers
        rosConnector.RegisterPublisher<RosPoseArray>("/groundtruth/rope_states");
        rosConnector.RegisterPublisher<RosPoseArray>("/groundtruth/gripper_poses");
        rosConnector.RegisterPublisher<RosFloatArray>("/groundtruth/gripper_states");

        // generate rope
        List<Vector3> points = generate_init_poses(90, ropeLength, ropeNumGroups, ropeRadius, ropeCentre);    
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
        Debug.Log("Generated "+rope.restLength+"m long Obi Rope.");

        // update obi parameters
        rope.gameObject.transform.localScale = new Vector3(1f,1f,1f);
        rope.solver.gravity = new Vector3(0,obiGravity,0);
        rope.solver.parameters.damping = obiDamping;
        rope.solver.PushSolverParameters();
        ObiFixedUpdater updater = rope.gameObject.transform.parent.gameObject.GetComponent<ObiFixedUpdater>();
        if (updater==null) // in case the updater is not added
            updater = rope.gameObject.transform.parent.gameObject.AddComponent<ObiFixedUpdater>();
            if (updater.solvers.Count==0)
                updater.solvers.Add(rope.solver); // add the solver to the updater:
        updater.substeps = obiSubsteps;

        // log the particle ids
        foreach (var group in rope.blueprint.groups) {
            List<int> ids = new List<int>();
            foreach (int id in group.particleIndices)
                ids.Add(id);
            ropeParticleIds.Add(ids);
        }
        // particleIds.RemoveAt(0); // remove the first gripper particle
        // particleIds.RemoveAt(particleIds.Count-1); // remove the last gripper particle
        // particleIds.Add(rope.blueprint.groups.First().particleIndices); // add the first gripper particle

        // move grippers to the ends of the rope
        Vector3 ropeEnd1 = rope.solver.positions[ropeParticleIds.First().First()];
        Vector3 ropeEnd2 = rope.solver.positions[ropeParticleIds.Last().Last()];
        gripperLeft.transform.position = new Vector3(ropeEnd1.x, gripperHeight, ropeEnd1.z);
        gripperRight.transform.position = new Vector3(ropeEnd2.x, gripperHeight, ropeEnd2.z);

        // gripper attachments
        leftGripperAttachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
        leftGripperAttachment.target = gripperLeft.transform.Find("Tip").transform;
        leftGripperAttachment.particleGroup = rope.blueprint.groups.First();
        // leftGripperAttachment.enabled = false;

        rightGripperAttachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
        rightGripperAttachment.target = gripperRight.transform.Find("Tip").transform;
        rightGripperAttachment.particleGroup = rope.blueprint.groups.Last();
        // rightGripperAttachment.enabled = false;

        // initialise ros messages
        ropeStateMsg.poses = new RosPose[ropeNumGroups];
        gripperPoseMsg.poses = new RosPose[2];
        gripperStateMsg.data = new float[2];

        // set initialisation flag
        initialised = true;

        // cuz im lazy
        emptyRosQuat = new RosQuaternion();
        emptyRosQuat.x = 0;
        emptyRosQuat.y = 0;
        emptyRosQuat.z = 0;
        emptyRosQuat.w = 0;

        // SETUP SIMULATION PARAMETERS
        Time.timeScale = timeScale;
        Time.fixedDeltaTime = timeStep;
        publishInterval = 1.0f/publishRate;

        // Initialise grasping point
        // int gsp_pt_ptc_id = rope.blueprint.groups[grasp_pt].particleIndices[0];
        // point_g = (Vector3)rope.solver.positions[gsp_pt_ptc_id];

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

    // private int FindGraspedParticleGroup(Vector3 gripperPos, float gripperOpening) {
    //     int id = -1; // -1 for no particle
    //     if (gripperOpening>0.1f) return id;
    //     float minDist = Mathf.Infinity;
    //     // for (int i=0; i<states.Count; i++) {
    //     //     float dist = Vector3.Distance(states[i], gripperPos);
    //     for (int i=0; i<rope.blueprint.groups.Count(); i++) {
    //         float dist = Vector3.Distance(rope.solver.positions[particleIds[i][0]], gripperPos);
    //         if (dist<minDist) {
    //             minDist = dist;
    //             id = i;
    //         }
    //     }
    //     return id;
    // }

    private void FixedUpdate()
    {
        if (!initialised) return;

        // wait for the next publish interval
        if (publishRateControl<publishInterval) {
            publishRateControl += Time.fixedDeltaTime;
        }
        else {
            publishRateControl = 0;
            // update the rope states
            for (int i=0; i<ropeNumGroups; i++) {
                ropeStateMsg.poses[i] = new RosPose();
                ropeStateMsg.poses[i].position = new RosPoint();
                ropeStateMsg.poses[i].position.x = rope.solver.positions[ropeParticleIds[i][0]].x;
                ropeStateMsg.poses[i].position.y = rope.solver.positions[ropeParticleIds[i][0]].y;
                ropeStateMsg.poses[i].position.z = rope.solver.positions[ropeParticleIds[i][0]].z;
                ropeStateMsg.poses[i].orientation = emptyRosQuat;
            }
            rosConnector.Publish("/groundtruth/rope_states", ropeStateMsg);

            // update the gripper poses
            gripperPoseMsg.poses[0] = new RosPose();
            gripperPoseMsg.poses[0].position = new RosPoint();
            gripperPoseMsg.poses[0].position.x = gripperLeft.transform.position.x;
            gripperPoseMsg.poses[0].position.y = gripperLeft.transform.position.y;
            gripperPoseMsg.poses[0].position.z = gripperLeft.transform.position.z;
            gripperPoseMsg.poses[0].orientation = emptyRosQuat;
            gripperPoseMsg.poses[1] = new RosPose();
            gripperPoseMsg.poses[1].position = new RosPoint();
            gripperPoseMsg.poses[1].position.x = gripperRight.transform.position.x;
            gripperPoseMsg.poses[1].position.y = gripperRight.transform.position.y;
            gripperPoseMsg.poses[1].position.z = gripperRight.transform.position.z;
            gripperPoseMsg.poses[1].orientation = emptyRosQuat;
            rosConnector.Publish("/groundtruth/gripper_poses", gripperPoseMsg);

            // update the gripper states
            gripperStateMsg.data[0] = gripperLeftClosed?1:0;
            gripperStateMsg.data[1] = gripperRightClosed?1:0;
            rosConnector.Publish("/groundtruth/gripper_states", gripperStateMsg);
        }

        // move the grippers
        Vector3 leftGripperAction = new Vector3(Input.GetAxis("HorizontalLeftGripper"), 
            Input.GetAxis("JumpLeftGripper"), 
            Input.GetAxis("VerticalLeftGripper"))*gripperTranslationSpeed;
        gripperLeft.transform.position+=leftGripperAction;
        // cap the gripper position
        gripperLeft.transform.position = new Vector3(
            Mathf.Clamp(gripperLeft.transform.position.x, -1f, 1f),
            Mathf.Clamp(gripperLeft.transform.position.y, gripperHeight, 1f),
            Mathf.Clamp(gripperLeft.transform.position.z, -0.8f, 0.8f)
        );
        Vector3 rightGripperAction = new Vector3(Input.GetAxis("HorizontalRightGripper"), 
            Input.GetAxis("JumpRightGripper"), 
            Input.GetAxis("VerticalRightGripper"))*gripperTranslationSpeed;
        gripperRight.transform.position+=rightGripperAction;
        // cap the gripper position
        gripperRight.transform.position = new Vector3(
            Mathf.Clamp(gripperRight.transform.position.x, -1f, 1f),
            Mathf.Clamp(gripperRight.transform.position.y, gripperHeight, 1f),
            Mathf.Clamp(gripperRight.transform.position.z, -0.8f, 0.8f)
        );

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
