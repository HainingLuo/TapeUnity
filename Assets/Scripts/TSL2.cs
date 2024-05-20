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
using RosIntArray = RosMessageTypes.Std.Int32MultiArrayMsg;



public class TSL2 : MonoBehaviour
{    
    // demo camera
    public Camera camera;
    private float rotateCounter = 360;

    // Unity params
    public float timeScale = 1.0f;
    public float timeStep = 0.02f;

    // obi rope
    private ObiRope rope;
    public string ropeName="";
    public Material ropeMaterial;
    private float ropeLength=13.8f;
    public float ropeRadius=0.03f;
    public float ropeResolution=0.2f;
    private int ropeColliderFilterEnd = ObiUtils.MakeFilter((1 << 0) | (1 << 1) | (1 << 3), 1);
    private int ropeColliderFilter = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything ^ (1 << 2), 0);
    // public int numControlPoints=10;
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
    private List<ObiParticleAttachment> gripperAttachments = new List<ObiParticleAttachment>();
    public Vector3 ropePID = new Vector3(0.1f, 0.1f, 0.1f);
    private List<Vector3> ropePIDIntegral = new List<Vector3>();
    private List<Vector3> ropePIDLastErrors = new List<Vector3>();

    // states
    private List<Vector3> states = new List<Vector3>(); // position of all particles
    private List<Vector3> statesEst = new List<Vector3>(); // tape estimated position of all particles
    private List<List<int>> particleIds = new List<List<int>>(); // particle ids of each group
    private GameObject stateMarkers;
    private List<GameObject> stateMarkersList = new List<GameObject>();
    public float adjustSpeed=1f; // mm
    private int numStates;
    private List<Vector3> adjustForcePrev = new List<Vector3>();

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
    public Vector3 gripperPID = new Vector3(0.1f, 0.1f, 0.1f);
    private List<Vector3> gripperPIDIntegral = new List<Vector3>();
    private List<Vector3> gripperPIDLastErrors = new List<Vector3>();

    // eyelets
    List<Transform> eyelets = new List<Transform>();
    private int active_eyelet = -1;
    private int active_eyelet_l = -1;
    private int active_eyelet_r = -1;
    private List<Vector3> target_eyelet_position = new List<Vector3>();
    private List<Quaternion> target_eyelet_rotation = new List<Quaternion>(); 


    // ROS
    private ROSConnection rosConnector;
    private RosPoseArray poseArrayMsg = new RosPoseArray();
    public string predServiceName = "/unity_predict";
    public string adjustServiceName = "/unity_adjust";
    public string resetServiceName = "/unity_reset";
    private Transform cam2rob;
    
    // cuz im lazy
    private RosQuaternion emptyRosQuat;
    // // DEFINE SIMULATION PARAMETERS
    // private float publish_rate_control = 0;
    // private float update_time_step;
    // private int stepControl = 0;
    // private int stepTime = 20; // time = stepTime*update timestep (default:0.02s)
    // private Vector3 action_active;
    
    public Vector3[] eyelet_offset = new Vector3[4] {
        new Vector3(0,0,0),
        new Vector3(0,0,0),
        new Vector3(0,0,0),
        new Vector3(0,0,0)
    };

    private bool initialised=false;
    private bool adjusting=false;
    private int adjustCounter=0;
    public int adjustTime=10;
    private bool predicting=false;
    private int predictCounter=0;
    public int predictTime=10;
    private float adjustErrorPrev=0;
    private float predErrorPrev=0;
    public float adjustDistThresh=0.001f;
    public float predDistThresh=0.001f;

    // stopwatch
    private System.Diagnostics.Stopwatch timer;


    private FpsDisplay fpsDisplay;


    async void Start()
    {
        fpsDisplay = gameObject.AddComponent<FpsDisplay>();
        fpsDisplay.showFPS = true;
        // Limit framerate to cinematic 24fps.
        QualitySettings.vSyncCount = 0; // Set vSyncCount to 0 so that using .targetFrameRate is enabled.
        Application.targetFrameRate = 120;

        // SET ROS CONNECTION
        rosConnector = ROSConnection.instance;
        rosConnector.ImplementService<SimAdjustRequest, SimAdjustResponse>(adjustServiceName, Adjust);
        rosConnector.ImplementService<SimResetRequest, SimResetResponse>(resetServiceName, Reset);
        // TODO: (temp solution) add eyelet for testing only
        rosConnector.Subscribe<RosPoseArray>("/eyelet_init", tempEyeletCallback);
        rosConnector.Subscribe<RosIntArray>("/cursor", cursorCallback);
        rosConnector.Subscribe<RosPoseArray>("/eyelet_pose", eyeletCallback);
        rosConnector.Subscribe<RosPoseArray>("/aglet_pose", agletCallback);
        cam2rob = new GameObject("cam2rob").transform;

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
        Debug.Log("Simulation started!");
        fpsDisplay.sysMessage = "Status: Ready";
    }


    //////////// NOT SUPPOSED TO BE HERE ////////////
    private void tempEyeletCallback(RosPoseArray msg) {
        Debug.Log("Received eyelet poses!");

        // generate eyelets
        GameObject eyelets_object = new GameObject("Eyelets");
        for (int i=0; i<msg.poses.Count(); i++) {
            int row = i/2;
            GameObject eyelet = createRopeEyelet(i.ToString(), 
                                msg.poses[i].position.From<FLU>()+eyelet_offset[i],
                                msg.poses[i].orientation.From<FLU>(),
                                Vector3.one*0.06f,
                                eyelets_object.transform
                                );
            // GameObject eyelet = createCapsuleEyelet("Eyelet "+i.ToString(), 
            //                     msg.poses[i].position.From<FLU>()+eyelet_offset[i],
            //                     msg.poses[i].orientation.From<FLU>(),
            //                     Vector3.one*0.06f,
            //                     eyelets_object.transform
            //                     );
            eyelets.Add(eyelet.transform);
            target_eyelet_position.Add(eyelet.transform.position);
            target_eyelet_rotation.Add(eyelet.transform.rotation);
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

    GameObject createRopeEyelet(string name, Vector3 position, Quaternion rotation, Vector3 scale, Transform parent) {
        // generate rope eyelet
        GameObject eyelet = new GameObject("Eyelet"+name);
        // eyelet.transform.localScale = scale;
        eyelet.transform.parent = parent;
        // generate points in a loop
        int eyeletResolution = 6;
        List<Vector3> points = new List<Vector3>();
        for (int i=0; i<eyeletResolution; i++) {
            Vector3 point = new Vector3(Mathf.Cos(Mathf.PI*2/eyeletResolution*i)*0.006f, Mathf.Sin(Mathf.PI*2/eyeletResolution*i)*0.006f, 0);
            // point+=position;
            points.Add(point);
        }
        points.Add(points[0]);
        ObiRope rope = Generators.Rope(
            points:points,
            material:ropeMaterial,
            collider_filter:ropeColliderFilter,
            collider_filter_end:ropeColliderFilterEnd,
            rope_radius:0.001f,
            resolution:ropeResolution,
            pooled_particles:ropePooledParticles,
            name:name,
            stretch_compliance:ropeStretchCompliance,
            stretching_scale:ropeStretchingScale,
            bend_compliance:ropeBendCompliance,
            max_bending:ropeMaxBending,
            mass:ropeMass,
            damping:ropeDamping,
            substeps:obiSubsteps,
            self_collision:false
            );
        Debug.Log("Generated "+rope.restLength+"m long Obi Rope with "+ rope.blueprint.groups.Count + " groups and "+rope.solverIndices.Length+" particles.");        
        // change rope material to white.mat
        rope.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/White");
        // // change the invmass of all particles to 0
        // for (int i=0; i<rope.solverIndices.Length; i++) {
        //     int index = rope.solverIndices[i];
        //     rope.solver.invMasses[index] = 0;
        // }
        // rope.gameObject.transform.parent = eyelet.transform;
        // change collision layer
        rope.gameObject.layer = LayerMask.NameToLayer("eyelet"+name);
        // add attachment to all groups
        for (int i=0; i<rope.blueprint.groups.Count(); i++) {
            ObiParticleAttachment attachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            attachment.target = eyelet.transform;
            attachment.particleGroup = rope.blueprint.groups[i];
        }
        eyelet.transform.position = position;
        eyelet.transform.rotation = rotation;
        return eyelet;
    }

    void eyeletCallback(RosMessageTypes.Geometry.PoseArrayMsg eyelet_poses)
    {
        if (eyelet_poses != null)
        {
            for (int i=0; i<eyelet_poses.poses.Count(); i++) {
                if (eyelet_poses.poses[i].position.x == 0 || double.IsNaN(eyelet_poses.poses[i].position.x)) continue;
                target_eyelet_position[i] = eyelet_poses.poses[i].position.From<FLU>();
                // target_eyelet_rotation[i] = eyelet_poses.poses[i].orientation.From<FLU>()*Quaternion.Euler(90,0,0)*Quaternion.Euler(0,90,0);
                target_eyelet_rotation[i] = eyelet_poses.poses[i].orientation.From<FLU>();
            } 
        }
    }
    void cursorCallback(RosIntArray cursors)
    {
        if (cursors != null)
        {
            // active_eyelet = cursors.data[0];
            active_eyelet_l = cursors.data[0];
            active_eyelet_r = cursors.data[1];
            Debug.Log("changing active eyelet to "+active_eyelet_l.ToString()+" "+active_eyelet_r.ToString());
        }
    }

    void agletCallback(RosMessageTypes.Geometry.PoseArrayMsg aglet_poses)
    {
        // Debug.Log("Predicting the environment...");
        
        // check if rope has been initialised
        if (!initialised) {
            Debug.Log("Environment not initialised yet!");
            return;
        }

        // update gripper targets
        for (int i=0; i<aglet_poses.poses.Count(); i++) {
            RosPose pose = aglet_poses.poses[i];
            gripperTargets[i] = pose.position.From<FLU>();
            gripperTargetOrients[i] = pose.orientation.From<FLU>();
            // gripperTargets[i] = cam2rob.TransformPoint(pose.position.From<FLU>());
            // gripperTargetOrients[i] = cam2rob.rotation*pose.orientation.From<FLU>();
            // update gripper speed to guarantee reaching the target in prediction time
            // gripperTranslationSpeed = Vector3.Distance(grippersList[i].transform.position, gripperTargets[i])/(predictTime-1);
            // gripperRotationSpeed = Quaternion.Angle(grippersList[i].transform.rotation, gripperTargetOrients[i])/(predictTime-1);
            gripperMarkers[i].transform.position = gripperTargets[i];
            // TODO grasped particles only change when gripper states change
            // graspedParticles[i] = FindGraspedParticleGroup(gripperTargets[i], request.gripper_states.data[i]);
            gripperPIDIntegral[i] = Vector3.zero;
            gripperPIDLastErrors[i] = gripperMarkers[i].transform.position - grippersList[i].transform.position;
        }

        // start prediction
        predicting = true;
        predErrorPrev = 0;
        // while (predicting)
        //     await Task.Yield();
    }
    /////////////////////////////////////////////////

    private async Task<SimResetResponse> Reset(SimResetRequest request) 
    {
        fpsDisplay.sysMessage = "Status: Resetting";
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
            gripperPIDIntegral.Clear();
            gripperPIDLastErrors.Clear();
            ropePIDIntegral.Clear();
            ropePIDLastErrors.Clear();
            adjustErrorPrev = 0;
            predErrorPrev = 0;
            adjustForcePrev.Clear();
        }

        // read the request
        cam2rob.position = request.cam2rob.translation.From<FLU>();
        cam2rob.rotation = request.cam2rob.rotation.From<FLU>();
        stateMarkers = new GameObject("StateMarkers");
        numStates = request.states_est.poses.Count();
        for (int i=0; i<numStates; i++) {
            RosPose pose = request.states_est.poses[i];
            states.Add(cam2rob.TransformPoint(pose.position.From<FLU>()));
            statesEst.Add(cam2rob.TransformPoint(pose.position.From<FLU>()));

            // add markers
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Marker"+i.ToString();
            marker.transform.position = states[i];
            marker.transform.localScale = Vector3.one*0.01f;
            marker.transform.parent = stateMarkers.transform;
            marker.GetComponent<SphereCollider>().enabled = false;
            marker.GetComponent<MeshRenderer>().material = Resources.Load<Material>("Materials/Marker2");
            // add ridigbody for pin constraints
            // marker.AddComponent<ObiCollider>();
            stateMarkersList.Add(marker);
        }
        ropeLength = request.rope_length.data;
        // ropeRadius = request.rope_radius.data;
        // generate initail rope config
        // List<Vector3> points = generate_init_poses(0, ropeLength, states.Count(), ropeRadius);

        // generate a new rope and grippers
        List<Vector3> points = new List<Vector3>(states);
        // add gripper positions to the rope
        points.Insert(0, cam2rob.TransformPoint(request.gripper_poses.poses[0].position.From<FLU>()));
        points.Add(cam2rob.TransformPoint(request.gripper_poses.poses[1].position.From<FLU>()));
        rope = Generators.Rope(
            points:points,
            material:ropeMaterial,
            collider_filter:ropeColliderFilter,
            collider_filter_end:ropeColliderFilterEnd,
            rope_radius:ropeRadius,
            resolution:0f,
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
        Debug.Log("Generated "+rope.restLength+"m long Obi Rope with "+ rope.blueprint.groups.Count + " groups.");

        rope.gameObject.layer = LayerMask.NameToLayer("shoelace");
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
                ids.Add(rope.solverIndices[id]);
            particleIds.Add(ids);
            ropePIDIntegral.Add(Vector3.zero);
            ropePIDLastErrors.Add(Vector3.zero);
            adjustForcePrev.Add(Vector3.zero);
        }
        particleIds.RemoveAt(0); // remove the first gripper particle
        particleIds.RemoveAt(numStates-1); // remove the last gripper particle
        // particleIds.Add(rope.blueprint.groups.First().particleIndices); // add the first gripper particle

        // add grippers
        for (int i=0; i<request.gripper_poses.poses.Count(); i++) {
            RosPose pose = request.gripper_poses.poses[i];
            GameObject gripper = new GameObject("Gripper"+i.ToString());
            gripper.transform.position = cam2rob.TransformPoint(pose.position.From<FLU>());
            gripper.transform.rotation = cam2rob.rotation*pose.orientation.From<FLU>();
            gripper.transform.parent = grippers.transform;
            gripper.layer = LayerMask.NameToLayer("aglets");
            grippersList.Add(gripper);
            // add gripper targets
            gripperTargets.Add(gripper.transform.position);
            gripperTargetOrients.Add(gripper.transform.rotation);
            gripperPIDIntegral.Add(Vector3.zero);
            // add gripper rigidbody
            Rigidbody rb = gripper.AddComponent<Rigidbody>();
            rb.isKinematic = false;
            rb.useGravity = false;
            // add gripper attachment
            ObiParticleAttachment attachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
            attachment.target = gripper.transform;
            // find the particle that should be attached to the gripper
            // int id = FindGraspedParticleGroup(gripper.transform.position, request.gripper_states.data[i]);
            int id;
            if (i==0)
                id = 0;
            else
                id = rope.blueprint.groups.Count()-1;
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
            gripperPIDLastErrors.Add(gripperMarkers[i].transform.position - grippersList[i].transform.position);
        }
        initialised = true;

        // initial adjustment
        adjusting = true;
        while (adjusting)
            await Task.Yield();

        Debug.Log("Environment reset!");
        fpsDisplay.sysMessage = "Status: Ready";
        return response;
    }
    
    private async Task<SimAdjustResponse> Adjust(SimAdjustRequest request)
    {
        // Debug.Log("Adjusting the environment...");
        // TODO: check validity of the request

        // time the function
        timer = System.Diagnostics.Stopwatch.StartNew();

        // prepare the response
        SimAdjustResponse response = new SimAdjustResponse();
        
        // check if rope has been initialised
        if (!initialised) {
            Debug.Log("Environment not initialised yet!");
            return response;
        }

        // while (predicting)
        //     await Task.Yield();

        // update state estimation
        statesEst.Clear();
        for (int i=0; i<numStates; i++) {
            RosPose pose = request.states_est.poses[i];
            statesEst.Add(cam2rob.TransformPoint(pose.position.From<FLU>()));
            // update markers
            stateMarkersList[i].transform.position = statesEst[i];
            ropePIDIntegral[i] = Vector3.zero;
            ropePIDLastErrors[i] = Vector3.zero;
        }

        // // get a hold of the constraint type we want, in this case, pin constraints:
        // var pinConstraints = rope.GetConstraintsByType(Oni.ConstraintType.Pin) as ObiConstraints<ObiPinConstraintsBatch>;

        // // remove all batches from it, so we start clean:
        // pinConstraints.Clear();

        // // create a new pin constraints batch
        // var batch = new ObiPinConstraintsBatch();

        // // Add a couple constraints to it, pinning the first and last particles in the rope:
        // for (int i=0; i<statesEst.Count; i++) {
        //     // add the constraint to the batch:
        //     batch.AddConstraint(rope.solverIndices[particleIds[i][0]], stateMarkersList[i].GetComponent<ObiCollider>(), Vector3.zero, Quaternion.identity, 0, 0, float.PositiveInfinity);
        // }

        // // set the amount of active constraints in the batch to 2 (the ones we just added).
        // batch.activeConstraintCount = statesEst.Count;

        // // append the batch to the pin constraints:
        // pinConstraints.AddBatch(batch);

        // // this will cause the solver to rebuild pin constraints at the beginning of the next frame:
        // rope.SetConstraintsDirty(Oni.ConstraintType.Pin);



        // update the rope
        adjusting = true;
        adjustErrorPrev = 0;
        while (adjusting)
            await Task.Yield();

        // generate the response
        List<RosPose> poseList = new List<RosPose>();
        for(int i=0; i<numStates; i++) {
            // update the rope states
            states[i] = cam2rob.InverseTransformPoint(rope.solver.positions[particleIds[i][0]]);
            poseList.Add(new RosPose(states[i].To<FLU>(), emptyRosQuat));
        }
        response.states_sim.poses = poseList.ToArray();
            
        // stop the timer
        timer.Stop();
        var elapsedMs = timer.ElapsedMilliseconds;
        Debug.Log("Adjustment time: "+elapsedMs+"ms");
        fpsDisplay.debugMessage = "Adjustment time: "+elapsedMs+"ms";
        
        // Debug.Log("Environment adjusted!");
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

    private SimPredResponse Predict(SimPredRequest request)
    {
        // Debug.Log("Predicting the environment...");
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
            // gripperTargets[i] = cam2rob.TransformPoint(pose.position.From<FLU>());
            // gripperTargetOrients[i] = cam2rob.rotation*pose.orientation.From<FLU>();
            // update gripper speed to guarantee reaching the target in prediction time
            // gripperTranslationSpeed = Vector3.Distance(grippersList[i].transform.position, gripperTargets[i])/(predictTime-1);
            // gripperRotationSpeed = Quaternion.Angle(grippersList[i].transform.rotation, gripperTargetOrients[i])/(predictTime-1);
            gripperMarkers[i].transform.position = gripperTargets[i];
            // TODO grasped particles only change when gripper states change
            // graspedParticles[i] = FindGraspedParticleGroup(gripperTargets[i], request.gripper_states.data[i]);
            gripperPIDIntegral[i] = Vector3.zero;
            gripperPIDLastErrors[i] = gripperMarkers[i].transform.position - grippersList[i].transform.position;
        }

        // start prediction
        predicting = true;
        predErrorPrev = 0;
        // while (predicting)
        //     await Task.Yield();

        // // generate the response
        // List<RosPose> poseList = new List<RosPose>();
        // for(int i=0; i<numStates; i++) {
        //     // update the rope states
        //     states[i] = cam2rob.InverseTransformPoint(rope.solver.positions[particleIds[i][0]]);
        //     poseList.Add(new RosPose(states[i].To<FLU>(), emptyRosQuat));
        // }
        // response.states_pred.poses = poseList.ToArray();

        // Debug.Log("Environment predicted!");
        return response;
    }

    private void FixedUpdate()
    {
        // spin demo camera
        if (Input.GetKey("d")) {
            // find the eyelet centre
            Vector3 eyeletCentre = Vector3.zero;
            for (int i=0; i<eyelets.Count; i++) {
                eyeletCentre += eyelets[i].position;
            }
            eyeletCentre /= eyelets.Count;
            // reset camera pose to be 1m to the eyelet centre and look at the eyelet centre
            camera.transform.position = new Vector3(0, 0, -1)+eyeletCentre;
            camera.transform.LookAt(eyeletCentre);
            // spin the camera around the eyelet centre for 360 degrees
            rotateCounter = 0;
        }
        if (rotateCounter<360) {
            // find the eyelet centre
            Vector3 eyeletCentre = Vector3.zero;
            for (int i=0; i<eyelets.Count; i++) {
                eyeletCentre += eyelets[i].position;
            }
            eyeletCentre /= eyelets.Count;
            // rotate the camera around the eyelet centre
            rotateCounter += 20 * Time.deltaTime;
            camera.transform.RotateAround(eyeletCentre, Vector3.up, 20 * Time.deltaTime);
        }

        if (!initialised) return;

        
            // Vector3 testAction = new Vector3(Input.GetAxis("HorizontalLeftGripper"), 
            //     Input.GetAxis("JumpLeftGripper"), 
            //     Input.GetAxis("VerticalLeftGripper"))*0.1f;
            // if (testAction!=Vector3.zero) {
            //     gripperTargets[0] += testAction;
            //     gripperMarkers[0].transform.position = gripperTargets[0];
            //     gripperPIDIntegral[0] = Vector3.zero;
            //     gripperPIDLastErrors[0] = gripperMarkers[0].transform.position - grippersList[0].transform.position;
            //     predicting = true;
            // }

        if (adjusting) {
            float distance = 0;
            for (int i=0; i<numStates; i++) {
                distance += Vector3.Distance(statesEst[i], rope.solver.positions[particleIds[i][0]]);
            }
            if (Mathf.Abs(adjustErrorPrev-distance)<adjustDistThresh || adjustCounter>adjustTime) {
                if (Mathf.Abs(adjustErrorPrev-distance)>=adjustDistThresh) Debug.Log("Adjustment time out!");
                else Debug.Log("Adjustment finished in "+adjustCounter+" steps. Final distance: "+distance/numStates+"m.");
                adjusting = false;
                adjustCounter = 0;
                adjustErrorPrev = 0;
                // set all particle speeds to zero
                for (int i=0; i<rope.solver.velocities.Count(); i++) {
					rope.solver.velocities[i] = Vector3.zero;
                }
                return;
            }
            adjustCounter++;
            adjustErrorPrev = distance;
        }

            // add gravity to corresponding particles
            var particle_velocities = rope.solver.velocities.AsNativeArray<float4>();
            var particle_positions = rope.solver.positions.AsNativeArray<float4>();
            for (int i=0; i<numStates; i++) {
                float3 centre = statesEst[i];
                // // set particle positions to the estimated states
                // particle_positions[particleIds[i+1][0]] = new float4(centre, 1);

                // generate velocity for the control particles
                var vel = particle_velocities[particleIds[i][0]];
                var pos = particle_positions[particleIds[i][0]];
                // vel.xyz += math.normalizesafe(centre - particle_positions[particleIds[i+1][0]].xyz) * math.distance(centre, particle_positions[particleIds[i+1][0]].xyz) * adjustSpeed * Time.deltaTime;
                // vel.xyz += math.normalizesafe(centre - particle_positions[particleIds[i+1][0]].xyz) * adjustSpeed;

                //PID
                Vector3 error = centre - particle_positions[particleIds[i][0]].xyz;
                ropePIDIntegral[i] += error * Time.deltaTime;
                Vector3 deriv = (error - ropePIDLastErrors[i]) / Time.deltaTime;
                ropePIDLastErrors[i] = error;
                Vector3 input = ropePID.x*error + ropePID.y*ropePIDIntegral[i] + ropePID.z*deriv;
                input = Vector3.ClampMagnitude(input, adjustSpeed); // clamp input
                // pos.xyz += (float3)input;
                // particle_positions[particleIds[i][0]] = pos;
                // vel.xyz += (float3)input;
                // particle_velocities[particleIds[i][0]] = vel;
                adjustForcePrev[i] = input;
                rope.solver.externalForces[particleIds[i][0]] += (Vector4)input;

                // // 'gravity'
                // Vector4 force = new Vector4();
                // force = (Vector3)(centre - particle_positions[particleIds[i][0]].xyz)* adjustSpeed;
                // // execute control signals
                // rope.solver.externalForces[particleIds[i][0]] += force;

                // foreach (int id in particleIds[i]) {
                //     // var vel = particle_velocities[id];
                //     // vel.xyz += math.normalizesafe(centre - particle_positions[id].xyz) * math.distance(centre, particle_positions[id].xyz) * adjustSpeed * Time.deltaTime;
                //     // particle_velocities[id] = vel;
                // Vector4 force = new Vector4();
                // force = math.distance(centre, particle_positions[id].xyz* adjustSpeed);
                // // execute control signals
                // rope.solver.externalForces[id] += force;
                // }
            }

        // if (predicting) {
        //     float distance = 0;
        //     for (int i=0; i<grippersList.Count; i++) {
        //         distance += Vector3.Distance(grippersList[i].transform.position, gripperMarkers[i].transform.position);
        //     }
        // if (Mathf.Abs(predErrorPrev-distance)<predDistThresh || predictCounter>predictTime) {

        //     if (Mathf.Abs(predErrorPrev-distance)>=predDistThresh) Debug.Log("Prediction time out!");
        //     else Debug.Log("Prediction finished in "+predictCounter+" steps.");
        //     predicting = false;
        //     predictCounter = 0;
        //     grippersList[0].GetComponent<Rigidbody>().velocity = Vector3.zero;
        //     grippersList[0].GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        //     grippersList[1].GetComponent<Rigidbody>().velocity = Vector3.zero;
        //     grippersList[1].GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        //     return;
        // }

            // move grippers gradually to targets
            for (int i=0; i<grippersList.Count; i++) {
                // grippersList[i].transform.position = Vector3.MoveTowards(grippersList[i].transform.position, gripperTargets[i], gripperTranslationSpeed);
                // add volecity to the gripper
                // grippersList[i].GetComponent<Rigidbody>().velocity = (gripperMarkers[i].transform.position-grippersList[i].transform.position).normalized*gripperTranslationSpeed;
                // add force to the gripper
                // grippersList[i].GetComponent<Rigidbody>().AddForce((gripperMarkers[i].transform.position-grippersList[i].transform.position).normalized*gripperTranslationSpeed, ForceMode.VelocityChange);
                // PID control
                Vector3 error = gripperMarkers[i].transform.position - grippersList[i].transform.position;
                gripperPIDIntegral[i] += error * Time.deltaTime;
                Vector3 deriv = (error - gripperPIDLastErrors[i]) / Time.deltaTime;
                gripperPIDLastErrors[i] = error;
                Vector3 input = gripperPID.x*error + gripperPID.y*gripperPIDIntegral[i] + gripperPID.z*deriv;
                input = Vector3.ClampMagnitude(input, gripperTranslationSpeed); // clamp input
                // reset rigidbody velocity
                grippersList[i].GetComponent<Rigidbody>().velocity = Vector3.zero;
                grippersList[i].GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
                grippersList[i].GetComponent<Rigidbody>().AddForce(input, ForceMode.VelocityChange);
                // grippersList[i].transform.position += input;
                grippersList[i].transform.rotation = Quaternion.RotateTowards(grippersList[i].transform.rotation, gripperTargetOrients[i], gripperRotationSpeed);
            }
            predictCounter++;
            // predErrorPrev = distance;
        // }

        // if (eyelets.Count==0) return;
        // if (active_eyelet_l!=-1 && active_eyelet_l<eyelets.Count-1) {
        //     eyelets[active_eyelet_l].position = Vector3.MoveTowards(eyelets[active_eyelet_l].position, target_eyelet_position[active_eyelet_l], 2f);
        //     eyelets[active_eyelet_l].rotation = Quaternion.RotateTowards(eyelets[active_eyelet_l].rotation, target_eyelet_rotation[active_eyelet_l], 90f);
        // }
        // if (active_eyelet_r!=-1 && active_eyelet_r<eyelets.Count) {
        //     eyelets[active_eyelet_r].position = Vector3.MoveTowards(eyelets[active_eyelet_r].position, target_eyelet_position[active_eyelet_r], 2f);
        //     eyelets[active_eyelet_r].rotation = Quaternion.RotateTowards(eyelets[active_eyelet_r].rotation, target_eyelet_rotation[active_eyelet_r], 90f);
        // }
        for (int i=0; i<eyelets.Count; i++) {
            eyelets[i].position = Vector3.MoveTowards(eyelets[i].position, target_eyelet_position[i], 2f);
            eyelets[i].rotation = Quaternion.RotateTowards(eyelets[i].rotation, target_eyelet_rotation[i], 90f);
        }
    }

}
