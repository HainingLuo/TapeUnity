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
    public float envSpacing = 3.0f;
    private List<GameObject> envs=new List<GameObject>();

    // rope variables
    private List<ObiRope> ropes=new List<ObiRope>();
    private List<string> ropeNames;
    private List<Vector3> ropeCentres=new List<Vector3>();
    private ObiSolver obiSolver;
    public Material ropeMaterial;
    public float ropeLength=13.8f;
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

    // gripper variables
    public List<GameObject> leftGrippers;
    public List<GameObject> rightGrippers;


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

        // Create a set of ropes
        for (int i=0; i<numEnvs; i++) {
        }
        
        // Create a SimPredBatch server

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
