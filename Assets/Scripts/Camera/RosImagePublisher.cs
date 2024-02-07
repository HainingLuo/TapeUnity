using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosTimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;
using RosHeaderMsg = RosMessageTypes.Std.HeaderMsg;
using RosCameraInfoMsg = RosMessageTypes.Sensor.CameraInfoMsg;
using RosImageMsg = RosMessageTypes.Sensor.ImageMsg;
using RosCompressedImageMsg = RosMessageTypes.Sensor.CompressedImageMsg;
using RosPoseMsg = RosMessageTypes.Geometry.PoseStampedMsg;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.Robotics.ROSTCPConnector;
using RosCameraInfoGenerator = Unity.Robotics.ROSTCPConnector.MessageGeneration.CameraInfoGenerator;
using RosMessageExtensions = Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageExtensions;
/// <summary>
///
/// </summary>
public class RosImagePublisher : MonoBehaviour
{
    ROSConnection rosConnector;
    public string cameraName = "unity_camera";
    private string ImagetopicName;
    private string cameraInfoTopicName;
    private string cameraPoseTopicName;
    // The game object
    public Camera imageCamera;
    private string frameId;
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    [Range(0, 100)]
    public int imageQuality = 75;
    private Texture2D texture2D;
    private Rect rect;
    // Publish the cube's position and rotation every N seconds
    private float publishMessageFrequency = 0.1f;
    public float frameRate = 10f;
    private int img_data_size;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    public float seed=1;
    private uint seqNum = 0;
    private Transform robot_local;
    private Quaternion to_aglet_frame = Quaternion.Euler(0, 90f, 90f);
    // private RosCompressedImageMsg message;
    public RosImageMsg message;
    public RosCameraInfoMsg cameraInfoMessage;
    public RosPoseMsg poseMessage;
    private static Timer timer = new Timer();

    async void Start()
    {
        // imageCamera.enabled = true;
        // ImagetopicName = cameraName+"/rgb/image_raw/compressed";
        ImagetopicName = cameraName+"/color/image_raw";
        cameraInfoTopicName = cameraName+"/color/camera_info";
        frameId = cameraName+"_color_optical_frame";
        cameraPoseTopicName = cameraName+"/pose";
        publishMessageFrequency = 1/frameRate;
        timeElapsed = publishMessageFrequency*seed/100f;

        // start the ROS connection
        rosConnector = ROSConnection.instance;
        // rosConnector.RegisterPublisher<RosCompressedImageMsg>(ImagetopicName);
        rosConnector.RegisterPublisher<RosImageMsg>(ImagetopicName);
        rosConnector.RegisterPublisher<RosCameraInfoMsg>(cameraInfoTopicName);
        rosConnector.RegisterPublisher<RosPoseMsg>(cameraPoseTopicName);

        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        // message = new RosCompressedImageMsg();
        // message.format = "jpeg";
        int image_step = 3;
        message = new RosImageMsg();
        message.width = (uint) resolutionWidth;
        message.height = (uint) resolutionHeight;
        img_data_size = image_step * resolutionWidth; 
        message.step = (uint) img_data_size;
        message.encoding = "rgb8";

        // Initialize game Object
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        imageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);

        // Camera.onPostRender += UpdateImage;
        cameraInfoMessage = RosCameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, headerMsg, 0.0f, 0.01f);
        // cameraInfoMessage.k = GetIntrinsic(imageCamera);

        robot_local = GameObject.Find("Base Link").transform;
        poseMessage = new RosPoseMsg();
        poseMessage.header.frame_id = "base_link";
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (texture2D != null && timeElapsed > publishMessageFrequency) {
            // execute as coroutine to wait for the EndOfFrame before starting capture
            StartCoroutine(
                UpdateMessage());
            timeElapsed = 0;
        }
    }
    
    double[] GetIntrinsic(Camera cam)
    {
        Debug.Log(cam.focalLength);

        Debug.Log(cam.sensorSize.x);
        Debug.Log(cam.sensorSize.y);

        Debug.Log(cam.pixelWidth);
        Debug.Log(cam.pixelHeight);
        float pixel_aspect_ratio = (float)cam.pixelWidth / (float)cam.pixelHeight;

        float alpha_u = cam.focalLength * ((float)cam.pixelWidth / cam.sensorSize.x);
        float alpha_v = cam.focalLength * pixel_aspect_ratio * ((float)cam.pixelHeight / cam.sensorSize.y);

        float u_0 = (float)cam.pixelWidth / 2;
        float v_0 = (float)cam.pixelHeight / 2;

        //IntrinsicMatrix in row major
        double[] camIntriMat = new double[]
            {
                alpha_u, 0f, u_0,
                0f, alpha_v, v_0,
                0f, 0f, 1f
            };
        return camIntriMat;
    }

    public void FetchImage()
    {
        // yield return new WaitForEndOfFrame();
        var renderRT =
            RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default, 1);

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        imageCamera.targetTexture = renderRT;
        
        imageCamera.Render();
        texture2D.ReadPixels(rect, 0, 0);
        // texture2D.Apply();

        // imageCamera.targetTexture.Release();
        if ( imageCamera.targetTexture != null ) {
            imageCamera.targetTexture = null;
            RenderTexture.ReleaseTemporary(renderRT);
            RenderTexture.active = null;
        }

        // Update image message
        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        // RosCompressedImageMsg message = new RosCompressedImageMsg(headerMsg, "jpeg", texture2D.EncodeToJPG());
        message.header = headerMsg;
        // message.data = ImageConversion.EncodeToJPG(texture2D,imageQuality);
        message.data = texture2D.GetRawTextureData();

        // Update camera info message
        cameraInfoMessage.header = headerMsg;

        // Update pose message
        poseMessage.pose.position = robot_local.InverseTransformPoint(imageCamera.transform.position).To<FLU>();
        poseMessage.pose.orientation = (Quaternion.Inverse(robot_local.transform.rotation) * imageCamera.transform.rotation*to_aglet_frame).To<FLU>();

        seqNum += 1;
    }
        

    public IEnumerator UpdateMessage()
    {
        // yield return new WaitForEndOfFrame();
        var renderRT =
            RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default, 1);

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        imageCamera.targetTexture = renderRT;
        
        imageCamera.Render();
        texture2D.ReadPixels(rect, 0, 0);
        // texture2D.Apply();
        yield return null;

        // imageCamera.targetTexture.Release();
        if ( imageCamera.targetTexture != null ) {
            imageCamera.targetTexture = null;
            RenderTexture.ReleaseTemporary(renderRT);
            RenderTexture.active = null;
        }

        // Update image message
        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        // RosCompressedImageMsg message = new RosCompressedImageMsg(headerMsg, "jpeg", texture2D.EncodeToJPG());
        message.header = headerMsg;
        // message.data = ImageConversion.EncodeToJPG(texture2D,imageQuality);
        message.data = texture2D.GetRawTextureData();

        // Update camera info message
        cameraInfoMessage.header = headerMsg;

        // Update pose message
        poseMessage.pose.position = robot_local.InverseTransformPoint(imageCamera.transform.position).To<FLU>();
        poseMessage.pose.orientation = (Quaternion.Inverse(robot_local.transform.rotation) * imageCamera.transform.rotation*to_aglet_frame).To<FLU>();

        seqNum += 1;

        // send mesages out
        rosConnector.Send(ImagetopicName, message);
        rosConnector.Send(cameraInfoTopicName, cameraInfoMessage);
        rosConnector.Send(cameraPoseTopicName, poseMessage);
    }
}