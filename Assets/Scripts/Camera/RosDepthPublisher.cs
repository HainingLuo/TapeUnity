using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using RosTimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;
using RosHeaderMsg = RosMessageTypes.Std.HeaderMsg;
using RosCameraInfoMsg = RosMessageTypes.Sensor.CameraInfoMsg;
using RosImageMsg = RosMessageTypes.Sensor.ImageMsg;
using RosCompressedImageMsg = RosMessageTypes.Sensor.CompressedImageMsg;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosCameraInfoGenerator = Unity.Robotics.ROSTCPConnector.MessageGeneration.CameraInfoGenerator;
using RosMessageExtensions = Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageExtensions;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Rendering;
using Unity.Mathematics;
using UnityEngine.Experimental.Rendering;

/// <summary>
/// useful links:
/// Understanding depth texture: https://docs.unity3d.com/Manual/SL-CameraDepthTexture.html
/// Understanding Unity camera: https://docs.unity3d.com/Manual/PhysicalCameras.html
/// Understanding texture formats: https://docs.unity3d.com/ScriptReference/TextureFormat.html
/// Understanding raw texture data: https://docs.unity3d.com/ScriptReference/Texture2D.GetRawTextureData.html
/// Retrieving intrinsics: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.ros-tcp-connector/Runtime/Extensions/CameraInfoGenerator.cs
/// Retrieving intrinsics: https://github.com/Unity-Technologies/com.unity.perception/issues/548 (the results are not good)
/// Realsense intrinsics: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#intrinsic-camera-parameters
/// Realsense intrinsics: https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20
/// </summary>

public class RosDepthPublisher : MonoBehaviour
{
    ROSConnection rosConnector;
    public string cameraName = "unity_camera";
    private string ImagetopicName;
    private string cameraInfoTopicName;

    // The game object
    public Camera imageCamera;
    private string frameId;
    public int resolutionWidth = 1280;
    public int resolutionHeight = 720;
    // [Range(0, 100)]
    public int imageQuality = 75;
    // private Texture2D texture2D;
    // private Rect rect;
    // Publish the cube's position and rotation every N seconds
    private float publishMessageFrequency = 0.1f;
    public float frameRate = 10f;
    private int img_data_size;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    public float seed=1;

    private uint seqNum = 0;
    // private RosCompressedImageMsg message;
    public RosImageMsg message;
    public RosCameraInfoMsg cameraInfoMessage;

    bool supportsAntialiasing = true;
    bool needsRescale = false;
    int depth = 32;
    RenderTextureFormat format = RenderTextureFormat.RFloat;
    RenderTextureReadWrite readWrite = RenderTextureReadWrite.Default;
    int antiAliasing = 1;

    // pass configuration
    private CapturePass capturePass = new CapturePass() { name = "_depth" };
    private Texture2D texture2D;
    private Rect rect;
    struct CapturePass
    {
        // configuration
        public string name;
        public bool supportsAntialiasing;
        public bool needsRescale;
        public CapturePass(string name_) { name = name_; supportsAntialiasing = true; needsRescale = false; camera = null; }
        public Camera camera;
    };

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.allowHDR = false;
        cam.allowMSAA = false;
    }

    public enum ReplacementMode
    {
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4
    };

    async void Start()
    {
        // imageCamera.enabled = true;
        // ImagetopicName = cameraName+"/aligned_depth_to_color/image_raw/compressed";
        ImagetopicName = cameraName+"/aligned_depth_to_color/image_raw";
        cameraInfoTopicName = cameraName+"/aligned_depth_to_color/camera_info";
        frameId = cameraName+"_color_optical_frame";
        publishMessageFrequency = 1/frameRate;
        timeElapsed = publishMessageFrequency*seed/100f;

        antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

        // start the ROS connection
        rosConnector = ROSConnection.instance;
        // rosConnector.RegisterPublisher<CompressedImageMsg>(ImagetopicName);
        rosConnector.RegisterPublisher<RosImageMsg>(ImagetopicName);
        rosConnector.RegisterPublisher<RosCameraInfoMsg>(cameraInfoTopicName);

        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        // message = new RosCompressedImageMsg();
        // message.format = "jpeg";
        cameraInfoMessage = RosCameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, headerMsg, 0.0f, 0.01f);
        cameraInfoMessage.k = GetIntrinsic(imageCamera);

        int image_step = 4;
        message = new RosImageMsg();
        message.width = (uint) resolutionWidth;
        message.height = (uint) resolutionHeight;
        img_data_size = image_step * resolutionWidth; 
        message.step = (uint) img_data_size;
        message.encoding = "rgba8";

        // Initialize game Object
        // texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        // rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        // ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RFloat, false); // RFloat: reder only the red channel (32bits)
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        var uberReplacementShader = Shader.Find("Hidden/DepthCamera");
        imageCamera.RemoveAllCommandBuffers();
        //set up camera shader
        SetupCameraWithReplacementShader(imageCamera, uberReplacementShader, ReplacementMode.DepthCompressed, Color.white);

        capturePass.camera = imageCamera;

        // on scene change
        var renderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
        var mpb = new MaterialPropertyBlock();
        foreach (var r in renderers)
        {
            var id = r.gameObject.GetInstanceID();
            var layer = r.gameObject.layer;
            var tag = r.gameObject.tag;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
            r.SetPropertyBlock(mpb);
        }
    }
    
    double[] GetIntrinsic(Camera cam)
    {
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
    public void FetchImage()
    {
        // yield return new WaitForEndOfFrame();
        var renderRT =
            RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, depth, format, readWrite, antiAliasing);

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

        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        // RosCompressedImageMsg message = new RosCompressedImageMsg(headerMsg, "jpeg", texture2D.EncodeToJPG());
        message.header = headerMsg;
        // message.data = ImageConversion.EncodeToJPG(texture2D,imageQuality);
        message.data = texture2D.GetRawTextureData();

        // Camera Info message
        // CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, message.header, 0.0f, 0.01f);
        cameraInfoMessage.header = headerMsg;

        seqNum += 1;
    }

    public IEnumerator UpdateMessage()
    {
        // yield return new WaitForEndOfFrame();
        var renderRT =
            RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, depth, format, readWrite, antiAliasing);

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        imageCamera.targetTexture = renderRT;
        
        imageCamera.Render();
        texture2D.ReadPixels(rect, 0, 0);
        // Debug.Log(texture2D.GetPixel(640,360).r.ToString());
        // Debug.Log(texture2D.GetPixel(640,360).r.GetType().ToString());
        // texture2D.Apply();
        yield return null;

        // imageCamera.targetTexture.Release();
        if ( imageCamera.targetTexture != null ) {
            imageCamera.targetTexture = null;
            RenderTexture.ReleaseTemporary(renderRT);
            RenderTexture.active = null;
        }

        var timeMessage = new RosTimeMsg();
        var headerMsg = new RosHeaderMsg(seqNum, timeMessage, frameId);
        // RosCompressedImageMsg message = new RosCompressedImageMsg(headerMsg, "jpeg", texture2D.EncodeToJPG());
        message.header = headerMsg;
        // message.data = ImageConversion.EncodeToJPG(texture2D,imageQuality);
        message.data = texture2D.GetRawTextureData();
        // var texture_data = texture2D.GetRawTextureData<Color>();
        // message.data = new byte[img_data_size];
        // Debug.Log(message.data.Length);
        // Debug.Log(texture_data.Length);
        // int index = 0;
        // for (int y = 0; y < resolutionHeight; y++)
        // {
        //     for (int x = 0; x < resolutionWidth; x++)
        //     {
        //         message.data[index++*4] = (byte)texture_data[index].r;
        //         // index+=4;
        //     }
        // }


        // Camera Info message
        // CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(imageCamera, message.header, 0.0f, 0.01f);
        cameraInfoMessage.header = headerMsg;

        seqNum += 1;

        // Publish messages
        rosConnector.Publish(ImagetopicName, message);
        rosConnector.Publish(cameraInfoTopicName, cameraInfoMessage);
    }
}
