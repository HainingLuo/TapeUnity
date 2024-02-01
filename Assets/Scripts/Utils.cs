using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosPose = RosMessageTypes.Geometry.PoseMsg;
using RosPoint = RosMessageTypes.Geometry.PointMsg;
using RosPoseArray = RosMessageTypes.Geometry.PoseArrayMsg;
using RosQuaternion = RosMessageTypes.Geometry.QuaternionMsg;

static class Utils
{
    public static RosPoint Unity2Msg(Vector3 vector3) {
        return new RosPoint(vector3.z, -vector3.x, vector3.y);
    }

    public static RosQuaternion Unity2Msg(Quaternion quaternion) {
        return new RosQuaternion((float)-quaternion.z, (float)quaternion.x, (float)-quaternion.y, (float)quaternion.w);
    }

    public static Vector3 Msg2Unity(RosPoint point) {
        return new Vector3((float)-point.y, (float)point.z, (float)point.x);
    }

    public static Vector3 ROS2UnityPoint(Vector3 point) {
        return new Vector3((float)-point.y, (float)point.z, (float)point.x);
    }

}