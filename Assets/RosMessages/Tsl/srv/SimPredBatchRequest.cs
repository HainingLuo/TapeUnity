//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Tsl
{
    [Serializable]
    public class SimPredBatchRequest : Message
    {
        public const string k_RosMessageName = "tsl/SimPredBatch";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseArrayMsg gripper_poses_prev;
        public Geometry.PoseArrayMsg gripper_poses_curr;
        public Std.Float32MultiArrayMsg gripper_states;
        public Geometry.PoseArrayMsg states_prev;
        public Std.Float32MultiArrayMsg parameters;

        public SimPredBatchRequest()
        {
            this.gripper_poses_prev = new Geometry.PoseArrayMsg();
            this.gripper_poses_curr = new Geometry.PoseArrayMsg();
            this.gripper_states = new Std.Float32MultiArrayMsg();
            this.states_prev = new Geometry.PoseArrayMsg();
            this.parameters = new Std.Float32MultiArrayMsg();
        }

        public SimPredBatchRequest(Geometry.PoseArrayMsg gripper_poses_prev, Geometry.PoseArrayMsg gripper_poses_curr, Std.Float32MultiArrayMsg gripper_states, Geometry.PoseArrayMsg states_prev, Std.Float32MultiArrayMsg parameters)
        {
            this.gripper_poses_prev = gripper_poses_prev;
            this.gripper_poses_curr = gripper_poses_curr;
            this.gripper_states = gripper_states;
            this.states_prev = states_prev;
            this.parameters = parameters;
        }

        public static SimPredBatchRequest Deserialize(MessageDeserializer deserializer) => new SimPredBatchRequest(deserializer);

        private SimPredBatchRequest(MessageDeserializer deserializer)
        {
            this.gripper_poses_prev = Geometry.PoseArrayMsg.Deserialize(deserializer);
            this.gripper_poses_curr = Geometry.PoseArrayMsg.Deserialize(deserializer);
            this.gripper_states = Std.Float32MultiArrayMsg.Deserialize(deserializer);
            this.states_prev = Geometry.PoseArrayMsg.Deserialize(deserializer);
            this.parameters = Std.Float32MultiArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.gripper_poses_prev);
            serializer.Write(this.gripper_poses_curr);
            serializer.Write(this.gripper_states);
            serializer.Write(this.states_prev);
            serializer.Write(this.parameters);
        }

        public override string ToString()
        {
            return "SimPredBatchRequest: " +
            "\ngripper_poses_prev: " + gripper_poses_prev.ToString() +
            "\ngripper_poses_curr: " + gripper_poses_curr.ToString() +
            "\ngripper_states: " + gripper_states.ToString() +
            "\nstates_prev: " + states_prev.ToString() +
            "\nparameters: " + parameters.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
