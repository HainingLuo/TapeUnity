//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Tsl
{
    [Serializable]
    public class SimAdjustResponse : Message
    {
        public const string k_RosMessageName = "tsl/SimAdjust";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseArrayMsg states_sim;

        public SimAdjustResponse()
        {
            this.states_sim = new Geometry.PoseArrayMsg();
        }

        public SimAdjustResponse(Geometry.PoseArrayMsg states_sim)
        {
            this.states_sim = states_sim;
        }

        public static SimAdjustResponse Deserialize(MessageDeserializer deserializer) => new SimAdjustResponse(deserializer);

        private SimAdjustResponse(MessageDeserializer deserializer)
        {
            this.states_sim = Geometry.PoseArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.states_sim);
        }

        public override string ToString()
        {
            return "SimAdjustResponse: " +
            "\nstates_sim: " + states_sim.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
