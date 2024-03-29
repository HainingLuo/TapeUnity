//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Tape
{
    [Serializable]
    public class SimPredBatchResponse : Message
    {
        public const string k_RosMessageName = "tape/SimPredBatch";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseArrayMsg states_pred;

        public SimPredBatchResponse()
        {
            this.states_pred = new Geometry.PoseArrayMsg();
        }

        public SimPredBatchResponse(Geometry.PoseArrayMsg states_pred)
        {
            this.states_pred = states_pred;
        }

        public static SimPredBatchResponse Deserialize(MessageDeserializer deserializer) => new SimPredBatchResponse(deserializer);

        private SimPredBatchResponse(MessageDeserializer deserializer)
        {
            this.states_pred = Geometry.PoseArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.states_pred);
        }

        public override string ToString()
        {
            return "SimPredBatchResponse: " +
            "\nstates_pred: " + states_pred.ToString();
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
