//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoMoveit
{
    public class Current_joints : Message
    {
        public const string RosMessageName = "niryo_moveit/Current_joints";

        public Sensor.JointState joint_state;

        public Current_joints()
        {
            this.joint_state = new Sensor.JointState();
        }

        public Current_joints(Sensor.JointState joint_state)
        {
            this.joint_state = joint_state;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(joint_state.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.joint_state.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "Current_joints: " +
            "\njoint_state: " + joint_state.ToString();
        }
    }
}
