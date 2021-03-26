using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;

using Joints_PSM1 = RosMessageTypes.Sensor.JointState;
using Joints_PSM2 = RosMessageTypes.Sensor.JointState;



public class Joint_Subsciber : MonoBehaviour
{
    public GameObject PSM1;
    public GameObject PSM2;
    public GameObject ECM; 



    // Start is called before the first frame update
    void Start()
    {

        ROSConnection.instance.Subscribe<RosMessageTypes.Sensor.JointState>("/dvrk/PSM1/joint_states/", Write_joints_PSM1);
        ROSConnection.instance.Subscribe<RosMessageTypes.Sensor.JointState>("/dvrk/PSM2/joint_states/", Write_joints_PSM2);
        ROSConnection.instance.Subscribe<RosMessageTypes.Sensor.JointState>("/dvrk/ECM/joint_states/", Write_joints_ECM);
    }

    void Write_joints_PSM1(RosMessageTypes.Sensor.JointState joints_message)
    {
        ArticulationBody[] joint_chain = PSM1.GetComponentsInChildren<ArticulationBody>();


        for (int i = 0; i < (joints_message.position.Length - 3); i++)
        {

            GameObject joint_object;
            ArticulationBody joint;
            if (joints_message.name[i] == "outer_insertion")
            {
                joint_object = GameObject.Find("PSM1_" + joints_message.name[i]);
                joint = joint_object.GetComponent<ArticulationBody>();
                var joint1XDrive = joint.xDrive;
                joint1XDrive.target = Convert.ToSingle(joints_message.position[i]);
                joint.xDrive = joint1XDrive;
            }
            else
            {
                joint_object = GameObject.Find("PSM1_" + joints_message.name[i]);
                joint = joint_object.GetComponent<ArticulationBody>();
                var joint1XDrive = joint.xDrive;
                joint1XDrive.target = Convert.ToSingle((180 / Math.PI) * joints_message.position[i]);
                joint.xDrive = joint1XDrive;
            }
        }
    }

    void Write_joints_PSM2(RosMessageTypes.Sensor.JointState joints_message)
    {
        ArticulationBody[] joint_chain = PSM2.GetComponentsInChildren<ArticulationBody>();


        for (int i = 0; i < (joints_message.position.Length - 3); i++)
        {

            GameObject joint_object;
            ArticulationBody joint;
            if (joints_message.name[i] == "outer_insertion")
            {
                joint_object = GameObject.Find("PSM2_" + joints_message.name[i]);
                joint = joint_object.GetComponent<ArticulationBody>();
                var joint1XDrive = joint.xDrive;
                joint1XDrive.target = Convert.ToSingle(joints_message.position[i]);
                joint.xDrive = joint1XDrive;
            }
            else
            {
                joint_object = GameObject.Find("PSM2_" + joints_message.name[i]);
                joint = joint_object.GetComponent<ArticulationBody>();
                var joint1XDrive = joint.xDrive;
                joint1XDrive.target = Convert.ToSingle((180 / Math.PI) * joints_message.position[i]);
                joint.xDrive = joint1XDrive;
            }
        }
    }

    void Write_joints_ECM(RosMessageTypes.Sensor.JointState joints_message)
    {
        ArticulationBody[] joint_chain = ECM.GetComponentsInChildren<ArticulationBody>();


        for (int i = 0; i < (joints_message.position.Length - 3); i++)
        {
            GameObject joint_object;
            ArticulationBody joint;
            if (joints_message.name[i] == "insertion")
            {
                joint_object = GameObject.Find("ecm_" + joints_message.name[i]);
                joint = joint_object.GetComponent<ArticulationBody>();
                var joint1XDrive = joint.xDrive;
                joint1XDrive.target = Convert.ToSingle(joints_message.position[i]);
                joint.xDrive = joint1XDrive;
            }
            else
            {
                joint_object = GameObject.Find("ecm_" + joints_message.name[i]);
                joint = joint_object.GetComponent<ArticulationBody>();
                var joint1XDrive = joint.xDrive;
                joint1XDrive.target = Convert.ToSingle((180 / Math.PI) * joints_message.position[i]);
                joint.xDrive = joint1XDrive;
            }
        }
    }
}
