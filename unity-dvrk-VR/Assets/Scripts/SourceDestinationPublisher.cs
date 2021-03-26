using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using System.Collections;
using System.Collections.Generic;
using System;

public class SourceDestinationPublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Variables required for ROS communication
    public string[] joint_topics;
    public string[] frame_topics;
    public string[] controller_topics;

    public GameObject[] Arms;
    public GameObject[] Base_Frames = new GameObject[3];
    public GameObject[] Controllers;

    public List<string> joint_Names;
    private List<double> current_Angles;
    private List<double> current_Vel;
    private List<double> current_Effort;

    public int i_ECM;
    public int i_PSM2;
    public int i_PSM1;

    public GameObject _camera;
    

    private RosMessageTypes.Geometry.Pose[] frame_message;
    private RosMessageTypes.Geometry.Pose[] position_message;
    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        RosMessageTypes.Geometry.Pose[] frame_message = new RosMessageTypes.Geometry.Pose[Arms.Length];
        RosMessageTypes.Geometry.Pose[] position_message = new RosMessageTypes.Geometry.Pose[Controllers.Length];
    // Get ROS connection static instance
        ros = ROSConnection.instance;
        int i = 0;
        
        joint_topics = new string[Arms.Length];
        frame_topics = new string[Arms.Length];
        controller_topics = new string[Controllers.Length];
        foreach (GameObject _controller in Controllers)
        {
            controller_topics[i] = "/unity/" +_controller.name + "/pose/";
            position_message[i] = new RosMessageTypes.Geometry.Pose();
            i++;
        }
        i = 0;
        foreach (GameObject arm in Arms)
        {
            frame_message[i] = new RosMessageTypes.Geometry.Pose();
            joint_topics[i] = "/unity/" + arm.name + "/joint_state/";
            frame_topics[i] = "/unity/" + arm.name + "/base_frame/";
            
            if (arm.name == "ECM")
            {
                i_ECM = i;
                Base_Frames[i] = GameObject.Find("ecm_setup_base_link");
            }
            else if (arm.name == "PSM1")
            {
                i_PSM1 = i;
                Base_Frames[i] = GameObject.Find(arm.name + "_psm_base_link");
            }
            else if (arm.name == "PSM2")
            {
                i_PSM2 = i;
                Base_Frames[i] = GameObject.Find(arm.name + "_psm_base_link");
            }
            i++;
        }

        for (i = 0; i < Base_Frames.Length; i++)
        {
            frame_message[i].position = Base_Frames[i].transform.position.To<FLU>();
            frame_message[i].orientation = Base_Frames[i].transform.rotation.To<FLU>();
        }
        

        i = 0;
     

        for (i = 0; i < frame_message.Length; i++)
        {
            ros.Send(frame_topics[i], frame_message[i]);
        }
    }

    void FixedUpdate()
    {
        Publish();
    }

    public void Publish()
    {
        RosMessageTypes.Geometry.Pose[] frame_message = new RosMessageTypes.Geometry.Pose[Arms.Length];
        RosMessageTypes.Geometry.Pose[] position_message = new RosMessageTypes.Geometry.Pose[Controllers.Length];
        int i = 0;
        foreach (GameObject arm in Arms)
        {
            frame_message[i] = new RosMessageTypes.Geometry.Pose();
            i++;
        }
        i = 0;
        

        for (i = 0; i < 3; i++)
        {
            frame_message[i].position = Base_Frames[i].transform.position.To<FLU>();
            frame_message[i].orientation = Base_Frames[i].transform.rotation.To<FLU>();
        }
        
        i = 0;
        foreach (GameObject _controller in Controllers)
        {
            position_message[i] = new RosMessageTypes.Geometry.Pose();
            position_message[i].position = Base_Frames[i].transform.InverseTransformPoint(_controller.transform.position).To<FLU>();
            position_message[i].orientation = Relative(Base_Frames[i].transform.rotation, _controller.transform.rotation).To<FLU>();
            i++;
        }
        
        for (i = 0; i < controller_topics.Length; i++)
        {
            ros.Send(controller_topics[i], position_message[i]);
        }

        for (i = 0; i < frame_message.Length; i++)
        {
            ros.Send(frame_topics[i], frame_message[i]);
        }
    }

    Quaternion Relative(Quaternion a, Quaternion b)
    {
        return Quaternion.Inverse(a) * b;
    }
}
