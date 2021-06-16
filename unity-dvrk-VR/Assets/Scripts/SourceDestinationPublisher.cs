using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.NiryoMoveit;
using RosMessageTypes.ROS;
using UnityEngine;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using System.Linq;

public class SourceDestinationPublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    public string Path;
    public string fileName;
    public bool DisplayPointCloud;

    public Color color;
    public float particleSize = 5;

    // Variables required for ROS communication
    private string[] joint_topics;
    public string[] frame_topics;
    public string[] controller_topics;

    public GameObject[] Arms;
    private GameObject[] Base_Frames = new GameObject[3];
    public GameObject[] Controllers;

    private List<string> joint_Names;
    private List<double> current_Angles;
    private List<double> current_Vel;
    private List<double> current_Effort;

    private int i_ECM;
    private int i_PSM2;
    private int i_PSM1;

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
        parseXYZ(ros, Path + fileName);
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
    
    void parseXYZ(ROSConnection rs,string filePath)
    {
        string[] lines = File.ReadAllLines(filePath);
        double[] X = new double[lines.Length];
        double[] Y = new double[lines.Length];
        double[] Z = new double[lines.Length];
        int i = 0;
        foreach (string line in lines)
        {
            var words = line.Split(' ');
            X[i] = (double.Parse(words[0]) / 1000);
            Y[i] = (double.Parse(words[1]) / 1000);
            Z[i] = (double.Parse(words[2]) / 1000);
            i++;
        }
        RosMessageTypes.ROS.XYZcloud Cloud = new RosMessageTypes.ROS.XYZcloud();
        Cloud.X = X;
        Cloud.Y = Y;
        Cloud.Z = Z;

        if (DisplayPointCloud == true)
        {
            ApplyToParticleSystem(X, Y, Z);
        }
        else
        {
            var ps = GetComponent<ParticleSystem>();
            ps.Stop();
        }

        rs.Send("/unity/XYZCloud", Cloud);
    }
    public void ApplyToParticleSystem(double[] x, double[] y, double[] z)
    {
        var ps = GetComponent<ParticleSystem>();
        if (ps == null)
        {
            Debug.Log("Particle system not created");
            return;
        }


        var particles = new ParticleSystem.Particle[x.Length];

        for (int i = 0; i < particles.Length; ++i)
        {
            particles[i].position = new UnityEngine.Vector3((float)x[i], (float)y[i], (float)z[i]);
            particles[i].startSize = particleSize;
            particles[i].startColor = color;
        }
        ps.SetParticles(particles);
        ps.Pause();
        Debug.Log("Particle system created");
    }
}
