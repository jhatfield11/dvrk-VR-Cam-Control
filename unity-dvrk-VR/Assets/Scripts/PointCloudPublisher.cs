using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.ROS;
using UnityEngine;
using UnityEngine.Rendering;
using ROSGeometry;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using System.Linq;



public class PointCloudPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string Path;
    public string fileName;
    public bool DisplayPointCloud;

    public Color color;
    public float particleSize = 5;
    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
        var filePath = Path + fileName;
        string[] lines = File.ReadAllLines(filePath);
        double[] X = new double[lines.Length];
        double[] Y = new double[lines.Length];
        double[] Z = new double[lines.Length];
        int i = 0;
        foreach (string line in lines)
        {
            var words = line.Split(' ');
            X[i] = (double.Parse(words[0])/1000);
            Y[i] = (double.Parse(words[1])/1000);
            Z[i] = (double.Parse(words[2])/1000);
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
        
        ros.Send("/unity/XYZCloud", Cloud);
        
    }

    // Update is called once per frame
    void Update()
    {
        
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
