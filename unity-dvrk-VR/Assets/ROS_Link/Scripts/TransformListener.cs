using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace ROSLink
{
    public class TransformListener : MonoBehaviour
    {
        private GameObject Bot;
        private static int prismaticJointVariable = 3;
        private static int revoluteJointVariable = 2;
        public List<float[]> dh;
        public List<ArticulationBody> jointChain;
        public List<string> jointNames;
        public List<float> currentAngles;


        [System.Serializable]
        public class DataRecord
        {
            public string Name;
            public float CurrentAng;
        }

        public List<DataRecord> table = new List<DataRecord>();
        // Start is called before the first frame update
        void Start()
        {
            Bot = this.gameObject;
            //Find all joints within robot and corresponding names 
            foreach (ArticulationBody joint in Bot.GetComponentsInChildren<ArticulationBody>())
            {
                DataRecord row = new DataRecord();
                if (joint.jointType != ArticulationJointType.FixedJoint)
                {
                    jointChain.Add(joint);
                    jointNames.Add(joint.name);
                    row.Name = joint.name;
                    if (joint.jointType == ArticulationJointType.RevoluteJoint)
                    {
                        currentAngles.Add(joint.jointPosition[0]);
                        row.CurrentAng = joint.jointPosition[0];
                    }
                    else if (joint.jointType == ArticulationJointType.PrismaticJoint)
                    {
                        currentAngles.Add(joint.jointPosition[3]);
                        row.CurrentAng = joint.jointPosition[3];
                    }
                    else
                    {
                        Debug.LogError("Other joint types not supported");
                    }
                    table.Add(row);
                }
            }

        }


        // Update is called once per frame
        private void FixedUpdate()
        {
           SendAngles();
        }
        
        void SendAngles()
        {
            for (int i = 0; i < jointChain.Count; i++)
            {
                DataRecord row = new DataRecord();
                row.Name = jointChain[i].name;
                if (jointChain[i].jointType == ArticulationJointType.RevoluteJoint)
                {
                    row.CurrentAng = (float)Math.Round(jointChain[i].jointPosition[0], 2);
                    currentAngles[i]=((float)Math.Round(jointChain[i].jointPosition[0], 2));
                }
                else if (jointChain[i].jointType == ArticulationJointType.PrismaticJoint)
                {
                    row.CurrentAng = (float)Math.Round(jointChain[i].jointPosition[3], 2);
                    currentAngles[i]=((float)Math.Round(jointChain[i].jointPosition[3], 2));
                }
                else
                {
                    Debug.LogError("Other joint types not supported");
                }
                table[i] = row;
            }
           // ClientSend.ControllerAngles(jointNames, currentAngles, Bot.name);
        }
    }
  
}
