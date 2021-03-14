/*
    Author(s):  Long Qian
    Created on: 2019-03-29
    (C) Copyright 2015-2018 Johns Hopkins University (JHU), All Rights Reserved.

    --- begin cisst license - do not edit ---
    This software is provided "as is" under an open source license, with
    no warranty.  The complete license can be found in license.txt and
    http://www.cisst.org/cisst/license.txt.
    --- end cisst license ---
*/
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace DVRK
{

    public class URDFRobot : MonoBehaviour
    {

        public List<URDFJoint> independentJoints = new List<URDFJoint>();
        public URDFJoint jaw = null;

        public static List<URDFRobot> instances = new List<URDFRobot>();
        private int instanceID = -1;

        private UDPClient udpClient;

        public GameObject SliderPrefab;
        public GameObject ButtonPrefab;



        public virtual void HandleMessage(string message)
        {
            Debug.Log("DVRK::URDFRobot base class not implementing HandleMessage");
        }


        // Use this for initialization
        void Start()
        {







            // all the joints, to setup linkage
            foreach (URDFJoint joint in GetComponentsInChildren<URDFJoint>())
            {
                joint.SetupRobotJoint();
            }
            foreach (URDFJoint joint in independentJoints)
            {
                joint.SetJointValueDefault();
            }
            if (jaw != null)
            {
                jaw.SetJointValueDefault();
            }

            udpClient = GetComponent<UDPClient>();

            instances.Add(this);
            instanceID = instances.Count - 1;
            Debug.Log(name + ": Current URDFRobot instanceID: " + instanceID);


            SpawnUI();
        }

        // LateUpdate is called once per frame
        void LateUpdate()
        {
            string message = "";
            message = udpClient.GetLatestUDPPacket();

            if (message != "")
            {
                HandleMessage(message);
            }
            UIupdate();
        }


        void SpawnUI()
        {

            Text label;
            Slider slider;
            Button button;
            int width = 100;
            int setupHeight = 20;
            int height = setupHeight * 3 * independentJoints.Count;
            int currentHeight = height / 3;


            GameObject newCanvas = new GameObject("Canvas");
            Canvas c = newCanvas.AddComponent<Canvas>();
            c.renderMode = RenderMode.WorldSpace;
            newCanvas.AddComponent<CanvasScaler>();
            newCanvas.AddComponent<GraphicRaycaster>();
            c.worldCamera = GameObject.Find("VR Camera").GetComponent<Camera>();
            

            RectTransform rectTransform;
           

            rectTransform = c.GetComponent<RectTransform>();
            rectTransform.localScale = new Vector3(0.005f, 0.005f, 0.001f);
            rectTransform.localPosition = transform.position + new Vector3(0, 1.3f, 0.5f);
            rectTransform.sizeDelta = new Vector2(width, height);

            foreach (URDFJoint joint in independentJoints)
            {
                GameObject mylabel = new GameObject(joint.name + "-label");
                label = mylabel.AddComponent<Text>();
                label.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
                label.text = joint.name;
                label.fontSize = 10;

                label.transform.SetParent(newCanvas.transform, false);

                
                rectTransform = label.GetComponent<RectTransform>();
                rectTransform.localPosition = new Vector3(0, currentHeight, 0);
                rectTransform.sizeDelta = new Vector2(width, setupHeight);
                currentHeight -= setupHeight;

                float val = joint.defaultJointValue;
                if (joint.jointType == URDFJoint.JointType.Revolute || joint.jointType == URDFJoint.JointType.Prismatic)
                {

                  
                    GameObject mySlider = Instantiate(SliderPrefab, new Vector3(0, 0, 0), Quaternion.identity);
                    mySlider.transform.SetParent(newCanvas.transform, false);

                    mySlider.name = joint.name + "-slider";
                    slider = mySlider.GetComponent<Slider>();

                    slider.minValue = joint.jointLimit.x;
                    slider.maxValue = joint.jointLimit.y;
                    slider.value = joint.currentJointValue;

                    slider.transform.SetParent(newCanvas.transform, false);

                    rectTransform = slider.GetComponent<RectTransform>();
                    rectTransform.localPosition = new Vector3(0, currentHeight, 0);
                    rectTransform.sizeDelta = new Vector2(width, setupHeight);
                    currentHeight -= setupHeight;
                }
                else if (joint.jointType == URDFJoint.JointType.Continuous)
                {
                    
                    GameObject mySlider = Instantiate(SliderPrefab, new Vector3(0, 0, 0), Quaternion.identity);
                    mySlider.transform.SetParent(newCanvas.transform, false);

                    mySlider.name = joint.name + "-slider";
                    slider = mySlider.GetComponent<Slider>();
                    slider.minValue = -180f;
                    slider.maxValue = 180f;
                    slider.value = joint.currentJointValue;

                    slider.transform.SetParent(newCanvas.transform, false);
                    

                    rectTransform = slider.GetComponent<RectTransform>();
                    rectTransform.localPosition = new Vector3(0, currentHeight, 0);
                    rectTransform.sizeDelta = new Vector2(width, setupHeight);
                    currentHeight -= setupHeight;
                }
                GameObject myButton = Instantiate(ButtonPrefab, new Vector3(0, 0, 0), Quaternion.identity);
                myButton.transform.SetParent(newCanvas.transform, false);

                myButton.name = joint.name + "Set Default Button";
                button = myButton.GetComponent<Button>();

                label = button.GetComponentInChildren<Text>();
                label.text = "Set Default";

                rectTransform = button.GetComponent<RectTransform>();
                rectTransform.localPosition = new Vector3(0, currentHeight, 0);
                rectTransform.sizeDelta = new Vector2(width, setupHeight);
                currentHeight -= setupHeight;
            }
        }


        public void UIupdate()
        {
            foreach(URDFJoint joint in independentJoints){

                Slider slider;
                Button button;
                

                slider = GameObject.Find (joint.name + "-slider").GetComponent<Slider>();
                button = GameObject.Find (joint.name + "Set Default Button").GetComponent<Button>();

                joint.SetJointValue(slider.value);

                Debug.Log(joint.currentJointValue);





            }
        }





        /*
        #if UNITY_EDITOR
                void OnGUI() {
                    int width = 100;
                    int height = 20;
                    int currentHeight = height;
                    int setupHeight = 20;
                    foreach (URDFJoint joint in independentJoints) {
                        GUI.Label(new Rect(10 + instanceID * width, currentHeight, width, height), joint.name);
                        currentHeight += setupHeight;
                        float val = joint.defaultJointValue;
                        if (joint.jointType == URDFJoint.JointType.Revolute || joint.jointType == URDFJoint.JointType.Prismatic) {
                            val = GUI.HorizontalSlider(new Rect(10 + instanceID * width, currentHeight, width, height), joint.currentJointValue,
                                joint.jointLimit.x, joint.jointLimit.y);
                        }
                        else if (joint.jointType == URDFJoint.JointType.Continuous) {
                            val = GUI.HorizontalSlider(new Rect(10 + instanceID * width, currentHeight, width, height), joint.currentJointValue,
                                -180f, 180f);
                        }
                        joint.SetJointValue(val);
                        currentHeight += setupHeight;
                    }
                    if (GUI.Button(new Rect(10 + instanceID * width, currentHeight, width, height), "Recenter")) {
                        foreach (URDFJoint joint in independentJoints) {
                            joint.SetJointValueDefault();
                        }
                    }

                }
        #endif
        */
    }
}

