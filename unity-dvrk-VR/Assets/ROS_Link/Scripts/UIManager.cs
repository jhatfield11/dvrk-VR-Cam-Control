using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace ROSLink
{
    public class UIManager : MonoBehaviour
    {
        public static UIManager instance;
        public GameObject startMenu;
        public InputField nameField;


        private void Awake()
        {
            if (instance == null)
            {
                instance = this;
            }
            else if (instance != this)
            {
                Debug.Log("Instance already exists, destroying instance");
                Destroy(this);

            }
        }

        public void ConnectToServer()
        {
            startMenu.SetActive(false);
            nameField.interactable = false;
            Client.instance.ConnectToServer();

        }
    }
}
