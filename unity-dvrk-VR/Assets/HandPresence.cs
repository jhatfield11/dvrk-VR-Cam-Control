using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class HandPresence : MonoBehaviour
{
    public bool showController;
    public List<GameObject> controllerPrefabs;
    public InputDeviceCharacteristics controllerCharachteristics;
    public GameObject handModelPrefab;

    private InputDevice targetDevice;
    private GameObject spawnController;
    private GameObject spawnedHandModel;
    private Animator handAnimator;

    // Start is called before the first frame update
    void Start()
    {
        tryInitialize();
    }

    void tryInitialize()
    {
       List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(controllerCharachteristics, devices);

        if(devices.Count > 0)
        {
            targetDevice = devices[0];
            GameObject prefab = controllerPrefabs.Find(controller => controller.name == targetDevice.name);
            if (prefab)
            {
                spawnController = Instantiate(prefab, transform);
            }
            else
            {
                Debug.Log("Did not find corresponding controller model ");
                spawnController = Instantiate(controllerPrefabs[0], transform);

            }

            spawnedHandModel = Instantiate(handModelPrefab, transform);
            handAnimator = spawnedHandModel.GetComponent<Animator>();
        } 
    }

    void updateHandAnimation()
    {
        if(targetDevice.TryGetFeatureValue(CommonUsages.trigger,out float triggerValue))
        {
            handAnimator.SetFloat("Trigger", triggerValue);
        }
        else
        {
            handAnimator.SetFloat("Trigger", 0);
        }

        if (targetDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue))
        {
            handAnimator.SetFloat("Grip", gripValue);
        }
        else
        {
            handAnimator.SetFloat("Grip", 0);
        }

    }

    // Update is called once per frame
    void Update()
    {
        if (!targetDevice.isValid)
        {
            tryInitialize();
        }
        else
        { 
            if(showController)
            {
                spawnedHandModel.SetActive(false);
                spawnController.SetActive(true);
            }
            else
            {
                spawnedHandModel.SetActive(true);
                spawnController.SetActive(false);
                updateHandAnimation();
            }
        }
        
    }
}
