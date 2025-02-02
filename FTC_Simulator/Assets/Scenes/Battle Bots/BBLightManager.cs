using System.Collections;
using System.Collections.Generic;
using UnityEditor.Animations;
using UnityEngine;

public class BBLightManager : MonoBehaviour
{
    public Transform bottomLightsLeftParent;
    List<Light> bottomLightsLeft = new List<Light>();
    public float bottomLightLeftIntensity = 50f;

    public Transform bottomLightsRightParent;
    List<Light> bottomLightsRight = new List<Light>();
    public float bottomLightRightIntensity = 50f;

    public Transform topLightsParent;
    List<Light> topLights = new List<Light>();
    public float topLightIntensity = 125f;

    public Transform spotLightsParent;
    List<Light> spotLights = new List<Light>();
    public float spotLightIntensity = 50f;

    public Animator lightsAnimator;


    // Start is called before the first frame update
    void Start()
    {
        // Find all the lights
        foreach(Light currLight in bottomLightsLeftParent.GetComponentsInChildren<Light>())
        {
            bottomLightsLeft.Add(currLight);
        }

        foreach (Light currLight in bottomLightsRightParent.GetComponentsInChildren<Light>())
        {
            bottomLightsRight.Add(currLight);
        }

        foreach (Light currLight in topLightsParent.GetComponentsInChildren<Light>())
        {
            topLights.Add(currLight);
        }

        foreach (Light currLight in spotLightsParent.GetComponentsInChildren<Light>())
        {
            spotLights.Add(currLight);
        }

        lightsAnimator = GetComponent<Animator>();
    }

    // Update is called once per frame
    void Update()
    {
        UpdateLights();

    }

    public void UpdateLights()
    {
        // Update all Lights
        foreach (Light currLight in bottomLightsLeftParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = bottomLightLeftIntensity;
        }

        foreach (Light currLight in bottomLightsRightParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = bottomLightRightIntensity;
        }

        foreach (Light currLight in topLightsParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = topLightIntensity;
        }

        foreach (Light currLight in spotLightsParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = spotLightIntensity;
        }
    }

    public void PlayAnimation()
    {
        if(lightsAnimator)
        {
            lightsAnimator.Play("Start");
        }
    }
}
