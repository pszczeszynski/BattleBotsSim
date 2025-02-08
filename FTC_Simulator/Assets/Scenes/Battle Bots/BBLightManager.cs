using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.SceneManagement;

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

    public TMP_Dropdown quality;

    float lightMultiplier = 1.0f;


    // Start is called before the first frame update
    void Start()
    {
        // Find all the lights
        foreach (Light currLight in bottomLightsLeftParent.GetComponentsInChildren<Light>())
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

        if (quality != null)
        {
            QualitySettings.SetQualityLevel(quality.value);
        }

        lightsAnimator = GetComponent<Animator>();
    }

    public void UpdateQuality(int newsetting)
    {
        QualitySettings.SetQualityLevel(newsetting);

        GameObject[] allroot = SceneManager.GetActiveScene().GetRootGameObjects();
        foreach (GameObject root in allroot)
        {
            foreach (MinQualityLevel child in root.GetComponentsInChildren<MinQualityLevel>(true))
            {
                child.Enforce();
            }

        }

    }

    // Update is called once per frame
    void Update()
    {
        UpdateLights();

    }

    public void UpdateLights()
    { 

        if(QualitySettings.GetQualityLevel() <= 1)
        {
            lightMultiplier = 2.0f; ;
        }
        else
        {
            lightMultiplier = 1.0f;
        }

    
        // Update all Lights
        foreach (Light currLight in bottomLightsLeftParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = lightMultiplier * bottomLightLeftIntensity;
        }

        foreach (Light currLight in bottomLightsRightParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = lightMultiplier * bottomLightRightIntensity;
        }

        foreach (Light currLight in topLightsParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = lightMultiplier * topLightIntensity;
        }

        foreach (Light currLight in spotLightsParent.GetComponentsInChildren<Light>())
        {
            currLight.intensity = lightMultiplier * spotLightIntensity;
        }
    }

    public void PlayAnimation()
    {
        if(lightsAnimator)
        {
            lightsAnimator.enabled = true;

            lightsAnimator.Play("Start");
        }
    }

    public void DisableAnimator()
    {
        lightsAnimator.enabled = false;
    }
}
