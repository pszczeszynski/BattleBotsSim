using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChampsInit : MonoBehaviour
{
    GameObject lighting;
    Color old_color;

    private void OnEnable()
    {
        // Find the Lighting top level and disable it from existing field
        lighting = GameObject.Find("Lighting");
        if( lighting ) { lighting.SetActive(false); }

        // Turn down the ambient light
        old_color = RenderSettings.ambientLight;
        RenderSettings.ambientLight = new Color(0.2f, 0.2f, 0.2f);

        // Turn of skybox
        Skybox myskybox = GameObject.FindObjectOfType<Skybox>();
        if( myskybox )
        {
            myskybox.enabled = false;
        }

    }

    private void OnDisable()
    {
        // Resotre any old settings
        if (lighting) { lighting.SetActive(true); }
        RenderSettings.ambientLight = old_color;

        // Turn on skybox
        Skybox myskybox = GameObject.FindObjectOfType<Skybox>();
        if (myskybox)
        {
            myskybox.enabled = enabled;
        }
    }

}
