using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LoadSkybox : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        // Get our skybox
        Skybox myskybox = GetComponent<Skybox>();

        if (myskybox == null) { return; }
      
        // Get the skybox material
        if ( GLOBALS.skybox == "None") 
        {
                myskybox.enabled = false;
                return; 
        }

        Material skybox_mat = Resources.Load("Skybox/" + GLOBALS.skybox) as Material;
        myskybox.material = skybox_mat;

    }

}
