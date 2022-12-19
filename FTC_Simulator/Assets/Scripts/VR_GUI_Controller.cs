using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Management;

public class VR_GUI_Controller : MonoBehaviour
{
    bool vr_started = false;
    public GameObject vrcamera;

    private void Start()
    {
        // Go through the command line and see if VR is enabled
        string[] commandlineargs = System.Environment.GetCommandLineArgs();

        bool runvr = false;
        foreach( string currarg in commandlineargs)
        {
            if( (currarg.ToLower() == "-runvr") || (currarg.ToLower() == "-runxr") || (currarg.ToLower() == "-dovr") || (currarg.ToLower() == "-doxr"))
            {
                runvr = true;
                break;
            }
        }

        // Start VR if applicable
        if( runvr )
        {
            StartVR();
        }
       
    }

    public void StartVR()
    {
        vr_started = true;
        MyUtils.XR_Start();
        vrcamera.SetActive(true);
    }

    public void StopVR()
    {
        vrcamera.SetActive(false);
        if (vr_started)
        {

            MyUtils.XR_Stop();
        }
    }

    private void OnDisable()
    {
        StopVR();
    }

}
