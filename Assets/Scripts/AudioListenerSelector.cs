using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AudioListenerSelector : MonoBehaviour
{
    public AudioListener otherListener;
    public AudioListener myListener;
    private bool old_enabled = true;


    private void OnEnable()
    {
        myListener = GetComponent<AudioListener>();
       
        if( myListener && otherListener)
        {
            old_enabled = otherListener.enabled;
            otherListener.enabled = false;
            myListener.enabled = true;
        }      
    }

    private void OnDisable()
    {
        if (myListener && otherListener)
        {
            otherListener.enabled = old_enabled;
            myListener.enabled = false;
        }
    }

}
