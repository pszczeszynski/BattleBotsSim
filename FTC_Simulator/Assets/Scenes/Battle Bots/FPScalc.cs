using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class FPScalc : MonoBehaviour
{
    public float fpsavg = 0;
    public float newfps = 0;
    float lasttime = 0;
    float avgtime = 0.25f; //averaging duration
    public TextMeshProUGUI label;
    public TextMeshProUGUI physicsLabel;
    public int physicsFrames = 0;
    public float calcfixedTime = 0;
    float physicsLastTime = 0;
    public float physicsAvgSpeed = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        lasttime = Time.realtimeSinceStartup;
        physicsLastTime = lasttime;


        if( label == null)
        {
            label = GetComponent<TextMeshProUGUI>();
        }
    }



    private void FixedUpdate()
    {
        physicsFrames++;    
    }

    // Update is called once per frame
    void Update()
    {
        float newtime = Time.realtimeSinceStartup;
        float deltatime = newtime - lasttime;

        // Should never happen, but check for a 0 deltatime
        if( deltatime <= 0) { return;  }

        newfps = 1.0f / deltatime;
        lasttime = newtime;

        float scalefactor = deltatime / avgtime;
        if( scalefactor > 1.0f)
        { scalefactor = 1.0f; }

        // Calculate new fps
        fpsavg = newfps * scalefactor + (1.0f - scalefactor) * fpsavg;

        // Update it
        if (label != null)
        {
            label.text = fpsavg.ToString("F0");
        }

        // Check for physics throttling
        if( newtime - physicsLastTime > 0.25f)
        {
            float physicsDeltaTime = newtime - physicsLastTime;
            physicsLastTime = newtime;

            // Avoid div/0
            if( physicsFrames <= 0 ) { return; }
            calcfixedTime = physicsDeltaTime / ((float)physicsFrames);
            physicsFrames = 0;

            // check for physics throttling
            physicsAvgSpeed = physicsAvgSpeed * 0.75f + 0.25f *  Time.fixedDeltaTime /calcfixedTime;
   
            if(physicsLabel != null)
            {
                physicsLabel.text = physicsAvgSpeed.ToString("F2");
            }
 
        }

    }
}
