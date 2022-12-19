using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RingShooter : ballshooting
{

    // When a ring is marked, mark it if it's in the nolaunchzone
    GenericFieldTracker nolaunchzone;

    // Do custom start initialization
    public override void MyStart()
    {
        // Do additional initializations if required by derived functions
        // Find the NoLaunchZone and extract the IR_FieldTracker
        GameObject go_nolaunchzone = GameObject.Find("NoLaunchZone");
        if (!go_nolaunchzone) { return; }

        nolaunchzone = go_nolaunchzone.GetComponent<GenericFieldTracker>();
    }

    override public void MarkGameElement(ball_data thisringdata)
    {
        // If no launchzone found, then find it
        if( !nolaunchzone)
        {
            MyStart();
            if( !nolaunchzone ) { return;  }
        }

        // Next see if we're inside the launch zone
        if (nolaunchzone.IsRobotInside(transform.root))
        {
            if (!thisringdata.flags.ContainsKey("Bad"))
            {
                thisringdata.flags.Add("Bad", "1");
            }
        }
        else
        {
            thisringdata.flags.Remove("Bad");
        }
    }


}


