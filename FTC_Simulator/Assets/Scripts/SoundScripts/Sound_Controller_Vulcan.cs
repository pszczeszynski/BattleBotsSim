using System;
using System.Collections;
using System.Collections.Generic;
using FTC_Simulator.Assets.Scripts;
using UnityEngine;

public class Sound_Controller_Vulcan : MonoBehaviour
{
    AudioManager mySoundPlayer;
    Robot_Vulcan myController;

    Transform myBody;
    public Transform myLift;

    private void Start() {
        //get a link to our sound player
        mySoundPlayer = GetComponent<AudioManager>();
        myController = transform.GetComponent<Robot_Vulcan>();
    }

    private void Update() {
        if (!myBody)
        {
            myBody = transform.Find("Body");
        }
        PlayFlippingSound();
        PlayCollectingSound();
        PlayLiftSound();
    }

    Vector3 liftLastRelativePosition;
    private void PlayLiftSound()
    {
        Vector3 liftRelativePos = myBody.worldToLocalMatrix.MultiplyPoint(myLift.position);
        float changeMagnitude = (liftRelativePos - liftLastRelativePosition).magnitude;

        if (changeMagnitude > 0.001f && changeMagnitude < 10) {
            if (!mySoundPlayer.SoundIsPlaying("Lift")) {
                mySoundPlayer.Play("Lift", 0, 1);
            }
        } else {
            if (mySoundPlayer.SoundIsPlaying("Lift")) {
                mySoundPlayer.Stop("Lift", 0.1f);
            }
        }
        liftLastRelativePosition = liftRelativePos;
    }

    private void PlayCollectingSound()
    {
        if (myController.arm_state == Robot_Vulcan.armStates.start ||
            myController.arm_state == Robot_Vulcan.armStates.tilt_down )
        { 
            if(!mySoundPlayer.findSound("Collector").isPlaying) {
                mySoundPlayer.Play("Collector", 0);
            }
        } else {
            if(mySoundPlayer.findSound("Collector").isPlaying) {
                mySoundPlayer.Stop("Collector", 0.1f);
            }
        }
    }

    private void PlayFlippingSound()
    {
        // Debug.Log($"armstate: {myController.armstate}");
        if (myController.gamepad1_dpad_right || myController.gamepad1_dpad_left)
        {
            if (!mySoundPlayer.findSound("Grip").isPlaying)
            {
                mySoundPlayer.Play("Grip", 0);
            }
        }
        else
        {
            if (mySoundPlayer.findSound("Grip").isPlaying)
            {
                mySoundPlayer.Stop("Grip", 0.1f);
            }
        }
    }


    float AngleWrapDeg(float degrees) {
        float ret = degrees % 360;
        if(ret > 180) {
            ret -= 360;
        }
        return ret;
    }

    float AngleWrapRad(float rad) {
        float ret = rad % (2*Mathf.PI);
        if(ret > Mathf.PI) {
            ret -= 2 * Mathf.PI;
        }
        return ret;
    }

}
