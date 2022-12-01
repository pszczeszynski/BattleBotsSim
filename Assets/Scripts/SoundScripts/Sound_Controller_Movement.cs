using System;
using System.Collections;
using System.Collections.Generic;
using FTC_Simulator.Assets.Scripts;
using UnityEngine;

public class Sound_Controller_Movement : MonoBehaviour
{
    public Sound driveSound = null;
    public float master_volume = 0.2f;

    private AudioManager mySoundPlayer;
    private RobotInterface3D myRI3D;


    private void Start() {

        //get a link to our sound player
        mySoundPlayer = GetComponentInParent<AudioManager>();
        if( !mySoundPlayer ) mySoundPlayer = GetComponentInChildren<AudioManager>();

        myRI3D = GetComponentInParent<RobotInterface3D>();
        if( !myRI3D) myRI3D = GetComponentInChildren<RobotInterface3D>();

        if (mySoundPlayer && myRI3D)
        {
            // Create our straffing sound
            AudioClip loadedsound = Resources.Load<AudioClip>("Sounds/SampledSounds/MovementStraif_loop");

            // Now set all parameters
            driveSound = new Sound();
            driveSound.name = "move_strafe";            
            driveSound._clip = loadedsound;
            driveSound._volume = master_volume;
            driveSound._pitch = 1f;
            driveSound._spatialBlend = 1;
            driveSound._loop = false;
            driveSound.sourceLocation = myRI3D.transform.Find("Body");
            driveSound.Init(mySoundPlayer); // Need to do this before I can set any of the other 

            mySoundPlayer.AddSound(driveSound);
        }
    }

    private void Update() {
        if( !myRI3D || (driveSound == null) ) { return; }  // Failsafe
        PlayMovementSound();
    }

  

    bool wasPlayingForwardsSoundLast = false;
    public float targetVelocity = 0f;
    public float currVelocity = 0f;


    private void PlayMovementSound() {

        //this is how fast we are trying to go currently
        targetVelocity = myRI3D.getAverageMovementForceMagnitude();

        // This is how fast we are going
        currVelocity = myRI3D.getAveragVelocityMagnitude();
        
        if(currVelocity > 10f) return;
        if(currVelocity > 0.1f) 
        {
            // Play the drivesound if our speed is > 0.2f
            if(!wasPlayingForwardsSoundLast) {
                mySoundPlayer.Play(driveSound.name, 0,-1f, true);
                if(targetVelocity > 0.85f){
                    driveSound.time = 0.0f;
                } else {
                    driveSound.time = 5.0f;
                    driveSound._pitch = 0.8f;
                }
            }
            wasPlayingForwardsSoundLast = true;

            
            // While playing sound calculations
            // TBD: Need to update volume via "PLAY" command so it changes it over the network
            driveSound._volume = currVelocity < 1 ? currVelocity * master_volume : master_volume;
    
        } 
        else // Turn off the sound if it was playing
        {
            if (wasPlayingForwardsSoundLast) 
            {
                mySoundPlayer.Stop(driveSound.name, 0.1f,true);
            }

            wasPlayingForwardsSoundLast = false;
        }


        float newPitch = 0.35f + currVelocity * 0.65f;
        driveSound._pitch = driveSound._pitch * 0.9f + 0.1f * newPitch;
        if(driveSound._pitch > 1) driveSound._pitch = 1;
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
