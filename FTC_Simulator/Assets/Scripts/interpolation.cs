using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class interpolation : MonoBehaviour {

    public bool disable = false;
    public bool disable2 = false; // disable is ored with disable2, allowing this to be a mask
    public bool do_not_push = false;  // Prevents interpolation from pushing the changes to the object by itself

    // Globals
    public Vector3 lastPosition;
    public Vector3 velocity;

    public Vector3 lastRotation;
    public Vector3 angularVelocity;

    public float time_last_v = 0f;
    public float time_last_a = 0f;
    public float my_time_last_v = 0;
    public float my_delta_time_last_v = 0;
    public float my_time_last_a = 0;
    public float my_delta_time_last_a = 0;
    public bool reset_delta_time = true;
    public bool pauseUpdates = false;

    // Decide which one to use
    private bool useLocalsv = false;
    private bool useLocalsa = false;

    // For BandwidthHelper, these wil lstore the desired new locations
    public Vector3 calculated_pos;
    public Vector3 calculated_angle;

    bool initialized = false; // Prevent changing of position until Start() has run to allow other Components to be up and ready 


    private void Start()
    {
        initialized = true;
        pauseUpdates = false;
    }

    private void SetPosInternal(Vector3 newPos, int timestamp = -1)
    {
        // Calculate velocities
        // But set velocity really small if this is the first time we are running this or our time_delta = 0
        float new_time = ((float) timestamp) / 1000f;
        if ( timestamp < 0)
        {
            new_time = Time.time;
        }
        float time_delta = new_time - time_last_v;
        time_last_v = new_time;
        my_time_last_v = Time.time;
        my_delta_time_last_v = 0;
       

        // If time_delta is <= 0, then skip this frame
        if (time_delta < 0 )
        {
            lastPosition = newPos;
            // Reset delta_time_with_server
            reset_delta_time = true;
            return;
        }
        if( time_delta < 0.001f)
        {
            time_delta = 0.001f;
        }
       
        velocity = (newPos - lastPosition) / time_delta;
        if(!GLOBALS.INTERPOLATE || disable || disable2) { velocity *= 0; }

        lastPosition = newPos;

        // Push output to object
        Update();
    }

 

    private void SetRotInternal(Quaternion newRot, int timestamp = -1)
    {
        SetRotInternal(newRot.eulerAngles, timestamp);
    }

    public Vector3 delta_angle_found;

    private void SetRotInternal(Vector3 newRot, int timestamp = -1)
    {
        // Calculate velocities
        // But set velocity really small if this is the first time we are running this or our time_delta = 0
        float new_time = (float)timestamp / 1000f;
        if (timestamp < 0)
        {
            new_time = Time.time;
        }
        float time_delta = new_time - time_last_a;
        time_last_a = new_time;
        my_time_last_a = Time.time;
        my_delta_time_last_a = 0;

        // If time_delta is <= 0, then skip this frame
        if (time_delta < 0)
        {
            lastRotation = newRot;
            // Reset delta_time_with_server
            reset_delta_time = true;
            return;
        }

        if( time_delta < 0.001f)
        {
            time_delta = 0.001f;
        }

        angularVelocity = (newRot - lastRotation);


        // If there was a rotational wrapping, then confine it to +/- 180Deger
        angularVelocity.x = (float) MyUtils.AngleWrap(angularVelocity.x);
        angularVelocity.y = (float) MyUtils.AngleWrap(angularVelocity.y);
        angularVelocity.z = (float) MyUtils.AngleWrap(angularVelocity.z);
        delta_angle_found = angularVelocity;

        // If any of the angles wrapped around (and created an absurd velocity, set angular velocity to 0
        if ( (Mathf.Abs(angularVelocity.x) > 90f) || (Mathf.Abs(angularVelocity.y) > 90f) || (Mathf.Abs(angularVelocity.z) > 90f))
       {
            angularVelocity = Vector3.zero;
       }

        angularVelocity /= time_delta;
        if (!GLOBALS.INTERPOLATE || disable || disable2) { angularVelocity = Vector3.zero; }


        lastRotation = newRot;

        // Push Change
        Update();
    }

    public void SetPosition(Vector3 newPos, int timestamp = -1)
    {
        useLocalsv = false;
        SetPosInternal(newPos, timestamp);
    }


    public void SetLocalPosition(Vector3 newPos, int timestamp = -1)
    {
        useLocalsv = true;
        SetPosInternal(newPos, timestamp);
    }


    public void SetRotation(Vector3 newRot, int timestamp = -1)
    {
        useLocalsa = false;
        SetRotInternal(newRot, timestamp);
    }


    public void SetLocalRotation(Vector3 newRot, int timestamp = -1)
    {
        useLocalsa = true;
        SetRotInternal(newRot, timestamp);
    }


    // Update is called once per frame
    void Update() {

        if( ! initialized) { return;  }

        // Calculate new positions
        UpdatePos();

        // If we have a BandwidthHelper attached, let it take care of updating
        if (do_not_push) { return; }

        // If we have pause enabled, don't do anything
        if (pauseUpdates) { return; }

        // Update actuale object
        if (useLocalsv == false)
        {
            transform.position = calculated_pos;
        }
        else
        {
            transform.localPosition = calculated_pos;
        }

        if (useLocalsa == false)
        {

            transform.eulerAngles = calculated_angle;
        }
        else
        {
            transform.localEulerAngles = calculated_angle;
        }
    }

    public void DoUpdate() // Allow others to force call update
    {
        Update();
    }

    // Calculates the extrapolated position and angle
    public void UpdatePos()
    {
        if (!GLOBALS.now_paused)
        {
            my_delta_time_last_v += Time.time - my_time_last_v;
            my_delta_time_last_a += Time.time - my_time_last_a;
        }

        float speed_multiplier = 1f;

        if( GLOBALS.now_playing)
        {
            speed_multiplier = GLOBALS.playback_speed;
        }

        my_time_last_v = Time.time;
        my_time_last_a = Time.time;

        calculated_pos = lastPosition + velocity * my_delta_time_last_v * speed_multiplier;
        calculated_angle = lastRotation + angularVelocity * my_delta_time_last_a * speed_multiplier;
    }

    public void StopMovement()
    {
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;

        time_last_v = 0;
        time_last_a = 0;
    }
}
