using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Scorpius 3
// *******************************



public class Robot_Spectator : RobotInterface3D {

    public bool disable_motion = false;
    double my_z_rotation = 0f; // degrees of rotation
    double my_y_rotation = 0f; // Degrees of rotation

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }
    public override void Update_Robot()
    {
        if (disable_motion) { return; }

        // Move Avator
        // Joystick left-x sets y-rotation

        // Set dead-band 
        float left_stick_y = gamepad1_left_stick_y;
        float left_stick_x = gamepad1_left_stick_x;
        if (Math.Abs(left_stick_y) < 0.07) { left_stick_y = 0f; }
        if (Math.Abs(left_stick_x) < 0.07) { left_stick_x = 0f; }

        float right_stick_y = gamepad1_right_stick_y;
        float right_stick_x = gamepad1_right_stick_x;
        if (Math.Abs(right_stick_y) < 0.07) { right_stick_y = 0f; }
        if (Math.Abs(right_stick_x) < 0.07) { right_stick_x = 0f; }

        // Undo rotation, only allow for y rotation
        rb_body.transform.localRotation = Quaternion.Euler(0, (float) my_y_rotation, 0);

        // Apply movement
        Vector3 movement = new Vector3(0, 0, 0);
           
        // Need to move it relative to current orientation. Using x-axis as the forward axis, z-axis as the side-to-side
        movement.x += -1f * left_stick_y * Time.deltaTime * 5f;
        movement.z += -1f * left_stick_x * Time.deltaTime * 5f;    
        rb_body.transform.Translate(movement, Space.Self);

            // Vector3 currpos_loc = rb_body.transform.localPosition;

            // Apply the rotation to movement
            // movement = rb_body.transform.localRotation * movement;

            // Add it to the curr pos
            // rb_body.transform.localPosition = currpos_loc + movement;

        // Move lift up
        if (gamepad1_dpad_up || gamepad1_dpad_down)
        {
            movement.x = 0;
            movement.z = 0;
            movement.y = Time.deltaTime * 2f;
            if (gamepad1_dpad_down) { movement.y *= -1f; }

            rb_body.transform.position = rb_body.transform.position + movement;

        }

        // Apply Limits
        Vector3 currpos = rb_body.transform.position;
        if( currpos.y > 10f ) { currpos.y = 10f; }
        if (currpos.y < 0f) { currpos.y = 0f; }
        if (currpos.x > 10f) { currpos.x = 10f; }
        if (currpos.x < -10f) { currpos.x = -10f; }
        if (currpos.z > 12f) { currpos.z = 12f; }
        if (currpos.z < -12f) { currpos.z = -12f; }

        rb_body.transform.position = currpos;

        // Apply rotation
        // Increment our rotation
        my_z_rotation += right_stick_y * Time.deltaTime * 150f;
        my_y_rotation += right_stick_x * Time.deltaTime * 150f;

        // apply angle wrap
        my_y_rotation = MyUtils.AngleWrap(my_y_rotation);

        // Limit z rotation to +/- 90
        if (my_z_rotation < -90f) { my_z_rotation = -90f; }
        if (my_z_rotation > 90f) { my_z_rotation = 90f; }

        rb_body.transform.localRotation = Quaternion.Euler(0, (float) my_y_rotation, (float) my_z_rotation * -1f);


        // Now record positions: if "A" is pressed at the same time as a keyboard number, it will assign that position to the keyboard key
        KeyCode alpha_pressed = KeyCode.None;

        for( KeyCode currkey = KeyCode.Alpha0; currkey <= KeyCode.Alpha9; currkey++)
        {
            // If the number is pressed, remember it and exit loop.
            if(Input.GetKey(currkey))
            {
                alpha_pressed = currkey;
                break;
            }
        }

        // Save the state
        if ( gamepad1_a && (alpha_pressed != KeyCode.None))
        {
            // Remember this location
            GLOBALS.GENERIC_DATA["SPEC" + alpha_pressed.ToString()] = rb_body.transform.localPosition.x + ":" + rb_body.transform.localPosition.y + ":" + rb_body.transform.localPosition.z + ":" +
                                                                    my_z_rotation + ":" + my_y_rotation;
            Settings.SavePrefs();
        } 
        // Load the state is button pressed
        else if((alpha_pressed != KeyCode.None))
        {
            if( GLOBALS.GENERIC_DATA.ContainsKey("SPEC" + alpha_pressed.ToString()))
            {
                string[] rawdata = GLOBALS.GENERIC_DATA["SPEC" + alpha_pressed.ToString()].Split(':');
                if( rawdata.Length >= 5)
                {
                    int i = 0;
                    Vector3 pos;
                    pos.x = float.Parse(rawdata[i++]);
                    pos.y = float.Parse(rawdata[i++]);
                    pos.z = float.Parse(rawdata[i++]);
                    my_z_rotation = double.Parse(rawdata[i++]);
                    my_y_rotation = double.Parse(rawdata[i++]);
                    rb_body.transform.localPosition = pos;
                    rb_body.transform.localRotation = Quaternion.Euler(0, (float)my_y_rotation, (float) my_z_rotation * -1f);
                }
            }
        }

    }
    // We do not have any movement, etc.. for now
    // This is called from FixedUpdate
    override public void UpdateMovement()
    {

    }

    public override void Start()
    {
        isSpectator = true;
        base.Start();
        SetKinematic(true);
    }


}