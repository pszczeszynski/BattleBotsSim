using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_SnailBot2 : RobotInterface3D {

    public ConfigurableJoint collector_l;
    public ConfigurableJoint collector_r;
    public ConfigurableJoint collector_mid_l;
    public ConfigurableJoint collector_mid_r;
    public ConfigurableJoint collector_out_l;
    public ConfigurableJoint collector_out_r;
    public ConfigurableJoint collector_shooter1;
    public ConfigurableJoint collector_shooter2;
    public ConfigurableJoint collector_shooter3;

    public ballshooting ball_feeder;

    public float dumper1_wheelspeed = 40;
    public float dumper1_reversspeed = 40;
    public float ball_feeder_hinge_speed = -600f; // Wheels that move ball from colelcted to shooting
    public float ball_feeder_speed = 1.5f; // Wheels that move ball from colelcted to shooting


    enum collectingStates : int
    {
        off = 0,
        onNormal,
        reverse
    }
    collectingStates collecting_state = collectingStates.off;

    bool last_button_a = false;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    // Do Pushbot updates
    public override void Update_Robot()
    {
        bool force_part_of_intake_on = false;

        // **********************************
        // Feed balls through flywheels
        if (gamepad1_x)
        {
            ball_feeder.hard_stop = false;
            ball_feeder.speed = ball_feeder_speed;
            force_part_of_intake_on = true; // Force part of the intake on (inside the chute)
        }
        else
        {
            ball_feeder.speed = ball_feeder_speed/4f;
            ball_feeder.hard_stop = true;
        }

        // ****************************************************
        // Intake

        // *********************
        // Ball Intake
        // If ypressed, rever
        bool intake_reverse = false;
        if (gamepad1_y)
        {
            intake_reverse = true;
            ball_feeder.speed = -1f * ball_feeder_speed;
            ball_feeder.hard_stop = false;
        }


        if (gamepad1_a && (last_button_a != gamepad1_a)) // if a is pressed make the intake go normal
        {
            if (collecting_state == collectingStates.off)
            {
                collecting_state = collectingStates.onNormal;
            }
            else
            {
                collecting_state = collectingStates.off;
            }
        }
        last_button_a = gamepad1_a;


        // ************************************
        // State-machine decoding. 
        // Not using as a state machine, but left it as such in case we want to make it into one in the future
        if (collecting_state == collectingStates.onNormal)
        {
            collector_l.targetAngularVelocity = new Vector3(dumper1_wheelspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(-1f * dumper1_wheelspeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(2f * dumper1_wheelspeed, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(-2f * dumper1_wheelspeed, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(2f * dumper1_wheelspeed, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(-2f * dumper1_wheelspeed, 0, 0);
            
            collector_shooter1.targetAngularVelocity = new Vector3(-2f*dumper1_wheelspeed, 0, 0);
            collector_shooter2.targetAngularVelocity = new Vector3(-2f*dumper1_wheelspeed, 0, 0);
            collector_shooter3.targetAngularVelocity = new Vector3(-2f*dumper1_wheelspeed, 0, 0);

        }
        if (collecting_state == collectingStates.off)
        {
            collector_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(0, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(0, 0, 0);

            collector_shooter1.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_shooter2.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_shooter3.targetAngularVelocity = new Vector3(0, 0, 0);
        }
        if (intake_reverse)
        {
            collector_l.targetAngularVelocity = new Vector3(-1f * dumper1_reversspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(dumper1_reversspeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(-2f * dumper1_reversspeed, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(2f * dumper1_reversspeed, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(-2f * dumper1_reversspeed, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(2f * dumper1_reversspeed, 0, 0);

            collector_shooter1.targetAngularVelocity = new Vector3( 2f*dumper1_wheelspeed, 0, 0);
            collector_shooter2.targetAngularVelocity = new Vector3(2f * dumper1_wheelspeed, 0, 0);
            collector_shooter3.targetAngularVelocity = new Vector3(2f * dumper1_wheelspeed, 0, 0);
        }

        // Override part of the intake to be on
        if(force_part_of_intake_on)
        {
            collector_shooter1.targetAngularVelocity = new Vector3(-2f * dumper1_wheelspeed, 0, 0);
            collector_shooter2.targetAngularVelocity = new Vector3(-2f * dumper1_wheelspeed, 0, 0);
            collector_shooter3.targetAngularVelocity = new Vector3(-2f * dumper1_wheelspeed, 0, 0);
        }
    }


}