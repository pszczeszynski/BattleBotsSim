using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_SnailBot : RobotInterface3D {

    public ConfigurableJoint collector_l;
    public ConfigurableJoint collector_r;
    public ConfigurableJoint collector_mid_l;
    public ConfigurableJoint collector_mid_r;
    public ConfigurableJoint collector_out_l;
    public ConfigurableJoint collector_out_r;
    public HingeJoint ball_feeder_hinge1;
    public HingeJoint ball_feeder_hinge2;
    public ballshooting ball_feeder;
    public ballshooting ball_feeder_pre; // same as ball_feeder, but doesn't apply hard stop. This is required so that you don't need to run the intake all the time in order to shoot ball.


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
        // **********************************
        // Feed balls through flywheels
        if (gamepad1_x)
        {
            ball_feeder.hard_stop = false;
            ball_feeder.speed = ball_feeder_speed;
            ball_feeder_pre.speed = ball_feeder_speed;
            JointMotor outmotor = ball_feeder_hinge1.motor;
            outmotor.targetVelocity = ball_feeder_hinge_speed;
            ball_feeder_hinge1.motor = outmotor;

            outmotor = ball_feeder_hinge2.motor;
            outmotor.targetVelocity = -1f*ball_feeder_hinge_speed;
            ball_feeder_hinge2.motor = outmotor;
        }
        else
        {
            ball_feeder.speed = 1f;
            ball_feeder_pre.speed = 0f;
            ball_feeder.hard_stop = true;
            JointMotor outmotor = ball_feeder_hinge1.motor;
            outmotor.targetVelocity = 0f;
            ball_feeder_hinge1.motor = outmotor;

            outmotor = ball_feeder_hinge2.motor;
            outmotor.targetVelocity = 0f;
            ball_feeder_hinge2.motor = outmotor;
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
            ball_feeder_pre.speed = -1f * ball_feeder_speed;
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

        }
        if (collecting_state == collectingStates.off)
        {
            collector_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(0, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(0, 0, 0);
        }
        if (intake_reverse)
        {
            collector_l.targetAngularVelocity = new Vector3(-1f * dumper1_reversspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(dumper1_reversspeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(-2f * dumper1_reversspeed, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(2f * dumper1_reversspeed, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(-2f * dumper1_reversspeed, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(2f * dumper1_reversspeed, 0, 0);
        }
    }

  
}