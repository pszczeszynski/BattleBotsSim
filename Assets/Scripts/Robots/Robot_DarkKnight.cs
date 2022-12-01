using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_DarkKnight : RobotInterface3D {

    // Intake - bottom
    public ConfigurableJoint collector_l;
    public ConfigurableJoint collector_r;
    public ConfigurableJoint collector_mid_l;
    public ConfigurableJoint collector_mid_r;
    public ConfigurableJoint collector_out_l;
    public ConfigurableJoint collector_out_r;
    
    // Intake - on lift
    public HingeJoint top_intake;
    public ballshooting top_intake_hitbox; 


    // Ramp lift
    public ConfigurableJoint ball_lift;

    public float intake_wheelspeed = 20;
    public float intake_reversespeed = 20;
    public float ball_feeder_hinge_speed = -200f; // Wheels that move ball from colelcted to shooting
    public float ball_feeder_speed = 0.5f; // the hitbox's ball speed
    public float lift_speed = 0.1f;
    public float lift_top_pos = -0.9f;

    enum collectingStates : int
    {
        off = 0,
        onNormal,
        reverse
    }
    collectingStates top_states = collectingStates.off;
    collectingStates intake_states = collectingStates.off;

    bool last_button_x = false;
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

        // Control setup
        //
        // button_x pressed: toggle on/off
        // button_y pressed: top_collector reverse
        //
        // button_a pressed: intake collects balls toggle
        // button_b pressed: intake reverses
        //
        // Dpad up - move lift up
        // Dpaf down - move lift down

        // *************************
        // TOP Collector
        bool collector_reverse = false;

        if ((gamepad1_x != last_button_x) && gamepad1_x) // intakes
        {
            if (top_states == collectingStates.off) // Time to turn it on
            {
                top_states = collectingStates.onNormal;
            }
            else
            {
    
                top_states = collectingStates.off;
            }
        }
        last_button_x = gamepad1_x;

        if (gamepad1_y) // Reverses while pressed down
        {
            collector_reverse = true;
        }

        


        if (top_states == collectingStates.off)
        {
            // Always apply some upward pressure
            // top_intake_hitbox.speed = -0.1f * ball_feeder_speed;
            JointMotor outmotor = top_intake.motor;
            outmotor.targetVelocity = 0f;
            top_intake.motor = outmotor;
        }


        if (top_states == collectingStates.onNormal)
        {
            // top_intake_hitbox.speed = -1f * ball_feeder_speed;
            JointMotor outmotor = top_intake.motor;
            outmotor.targetVelocity = ball_feeder_hinge_speed;
            top_intake.motor = outmotor;
        }

        if( collector_reverse)
        {
            top_intake_hitbox.speed = ball_feeder_speed;
            JointMotor outmotor = top_intake.motor;
            outmotor.targetVelocity = -1f * ball_feeder_hinge_speed;
            top_intake.motor = outmotor;
        }
        else
        {
            top_intake_hitbox.speed = 0f;
        }


        // *********************
        // Ball Intake
        // If b pressed, rever
        bool intake_reverse = false;
        if (gamepad1_b)
        {
            intake_reverse = true;
        }

        if ( (gamepad1_a != last_button_a) &&  gamepad1_a) // throw balls out
        {
            if(intake_states == collectingStates.off)
            {
                intake_states = collectingStates.onNormal;
            }
            else
            {
                intake_states = collectingStates.off;
            }
        }
        last_button_a = gamepad1_a;

        if (intake_states == collectingStates.onNormal)
        {
            collector_l.targetAngularVelocity = new Vector3(intake_wheelspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(-1f * intake_wheelspeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(2f * intake_wheelspeed, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(-2f * intake_wheelspeed, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(2f * intake_wheelspeed, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(-2f * intake_wheelspeed, 0, 0);

        }
        if (intake_states == collectingStates.off)
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
            collector_l.targetAngularVelocity = new Vector3(-1f * intake_reversespeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(intake_reversespeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(-2f * intake_reversespeed, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(2f * intake_reversespeed, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(-2f * intake_reversespeed, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(2f * intake_reversespeed, 0, 0);
        }

        // *********************************
        // LIFT stuff

        // Move lift up
        if (gamepad1_dpad_up)
        {
            // If we haven't reached the target, then keep lifting
            if (ball_lift.targetPosition.y > lift_top_pos) // Lift pos is negative
            {
                Vector3 target = ball_lift.targetPosition;

                target.y = MoveTowards(target.y, lift_top_pos, target.y, Time.deltaTime * lift_speed);
                ball_lift.targetPosition = target;
            }
        }

        // Move lift down
        if (gamepad1_dpad_down)
        {
            // If we haven't reached the target, then keep lifting
            if (ball_lift.targetPosition.y < 0) // Lift pos is negative
            {
                Vector3 target = ball_lift.targetPosition;

                target.y = MoveTowards(target.y, 0f, target.y, Time.deltaTime * lift_speed);
                ball_lift.targetPosition = target;
            }
        }

    }

    // If robot has a bumper, set the name
    public override void SetName(string name)
    {
        // Set the base items
        base.SetName(name);

        bool name_tag_found = false;
        // Set our own items
        Transform mynametag = transform.Find("Body/NametagB1");
        if (mynametag)
        {
            mynametag.GetComponent<TMPro.TextMeshPro>().text = name;
            name_tag_found = true;
        }
        // Set our own items
        mynametag = transform.Find("Body/NametagB2");
        if (mynametag)
        {
            mynametag.GetComponent<TMPro.TextMeshPro>().text = name;
            name_tag_found = true;
        }

        // Turn off main name-tag
        if (name_tag_found)
        {
            mynametag = transform.Find("Body/Nametag");
            mynametag.gameObject.SetActive(false);
        }
    }

}