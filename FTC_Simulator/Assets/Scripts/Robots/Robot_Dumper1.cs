using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_Dumper1 : RobotInterface3D {

    public HingeJoint dumper;
    public HingeJoint gripper_1;
    public HingeJoint gripper_2;
    public ConfigurableJoint lift;

    public ConfigurableJoint collector_l;
    public ConfigurableJoint collector_r;
    public ConfigurableJoint collector_mid_l;
    public ConfigurableJoint collector_mid_r;
    public ConfigurableJoint collector_out_l;
    public ConfigurableJoint collector_out_r;


    public float dumper1_wheelspeed = 20;
    public float dumper1_reversspeed = 40;
    bool gripped = false;
    bool grip_button_last = false;
    float lastResetTime = 0;
    bool a_button_last = false;

    enum collectingStates : int
    {
        off = 0,
        onNormal,
        reverse
    }
    collectingStates collecting_state = collectingStates.off;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    void deactivateGrippers()
    {
        JointSpring g1 = gripper_1.spring;
        g1.targetPosition = 0;
        gripper_1.spring = g1;

        JointSpring s1 = gripper_2.spring;
        s1.targetPosition = -0;
        gripper_2.spring = s1;
    }
    void activateGrippers()
    {
        JointSpring g1 = gripper_1.spring;
        g1.targetPosition = 40;
        gripper_1.spring = g1;

        JointSpring s1 = gripper_2.spring;
        s1.targetPosition = -40;
        gripper_2.spring = s1;
    }


    // Do Pushbot updates
    public override void Update_Robot()
    {
        if (gamepad1_b)
        {
            JointSpring s = dumper.spring;
            s.targetPosition = 0;
            dumper.spring = s;

            ConfigurableJoint l = lift;
            l.targetPosition = new Vector3(0, 0, 0);
            lift = l;

            gripped = true;
            lastResetTime = Time.fixedTime;
        }
        if (Time.fixedTime - lastResetTime > 0.6 && lastResetTime != -1)
        {
            gripped = false;
            lastResetTime = -1;
        }



        if (gamepad1_x)
        {
            JointSpring s = dumper.spring;
            s.targetPosition = -117;
            dumper.spring = s;

        }

        if (gamepad1_dpad_up)
        {
            ConfigurableJoint l = lift;
            l.targetPosition = new Vector3(0, -0.6f, 0);
            lift = l;

        }
        bool grip_button = gamepad1_right_bumper || gamepad1_dpad_down || gamepad1_dpad_up;
        if (grip_button && !grip_button_last)
        {
            gripped = !gripped;
        }
        grip_button_last = grip_button;

        if (gripped)
        {
            activateGrippers();
        }
        else
        {
            deactivateGrippers();
        }


        if (gamepad1_y)
        {
            collecting_state = collectingStates.reverse;
        }

        if (gamepad1_a && !a_button_last)
        {
            //switch the collecting state to off if we are on, to on if we are off
            if (collecting_state == collectingStates.onNormal)
            {
                collecting_state = collectingStates.off;
            }
            else
            {
                collecting_state = collectingStates.onNormal;
            }
        }
        a_button_last = gamepad1_a;



        if (collecting_state == collectingStates.onNormal)
        {
            collector_l.targetAngularVelocity = new Vector3(dumper1_wheelspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(-1f * dumper1_wheelspeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(0, 0, 0);
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
        if (collecting_state == collectingStates.reverse)
        {
            collector_l.targetAngularVelocity = new Vector3(-1f * dumper1_reversspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(dumper1_reversspeed, 0, 0);
            collector_mid_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_mid_r.targetAngularVelocity = new Vector3(2f * dumper1_reversspeed, 0, 0);

            collector_out_l.targetAngularVelocity = new Vector3(-2f * dumper1_reversspeed, 0, 0);
            collector_out_r.targetAngularVelocity = new Vector3(2f * dumper1_reversspeed, 0, 0);
        }
    }

}