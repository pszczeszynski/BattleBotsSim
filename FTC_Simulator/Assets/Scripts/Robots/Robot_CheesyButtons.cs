using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;


// *******************************
// ROBOT interface for Scorpius 2
// *******************************



public class Robot_CheesyButtons : RobotInterface3D {


    

    // Main arm stuff
    public HingeJoint mainArmJoint;
    public float arm_speed = 75f;
    public float arm_max_limit = 180f;
    public float arm_min_limit = -180f;


    private float turn_scale_original = -1f;
    private bool init_done = false;

    // Sliding Arm
    public ConfigurableJoint hand;
    public ConfigurableJoint lift_arm;
    public ConfigurableJoint dummy_lift_arm;
    public float lift_down_pos = 0;
    public float lift_up_pos = 10f;
    public float lift_speed = 10f;

    // Fingers
    public HingeJoint fingers;
    public float finger_speed = 100f;

    bool gamepad_a_previous = false;
    bool gamepad_b_previous = false;
    bool gamepad_y_previous = false;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    // Do Pushbot updates
    public override void Update_Robot()
    {
     

        // Remember some variable sthat we scale later during init
        if (!init_done)
        {
            turn_scale_original = turn_scale;
            init_done = true;
        }

        // ****************** ARM ROTATION ***************
        // UP rotation
        float scaled_arm_speed = arm_speed;

        scaled_arm_speed *= 1f - (lift_arm.targetPosition.x - lift_down_pos) / (lift_up_pos - lift_down_pos) * 0.8f;

        if (gamepad1_dpad_right)
        {
            MyMoveHinge(mainArmJoint, arm_max_limit, scaled_arm_speed);
        }

        // UP Rotation
        if (gamepad1_dpad_left)
        {
            MyMoveHinge(mainArmJoint, arm_min_limit, scaled_arm_speed);
        }


        // Reduce turning speed if arm is down
        if (mainArmJoint.spring.targetPosition < arm_min_limit + 40f)
        {
            float scaler = (arm_min_limit - mainArmJoint.spring.targetPosition) / -40f;
            if (scaler < 0.3f)
            { scaler = 0.3f; }

            turn_scale = turn_scale_original * scaler;
        }
        else
        {
            turn_scale = turn_scale_original;
        }

        // ******************** ARM Extending *************************
        // Move bucket up/down
        if (gamepad1_dpad_down )
        {
            hand.targetPosition = new Vector3(MoveTowards(hand.targetPosition.x, lift_down_pos+0.68f, hand.targetPosition.x, Time.deltaTime * lift_speed ), 0, 0);
            if( hand.targetPosition.x >= lift_arm.targetPosition.x + 0.67f )
            {
                lift_arm.targetPosition = new Vector3(MoveTowards(lift_arm.targetPosition.x, lift_down_pos, lift_arm.targetPosition.x, Time.deltaTime * lift_speed), 0, 0);
                dummy_lift_arm.targetPosition = new Vector3(MoveTowards(dummy_lift_arm.targetPosition.x, lift_down_pos / 2f, dummy_lift_arm.targetPosition.x, Time.deltaTime * lift_speed / 2f), 0, 0);
            }

        }

        if (gamepad1_dpad_up )
        {
            hand.targetPosition = new Vector3(MoveTowards(hand.targetPosition.x, lift_up_pos, hand.targetPosition.x, Time.deltaTime * lift_speed), 0, 0);
            if (hand.targetPosition.x <= lift_arm.targetPosition.x)
            {
                lift_arm.targetPosition = new Vector3(MoveTowards(lift_arm.targetPosition.x, lift_up_pos, lift_arm.targetPosition.x, Time.deltaTime * lift_speed), 0, 0);
                dummy_lift_arm.targetPosition = new Vector3(MoveTowards(dummy_lift_arm.targetPosition.x, lift_up_pos / 2f, dummy_lift_arm.targetPosition.x, Time.deltaTime * lift_speed / 2f), 0, 0);
            }
        }

        // Close finger
        if (gamepad1_x)
        {
            JointMotor motor  = fingers.motor;
            motor.targetVelocity = -1f * finger_speed;
            fingers.motor = motor;
        }

        if (gamepad1_b)
        {
            JointMotor motor = fingers.motor;
            motor.targetVelocity = finger_speed;
            fingers.motor = motor;
        }

    }

    // Moves joint by a small delta. If reached destination, return true
    // target is a positive number, but all positions in the spring are negative.
    // I probably should have left everything negative to avoid confusion...
    bool MyMoveHinge(HingeJoint hinge, float target, float speed)
    {
        return MoveHinge(hinge, target, speed) == target;
    }

}

