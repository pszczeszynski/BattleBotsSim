using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Cupbot based off pushbot
// *******************************
//



public class Robot_Cupbot : RobotInterface3D {

    public float pushbot_arm_speed = 75f;
	
    //pushbot stuff
    public HingeJoint mainArmJoint;

    public float arm_max_limit = 180f;
    public float arm_min_limit = -180f;
    private float turn_scale_original = -1f;
    private bool init_done = false;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    // Do Pushbot updates
    public override void Update_Robot()
    {
        HingeJoint tempJoint = mainArmJoint;
        JointSpring temp = tempJoint.spring;

        if (!init_done)
        {
            turn_scale_original = turn_scale;
            init_done = true;
        }


        if (gamepad1_dpad_down)
        {
            temp.targetPosition += pushbot_arm_speed * Time.deltaTime;
            if( temp.targetPosition > arm_max_limit)
            {
                temp.targetPosition = arm_max_limit;
            }

        }
        if (gamepad1_dpad_up)
        {
            temp.targetPosition -= pushbot_arm_speed * Time.deltaTime;
            if (temp.targetPosition < arm_min_limit)
            {
                temp.targetPosition = arm_min_limit;
            }

        }
        tempJoint.spring = temp;
        mainArmJoint = tempJoint;

        // Reduce speed if arm is down
        if( temp.targetPosition > arm_max_limit - 60f )
        {
            float scaler = (arm_max_limit - temp.targetPosition) / 60f;
            if( scaler < 0.3f)
            { scaler = 0.3f; }

            turn_scale = turn_scale_original * scaler;
        }
        else
        {
            turn_scale = turn_scale_original;
        }

        if (gamepad1_x)
        {
        
        }

        if (gamepad1_a)
        {
         
        }

    }

}