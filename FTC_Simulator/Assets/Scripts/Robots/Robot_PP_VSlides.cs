using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Robot_Vulcan
// *******************************
//

public class Robot_PP_VSlides : RobotInterface3D {

    public ConfigurableJoint limb_arm5; // Actual hand
    public ConfigurableJoint limb_arm4; // Main lifting arm: 4th stage
    public ConfigurableJoint limb_arm3; // dummy arm: 3rd stage
    public ConfigurableJoint limb_arm2; // dummy arm: 2nd stage
    public float limb_arm_start = 0;
    public float limb_arm_stop = 1f;
    public float limb_arm_speed = 100;
    public float limb_arm_lvl1 = 0.3f;
    public float limb_arm_lvl2 = 0.6f;
    public float limb_arm_lvl3 = 0.9f;

    public ConfigurableJoint finger_l; // fingers
    public ConfigurableJoint finger_r; // fingers
    public float finger_speed = 100;
    public float finger_start_pos = 0;
    public float finger_end_pos = -0.2f;

    // Other variables for internal use
    private AudioManager audio_manager;

    // Initialize joints to minimum state
    // Ground level also oppens the hand
    enum LiftStatemachine
    {
        manual = 0,
        lvl_ground,
        lvl_low,
        lvl_mid,
        lvl_high
    };

    LiftStatemachine lift_state = LiftStatemachine.manual;

    public void Awake()
    {
        info =
            "D-Pad Left/Right: Open/Close fingers\n" +
            "Left/Right Trigger: Move lift down/up\n" + 
            "Button A,X,Y,B: Auto move lift to ground, low, mid and high\n" +
            info;
    }

    public override void Start()
    {
        base.Start();

    }

    public override void Init_Robot()
    {
        // Initialize local cache variables
        audio_manager = GetComponentInChildren<AudioManager>(); // Only 1 audio manager in scene.
    }


    // Update Call
    public override void Update_Robot()
    {

        bool lift_moving = false;
        bool fingers_moving = false;

        // ****************************
        // buttons move lift to the correct level
        if (gamepad1_a )
        {
            lift_state = LiftStatemachine.lvl_ground;
        }
        if( gamepad1_x)
        {
            lift_state = LiftStatemachine.lvl_low;
        }
        if (gamepad1_y)
        {
            lift_state = LiftStatemachine.lvl_mid;
        }
        if (gamepad1_b)
        {
            lift_state = LiftStatemachine.lvl_high;
        }

     

        // ****************************
        // d_pad_up/down - nothing at the moment
        if (gamepad1_dpad_down) 
        {
            
        }
        if (gamepad1_dpad_up) 
        {
            
        }

        float curr_pos = 0f;

        // Triggers give precision control over slide
        if (gamepad1_left_trigger > 0f)
        {
            lift_state = LiftStatemachine.manual;
            curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_start, limb_arm_speed * turning_scaler * gamepad1_left_trigger);

            if (curr_pos != limb_arm_start )
            {
                lift_moving = true;
            }
        }

        if (gamepad1_right_trigger > 0f)
        {
            lift_state = LiftStatemachine.manual;
            curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_stop, limb_arm_speed * turning_scaler * gamepad1_right_trigger);

            if (curr_pos != limb_arm_stop)
            {
                lift_moving = true;
            }
        }

        // ****************************
        // d_pad_left/right closes/opens fingers
        if (gamepad1_dpad_right)
        {
            fingers_moving = true;
            MoveSlide(finger_l, Axis.x, finger_end_pos, turning_scaler * finger_speed);
            MoveSlide(finger_r, Axis.x, finger_end_pos, turning_scaler * finger_speed);           

        }
        if (gamepad1_dpad_left)
        {
            fingers_moving = true;
            MoveSlide(finger_l, Axis.x, finger_start_pos, turning_scaler * finger_speed);
            MoveSlide(finger_r, Axis.x, finger_start_pos, turning_scaler * finger_speed);
        }   

        switch( lift_state)
        {
            case LiftStatemachine.lvl_ground:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_start, limb_arm_speed);
                if (curr_pos  == limb_arm_start)
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { 
                    lift_moving = true;
                    fingers_moving = true;
                }

                MoveSlide(finger_l, Axis.x, finger_start_pos, finger_speed);
                MoveSlide(finger_r, Axis.x, finger_start_pos,  finger_speed);
                break;

            case LiftStatemachine.lvl_low:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_lvl1, limb_arm_speed);
                if (curr_pos == limb_arm_lvl1)
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { lift_moving = true; }

                break;

            case LiftStatemachine.lvl_mid:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_lvl2, limb_arm_speed);
                if (curr_pos == limb_arm_lvl2)
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { lift_moving = true; }

                break;

            case LiftStatemachine.lvl_high:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_lvl3, limb_arm_speed);
                if (curr_pos == limb_arm_lvl3)
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { lift_moving = true; }

                break;

        }

        if( lift_moving )
        {
            MoveSlide(limb_arm4, Axis.x, (curr_pos - limb_arm_start) * 3f / 4f + limb_arm_start, limb_arm_speed);
            MoveSlide(limb_arm3, Axis.x, (curr_pos - limb_arm_start) * 2f / 4f + limb_arm_start, limb_arm_speed);
            MoveSlide(limb_arm2, Axis.x, (curr_pos - limb_arm_start) * 1f / 4f + limb_arm_start, limb_arm_speed);
        }

        // Play sounds
        if (audio_manager && lift_moving && !audio_manager.IsSoundStarted("Lift"))
        {
            audio_manager.Play("Lift", 0.3f);
        }
        if (audio_manager && !lift_moving && audio_manager.IsSoundStarted("Lift"))
        {
            audio_manager.Stop("Lift", 0.3f);
        }

        // Play sounds
        if (audio_manager && fingers_moving && !audio_manager.IsSoundStarted("Fingers"))
        {
            audio_manager.Play("Fingers", 0.3f);
        }
        if (audio_manager && !fingers_moving && audio_manager.IsSoundStarted("Fingers"))
        {
            audio_manager.Stop("Fingers", 0.3f);
        }

    }

 

}