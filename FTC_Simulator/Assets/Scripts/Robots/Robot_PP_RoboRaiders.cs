using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Robot_Vulcan
// *******************************
//

public class Robot_PP_RoboRaiders : RobotInterface3D {

    public ConfigurableJoint limb_arm5; // Actual hand
    public ConfigurableJoint limb_arm4; // Main lifting arm: 4th stage
    public ConfigurableJoint limb_arm3; // dummy arm: 3rd stage
    public ConfigurableJoint limb_arm2; // dummy arm: 2nd stage
    public float limb_arm_start = 0;
    public float limb_arm_stop = 1f;
    public float limb_arm_speed = 100;
    public float limb_arm_lvl1 = -0.75f;
    public float limb_arm_lvl2 = -1.25f;
    public float limb_arm_lvl3 = -1.75f;

    public HingeJoint finger_l; // fingers
    public HingeJoint finger_r; // fingers
    public float finger_speed = 100;
    public float finger_start_pos = 0;
    public float finger_end_pos = -0.2f;

    public HingeJoint hand;
    public float hand_start = 0;
    public float hand_stop = -180f;
    public float hand_speed = 180f;
    public float hand_lift_threhsold =-0.45f;

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
            "D-Pad Up/Down: Rotate claw\n" + 
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
        bool hand_moving = false;
        bool rotate_hand_start = false;
        bool rotate_hand_stop = false;

        // ****************************
        // While A is pressed open the hand
        if (gamepad1_a )
        {
            // Move fingers
            MoveHinge(finger_l, finger_start_pos, finger_speed);
            MoveHinge(finger_r, finger_start_pos, finger_speed);

            // If they moved, play sound
            if (finger_l.spring.targetPosition != finger_start_pos)
            {
                fingers_moving = true;
            }
        }
        //when released, run state machine
        if( gamepad1_a_changed && !gamepad1_a)
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

        float curr_pos = 0f;

        // ****************************
        // d_pad_up/down - rotate hand
        if (gamepad1_dpad_down) 
        {
            curr_pos = MoveHinge(hand, hand_stop, turning_scaler * hand_speed);

            if (curr_pos != hand_stop)
            {
                hand_moving = true;
            }
        }
        if (gamepad1_dpad_up) 
        {
            curr_pos =  MoveHinge(hand, hand_start, turning_scaler * hand_speed);

            if (curr_pos != hand_start)
            {
                hand_moving = true;
            }
        }



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
            MoveHinge(finger_l, finger_end_pos, turning_scaler * finger_speed);
            MoveHinge(finger_r,  finger_end_pos, turning_scaler * finger_speed);

            if (finger_l.spring.targetPosition != finger_end_pos)
            {
                fingers_moving = true;
            }

        }
        if (gamepad1_dpad_left)
        {
            MoveHinge(finger_l, finger_start_pos, turning_scaler * finger_speed);
            MoveHinge(finger_r, finger_start_pos, turning_scaler * finger_speed);

            if (finger_l.spring.targetPosition != finger_start_pos)
            {
                fingers_moving = true;
            }
        }



        switch( lift_state)
        {
            case LiftStatemachine.lvl_ground:
                // First open fingers, than rotate arm
                if( finger_l.spring.targetPosition == finger_start_pos)
                {
                    rotate_hand_start = true;
                }
                // Make sure hand is rotated back before moving lift
                curr_pos = limb_arm5.targetPosition.x;
                // if ( !((curr_pos > hand_lift_threhsold) && (hand.spring.targetPosition != hand_start)))
                if (hand.spring.targetPosition == hand_start)
                {
                    curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_start, limb_arm_speed);
                }
                
                // Finally check if its finished
                if (curr_pos  == limb_arm_start)
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { 
                    lift_moving = true;
                    fingers_moving = true;
                }

                MoveHinge(finger_l, finger_start_pos, finger_speed);
                MoveHinge(finger_r, finger_start_pos,  finger_speed);
                
                break;

            case LiftStatemachine.lvl_low:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_lvl1, limb_arm_speed);
                rotate_hand_stop = true;
                if ((curr_pos == limb_arm_lvl1) && (hand.spring.targetPosition == hand_stop))
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { lift_moving = true; }

                break;

            case LiftStatemachine.lvl_mid:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_lvl2, limb_arm_speed);
                rotate_hand_stop = true;
                if ((curr_pos == limb_arm_lvl2) && (hand.spring.targetPosition == hand_stop))
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { lift_moving = true; }

                break;

            case LiftStatemachine.lvl_high:
                curr_pos = MoveSlide(limb_arm5, Axis.x, limb_arm_lvl3, limb_arm_speed);
                rotate_hand_stop = true;
                if ((curr_pos == limb_arm_lvl3) && (hand.spring.targetPosition == hand_stop))
                {
                    lift_state = LiftStatemachine.manual;
                }
                else { lift_moving = true; }

                break;

        }

        if( rotate_hand_start)
        {
            if( MoveHinge(hand, hand_start, hand_speed) != hand_start)
            {
                hand_moving = true;
            }

        }

        if( rotate_hand_stop)
        {
            // Curr_pos has the current lift level
            if (curr_pos < hand_lift_threhsold)
            {
                if (MoveHinge(hand, hand_stop, hand_speed) != hand_stop)
                {
                    hand_moving = true;
                }
            }
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

        // Play sounds
        if (audio_manager && hand_moving && !audio_manager.IsSoundStarted("Hand"))
        {
            audio_manager.Play("Hand", 0.3f);
        }
        if (audio_manager && !hand_moving && audio_manager.IsSoundStarted("Hand"))
        {
            audio_manager.Stop("Hand", 0.3f);
        }

    }

 

}