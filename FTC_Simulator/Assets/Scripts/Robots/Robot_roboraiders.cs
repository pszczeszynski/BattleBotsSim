using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Robot_Roboraiders
// *******************************
//



public class Robot_roboraiders : RobotInterface3D {

    public ConfigurableJoint collector_l;
    public ConfigurableJoint collector_r;

    public HingeJoint limb_arm; // z-axis rotation of arm  (extend arm). Angle is negative of this value
    public float limb_arm_angle_start = 5;
    public float limb_arm_speed = 100;

    public HingeJoint limb_forearm;    // z-axis rotation of forearm (extend forearm).  
    public float limb_forearm_angle_start = 4;
    public float limb_forearm_speed = 100;

    public HingeJoint limb_wrist;    // z-axis rotaton of hand (tilt of hand)
    public float limb_wrist_angle_start = 0;
    public float limb_wrist_speed = 100;

    public HingeJoint limb_hand;     // y-axis rotation of hand (flip hand around to face outward). Angle is negative of this value
    public float limb_hand_angle_start = 0;
    public float limb_hand_speed = 100;

    public HingeJoint limb_fingers;  // z-axis rotation of fingers (grasp block)
    public float limb_fingers_angle_start = 100;
    public float limb_fingers_speed = 100;

    public float wheelspeed = 20;
    public float reversspeed = 40;
    //bool gripped = false;
    //bool grip_button_last = false;
    //float lastResetTime = 0;
    bool a_button_last = false;
    bool b_button_last = false;
    JointSpring g1;

    enum collectingStates : int
    {
        off = 0,
        onNormal,
        reverse
    }
    collectingStates collecting_state = collectingStates.off;
    

    enum armStates : int
    {
        start = 0,    // All joints in starting position
        gripping,     // attempting to grip block
        starting_up,  // lift block up out of cradle
        swiveling,    // swivel block to ready poisition
        settling,     // preparing block to correct level
        ready,        // Ready to deliver block
        ungrip,       // ungrips block
        clear         // Clears the blocks before going to start  
    }
    armStates arm_state = armStates.start;
    public int armstate = 0;
    float forearm_angle_saved = 0;
    float arm_angle_saved = 0;
    float wrist_angle_saved = 0;
    // Initialize joints to minimum state

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }
    public override void Init_Robot()
    {
        ResetJoints();
    }

    // Reset all joints to starting position
    void ResetJoints()
    {
        g1 = limb_arm.spring;
        g1.targetPosition = limb_arm_angle_start * -1.0f;
        limb_arm.spring = g1;

        g1 = limb_forearm.spring;
        g1.targetPosition = limb_forearm_angle_start;
        limb_forearm.spring = g1;

        g1 = limb_wrist.spring;
        g1.targetPosition = limb_wrist_angle_start;
        limb_wrist.spring = g1;

        g1 = limb_hand.spring;
        g1.targetPosition = limb_hand_angle_start * -1.0f;
        limb_hand.spring = g1;

        g1 = limb_fingers.spring;
        g1.targetPosition = limb_fingers_angle_start;
        limb_fingers.spring = g1;
    }


    // Update Call
    public override void Update_Robot()
    {
        // ****************************
        // Y button: reverse intake wheels
        if (gamepad1_y)
        {
            collecting_state = collectingStates.reverse;
        }

        // ****************************
        // A button: turns intake wheels off/on
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

        // ****************************
        // B button: Returns arm to starting position
        if (gamepad1_b && !b_button_last)
        {
            // First take a snapshot of main arm positions so that we can clear the bricks clean
            forearm_angle_saved = limb_forearm.spring.targetPosition;
            arm_angle_saved = limb_arm.spring.targetPosition;
            wrist_angle_saved = limb_wrist.spring.targetPosition;
            arm_state = armStates.ungrip;
        }

        // ****************************
        // X button: Starts the block delivery state machine
        if (gamepad1_x && arm_state == armStates.start)
        {
            arm_state = armStates.gripping;
        }

        b_button_last = gamepad1_b;
        
        // ****************************
        // d_pad_up/down move forearm up/down
        if (gamepad1_dpad_down && arm_state == armStates.ready)
        {
            MyMoveHinge(limb_forearm, 0, limb_forearm_speed/2f);
        }
        if (gamepad1_dpad_up && arm_state == armStates.ready)
        {
            MyMoveHinge(limb_forearm, 180f, limb_forearm_speed / 2f);
        }

        // ****************************
        // d_pad_left/right move arm forward/backwards
        if (gamepad1_dpad_left && arm_state == armStates.ready)
        {
            MyMoveHinge(limb_arm, limb_arm_angle_start * -1.0f, limb_arm_speed / 2f);
        }
        if (gamepad1_dpad_right && arm_state == armStates.ready)
        {
            MyMoveHinge(limb_arm, -180f, limb_arm_speed / 2f);
        }


        // ****************************
        // left_bumper, right_bumper rotate block 
         if (gamepad1_left_bumper && arm_state == armStates.ready)
         {
             MyMoveHinge(limb_wrist, -180, limb_wrist_speed / 2f);
         }
         if (gamepad1_right_bumper && arm_state == armStates.ready)
         {
            MyMoveHinge(limb_wrist, 180, limb_wrist_speed / 2f);
        }
         

        // ****************************
        // Intake Wheel State Machine
        if (collecting_state == collectingStates.onNormal)
        {
            collector_l.targetAngularVelocity = new Vector3(wheelspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(-1f * wheelspeed, 0, 0);

        }
        if (collecting_state == collectingStates.off)
        {
            collector_l.targetAngularVelocity = new Vector3(0, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(0, 0, 0);
        }
        if (collecting_state == collectingStates.reverse)
        {
            collector_l.targetAngularVelocity = new Vector3(-1f * reversspeed, 0, 0);
            collector_r.targetAngularVelocity = new Vector3(reversspeed, 0, 0);
        }

        // ****************************
        // Arm State Machine
        if (arm_state == armStates.start)
        {
            // Move all joints to home position
            MyMoveHinge(limb_arm, limb_arm_angle_start * -1.0f, limb_arm_speed);
            MyMoveHinge(limb_forearm, limb_forearm_angle_start, limb_forearm_speed);
            MyMoveHinge(limb_wrist, limb_wrist_angle_start, limb_wrist_speed);
            MyMoveHinge(limb_hand, -1.0f * limb_hand_angle_start, limb_hand_speed);
            MyMoveHinge(limb_fingers, limb_fingers_angle_start, limb_fingers_speed);
        }
        if (arm_state == armStates.gripping)
        {
            // Move all joints to home position
            if ( MyMoveHinge(limb_fingers, -60, limb_fingers_speed) )
            {
                arm_state = armStates.starting_up;
            }
        }
        if (arm_state == armStates.starting_up)
        {
            bool alldone = true;
            // Lift arm up
            if( !MyMoveHinge(limb_arm, -15f, 0.5f*limb_arm_speed) ) { alldone = false; }
            if( !MyMoveHinge(limb_forearm, 30f, limb_forearm_speed)) { alldone = false; }
            if( !MyMoveHinge(limb_wrist, 30f, limb_wrist_speed)) { alldone = false; }

            if ( alldone )
            { arm_state = armStates.swiveling;  }
        }
        if (arm_state == armStates.swiveling)
        {
            bool alldone = true;
            // Lift arm up
            if (!MyMoveHinge(limb_arm, -40f, limb_arm_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_forearm, 60f, limb_forearm_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_wrist, -4f, limb_wrist_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_hand, -180f, limb_hand_speed)) { alldone = false; }

            if (alldone)
            { arm_state = armStates.settling; }
        }
        if (arm_state == armStates.settling)
        {
            bool alldone = true;
            // Lift arm up
            if (!MyMoveHinge(limb_arm, -20f, limb_arm_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_forearm, 30f, limb_forearm_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_wrist, -4f, limb_wrist_speed)) { alldone = false; }

            if (alldone)
            { arm_state = armStates.ready; }
        }
        if (arm_state == armStates.ungrip)
        {
            bool alldone = true;

            // Clear the brick
            if (!MyMoveHinge(limb_fingers, limb_fingers_angle_start, limb_fingers_speed)) { alldone = false; }

            if (alldone)
            { arm_state = armStates.clear; }
        }
        if (arm_state == armStates.clear)
        {
            bool alldone = true;
            // Clear the brick
            // if (!MoveHinge(limb_forearm, forearm_angle_saved + 30f, limb_forearm_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_arm, arm_angle_saved + 30f, limb_forearm_speed)) { alldone = false; }
            if (!MyMoveHinge(limb_wrist, limb_wrist_angle_start, limb_wrist_speed)) { alldone = false; }

            if (alldone)
            { arm_state = armStates.start; }
        }
        armstate = (int) arm_state;

        
    }

    // Moves joint by a small delta. If reached destination, return true
    bool MyMoveHinge( HingeJoint myhinge, float target, float speed)
    {
        return MoveHinge(myhinge, target, speed) == target;
    }
}