using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_AMOGO_v1 : RobotInterface3D {

    // Intake Collector
    public HingeJoint intake_collector;
    public HingeJoint intake_collector2;
    public float intake_speed = -1000f;  // Collector intake rotation speed

    // States for running intake off/on
    // States for running intake off/on
    enum intakeStates : int
    {
        off = 0,
        onNormal,
        reverse
    }
    intakeStates intake_statemachine = intakeStates.off;
    intakeStates old_intake_statemachine = intakeStates.off;

    public ballshooting ramp_force_box;
    public float ramp_speed = 1.5f;

    // Other variables for internal use
    bool a_button_last = false;

    
    // **** LIFT Stuff
    public ConfigurableJoint lift_arm;
    public ConfigurableJoint lift_inner;
    public ConfigurableJoint lift_middle;

    public float lift_speed = 1f;
    public float lift_max = -2f;
    public float lift_min = 0f;


    bool x_button_last = false;

    // *** GoalPivot
    public HingeJoint goal_pivot;
    public float goalp_max2 = 0;
    public float goalp_min2 = 45; // Referenced to right, negative for left
    public float goalp_speed = 200f;
    public float goalp_dump = 30f;


    // *** Platform Arm helper
    public HingeJoint platform_arms;
    public float platarms_min = 0;
    public float platarms_max = 45f;
    public float platarms_speed = 100f;

    public float lt = 0f;
    public float rt = 0f;

    private bool arm_played = false;


    // Other private variables
    private bool movedLift_sound = false;
    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;
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
        // Remember some variables that we scale later during init
        if (!init_done)
        {

            // Initialize local cache variables
            sound_controller = GetComponentInChildren<Sound_Controller_FRC_Shooter>();
            audio_manager = GetComponentInChildren<AudioManager>(); // Only 1 audio manager in scene.

            init_done = true;
        }

        // A button: turns intake wheels off/on
        if (gamepad1_a && !a_button_last)
        {
            //switch the collecting state to off if we are on, to on if we are off
            if (intake_statemachine == intakeStates.onNormal)
            {
                intake_statemachine = intakeStates.off;
            }
            else
            {
                intake_statemachine = intakeStates.onNormal;

            }
        }

        a_button_last = gamepad1_a;

        // ****************************
        // Y button: reverse intake wheels - but only while it is pressed
        if (gamepad1_y)
        {
            intake_statemachine = intakeStates.reverse;
        }
        else if (intake_statemachine == intakeStates.reverse)
        {
            intake_statemachine = intakeStates.onNormal;
        }

        // ****************************
        // Intake Wheel State Machine
        // ****************************

        // Do sound for intake state machine
        if (intake_statemachine != old_intake_statemachine)
        {
            // If intake got turned off, play sound
            if (intake_statemachine == intakeStates.off)
            {
                if (sound_controller) { sound_controller.revdown(); }
            }
            // If state machine got turned on, play sound
            else if (old_intake_statemachine == intakeStates.off)
            {
                if (sound_controller) { sound_controller.revup(); }
            }
        }

        old_intake_statemachine = intake_statemachine;

        if (intake_statemachine == intakeStates.onNormal)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = intake_speed;
            intake_collector.motor = outmotor;

            JointMotor outmotor2 = intake_collector2.motor;
            outmotor2.targetVelocity = -4f*intake_speed;
            intake_collector2.motor = outmotor2;
            ramp_force_box.speed = ramp_speed;


        }
        if (intake_statemachine == intakeStates.off)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = 0;
            intake_collector.motor = outmotor;

            JointMotor outmotor2 = intake_collector2.motor;
            outmotor2.targetVelocity = 0;
            intake_collector2.motor = outmotor2;

            ramp_force_box.speed = 0f;

        }
        if (intake_statemachine == intakeStates.reverse)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = -1f * intake_speed;
            intake_collector.motor = outmotor;

            JointMotor outmotor2 = intake_collector2.motor;
            outmotor2.targetVelocity = -4f * intake_speed;
            intake_collector2.motor = outmotor2;
            ramp_force_box.speed = -1f * ramp_speed;
        }


        // **************************************
        // Lift Movement

        bool movedLift = false;
        // Move lift up
        if (gamepad1_dpad_up)
        {
            movedLift = true;
            float targetpos = MoveTowards(lift_min, lift_max, lift_arm.targetPosition.x, Time.deltaTime * lift_speed * turning_scaler);
            lift_arm.targetPosition = new Vector3(targetpos, 0, 0);
            lift_inner.targetPosition = new Vector3(targetpos * 2f / 3f, 0, 0);
            lift_middle.targetPosition = new Vector3(targetpos * 1f / 3f, 0, 0);

        }

        // Move lift down
        if (gamepad1_dpad_down)
        {
            movedLift = true;
            float targetpos = MoveTowards(lift_max, lift_min, lift_arm.targetPosition.x, Time.deltaTime * lift_speed * turning_scaler);
            lift_arm.targetPosition = new Vector3(targetpos, 0, 0);
            lift_inner.targetPosition = new Vector3(targetpos * 2f / 3f, 0, 0);
            lift_middle.targetPosition = new Vector3(targetpos * 1f / 3f, 0, 0);

        }
        if (movedLift && (lift_arm.targetPosition.x != lift_max) && (lift_arm.targetPosition.x != lift_min))
        {
            if (!movedLift_sound)
            {
                if (audio_manager && !audio_manager.IsSoundStarted("hangerraise")) { audio_manager.Play("hangerraise", 0); }
                movedLift_sound = true;
            }
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("hangerraise")) { audio_manager.Stop("hangerraise", 0.3f); }
            movedLift_sound = false;
        }

        bool movedArm = false;
        // Move lift up

        if (gamepad1_dpad_left && goal_pivot.spring.targetPosition != goalp_max2)
        {
            MoveHinge(goal_pivot, goalp_max2, goalp_speed * turning_scaler);
            movedArm = true;
        }
        if (gamepad1_dpad_right && goal_pivot.spring.targetPosition != goalp_min2)
        {
            MoveHinge(goal_pivot, goalp_min2, goalp_speed * turning_scaler);
            movedArm = true;
        }


        if (gamepad1_x && goal_pivot.spring.targetPosition != goalp_dump)
        {
            MoveHinge(goal_pivot, goalp_dump, goalp_speed * turning_scaler);
            movedArm = true;
        }
        else if( goal_pivot.spring.targetPosition > goalp_max2 ) 
        {
            MoveHinge(goal_pivot, goalp_max2, goalp_speed * turning_scaler);
            movedArm = true;
        }

        if (movedArm)
        {
            if (audio_manager && !audio_manager.IsSoundStarted("arm")) 
            { 
                audio_manager.Play("arm", 0);
                arm_played = true;
            }

        }
        else if(arm_played)
        {
            if (audio_manager && audio_manager.IsSoundStarted("arm")) 
            { 
                audio_manager.Stop("arm", 0.3f);
                
            }
            arm_played = false;
        }

        x_button_last = gamepad1_x;


        bool moved_plat = false;
        float curr_hinge_pos = 0f;

        // Move platform arms up/down
        if ( gamepad1_left_trigger > 0f)
        {
            moved_plat = true;
            curr_hinge_pos = MoveHinge(platform_arms, platarms_min, platarms_speed * gamepad1_left_trigger);
        }

        if (gamepad1_right_trigger > 0f)
        {
            moved_plat = true;
            curr_hinge_pos = MoveHinge(platform_arms, platarms_max, platarms_speed * gamepad1_right_trigger);
        }

        lt = gamepad1_left_trigger;
        rt = gamepad1_right_trigger;



        if (moved_plat && (curr_hinge_pos != platarms_min) && (curr_hinge_pos != platarms_max) )
        {

            if (audio_manager && !audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Play("adjustangle", 0);
            }
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("adjustangle") )
            {
                audio_manager.Stop("adjustangle", 0.3f);
            }
        }






    }


}