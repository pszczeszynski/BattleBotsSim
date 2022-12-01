
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for MiniDrone
// *******************************



public class Robot_Thunderstamps : RobotInterface3D
{

    private bool init_done = false;

    private bool sound_shooting = false;
    private bool playing_intake = false;
    private bool playing_jump = false;

    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;

    // **************************************
    // Intake stuff
    // 

    // Intake motors
    public float intake_speed = -4000f;  // Collector intake rotation speed
    public HingeJoint intake_motor1;
    public Transform intake_wheels_1a;
    public Transform intake_wheels_1b;

    //State Machine
    public enum intakeStates : int
    {
        off=0,
        running,        // intaking 
        reversing,      // Reversing
    }

    public intakeStates intake_statemachine = intakeStates.off;


    // **************************************8
    // Indicator
    public GameObject indicator;
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;
    public MeshRenderer lightbulb;

    // *****************************************
    // Shooter
    public ballshooting_v2 ballforcer_entrance;

    public ballshooting_v2[] ballforceroutlist;
    public ballshooting_v2 ballforcer_out;
    public ballshooting_v2 ballforcer_intake;


    public float ball_speed_max = 9f;
    public float ball_speed_min = 8f;

    // Aimer
    public HingeJoint ball_aimer;
    public float aim_target_pos = 0f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 10f;
    public float aim_tilt_speed = -20f;    // aimer tilting speed

    // ****************************
    // Jumper
    public ConfigurableJoint jumper;
    public float jump_y = 0.5f;
    public float jump_speed = 1f;
    public ConfigurableJoint armlock1;
    public HandTracker armlock1_detect;

    public ConfigurableJoint armlock2;
    public HandTracker armlock2_detect;

    public enum jumperStates : int
    {
        off = 0,
        jumping,
        extended,
        resetting,      // Reversing
    }

    public jumperStates jumper_statemachine = jumperStates.off;



    public void Awake()
    {
        info =
            "D-Pad Up/Down: Aim Up/Down\n" +
            "Button A: Reverse intake" +
            "Button X: Toggle intake\n" +
            "Button Y: Shoot\n" + 
            "Button B: Toggle Jump / reset jumper\n" + 
            info;
        
    }

    public override void RobotFixedUpdate()
    {
        if (!rb_body || !myRobotID || !init_done || isKinematic)
        { return; }


        // Update indicator
        if (indicator)
        {
            // Do indicator rotation   
            // Rotate this by the Body's rotation 
            indicator.transform.localRotation = rb_body.transform.localRotation * indicator_rotation;
            indicator.transform.localPosition = rb_body.transform.localPosition + rb_body.transform.localRotation * indicator_starting_pos;
        }
    }


    public override void Start()
    {
        base.Start();
        indicator_starting_pos = indicator.transform.localPosition;

        // Intialize hinge
        JointSpring hingespring = ball_aimer.spring;
        hingespring.targetPosition = outhinge_min;
        ball_aimer.spring = hingespring;
        ballforcer_out.speed = ball_speed_max;
    }

    private bool activelly_shooting = false;

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

        // If this is a client side, only do animations
        DoAnimations(true); // All animations need to be applied in single player mode as well for this bot

        if (isKinematic)
        {
            return;
        }

        // ****************************
        // y button: shoot balls out
        if (gamepad1_y)
        {
            ballforcer_entrance.hard_stop = false;
            activelly_shooting = true;

            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f, -1); }
                sound_shooting = true;
            }

            foreach (ballshooting_v2 curr_forcer in ballforceroutlist)
            {
                curr_forcer.disable = false;
            }
        }
        else
        {
            // Disable scripts not required
            foreach (ballshooting_v2 curr_forcer in ballforceroutlist)
            {
                curr_forcer.disable = true;
            }


            ballforcer_entrance.hard_stop = true;
            activelly_shooting = false;

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }
        }

        // ****************************
        // a button: reverse intake wheels - but only while it is pressed
        if (gamepad1_a)
        {
            ballforcer_intake.reverse = true;

            if ( intake_statemachine == intakeStates.running)
            {
                intake_statemachine = intakeStates.reversing;
            }
        }
        else 
        {
            ballforcer_intake.reverse = false;
            if (intake_statemachine == intakeStates.reversing)
            {
                intake_statemachine = intakeStates.running;
            }
        }

        // Button x toggles left intake, bottle B toggle right intake
        if (gamepad1_x && gamepad1_x_changed)
        {
            if( intake_statemachine == intakeStates.off )
            {
                intake_statemachine = intakeStates.running;
            }
            else
            {
                intake_statemachine = intakeStates.off;
            }
        }


        bool play_intake = DoIntakeStateMachine(ref intake_statemachine,  intake_motor1);

        if( play_intake && !playing_intake)
        { 
            if (audio_manager) { audio_manager.Play("collector", 0.3f, -1); }
            playing_intake = true;
        }
        if (!play_intake && playing_intake)
        {
            if (audio_manager) { audio_manager.Stop("collector", 0.3f); }
            playing_intake = false;
        }

 

        // ****************************
        // d_pad_up/down moves aimer up/down
        bool movedAim = false;
        float curr_hinge_pos = 0f;

        if (gamepad1_dpad_down)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, outhinge_min, aim_tilt_speed * turning_scaler);
            ballforcer_out.speed = (1f- (curr_hinge_pos - outhinge_min) / (outhinge_max - outhinge_min)) * (ball_speed_max - ball_speed_min) + ball_speed_min;
        }
        if (gamepad1_dpad_up)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, outhinge_max, aim_tilt_speed * turning_scaler);
            ballforcer_out.speed = (1f-(curr_hinge_pos - outhinge_min) / (outhinge_max - outhinge_min)) * (ball_speed_max - ball_speed_min) + ball_speed_min;

        }

        if (movedAim && (curr_hinge_pos != outhinge_min) && (curr_hinge_pos != outhinge_max) && (curr_hinge_pos != aim_target_pos))
        {

            if (audio_manager && !audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Play("adjustangle", 0);
            }
        }
        else
        {
            if (audio_manager)
            {
                audio_manager.Stop("adjustangle", 0.3f);
            }
        }

        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
            //float
            float rotation = ball_aimer.spring.targetPosition;

            rotation = (rotation - outhinge_min) / (outhinge_max - outhinge_min) * 180f - 90f;

            // Now rotation is to rotate around the x-asis before the robot was moved.
            indicator_rotation.eulerAngles = new Vector3(-1f * rotation, 0, 0);

            RobotFixedUpdate();
        }

        // 
        // ****************************
        // Do jumping
        if (gamepad1_b && gamepad1_b_changed )
        {
            if (jumper_statemachine == jumperStates.off)
            { jumper_statemachine = jumperStates.jumping; }

            if( jumper_statemachine == jumperStates.extended)
            { jumper_statemachine = jumperStates.resetting;  }


        }

        bool play_jump = DoJumperStateMachine(ref jumper_statemachine, jumper);

        if (play_jump && !playing_jump)
        {
            if (audio_manager) { audio_manager.Play("jump", 0.3f); }
            playing_jump = true;
        }
        if (!play_jump && playing_jump)
        {
            if (audio_manager) { audio_manager.Stop("jump", 0.3f); }
            playing_jump = false;
        }
    }



    // Intake state machine
    // Returns true if the intake is activelly moving
    public bool DoIntakeStateMachine(ref intakeStates statemachine, HingeJoint intake_motor_1)
    {
        bool intake_moving = false;

        switch (statemachine)
        {
            case intakeStates.running:
                SetHingeSpeed(intake_motor_1, intake_speed);
                ballforcer_intake.disable = false;
                intake_moving = true;
                break;

            case intakeStates.reversing:
                SetHingeSpeed(intake_motor_1, -0.5f * intake_speed);
                ballforcer_intake.disable = false;
                intake_moving = true;
                break;

            case intakeStates.off:
                SetHingeSpeed(intake_motor_1, 0);
                ballforcer_intake.disable= true;
                break;



            default:
                
                SetHingeSpeed(intake_motor_1, 0);
                break;
        }

        return intake_moving;
    }



    // Intake state machine
    // Returns true if the intake is activelly moving
    public bool DoJumperStateMachine(ref jumperStates statemachine, ConfigurableJoint jumper_joint)
    {
        bool jumping = false;

        switch (statemachine)
        {
            case jumperStates.jumping:
                if (MoveSlide(jumper_joint, Axis.y, jump_y, jump_speed) == jump_y)
                {
                    statemachine = jumperStates.extended;
                };
                jumping = true;
                break;

            case jumperStates.extended:
                if( armlock1_detect.IsFreshHold())
                {
                    armlock1.targetPosition = new Vector3(0.25f, 0, 0);
                }

                if (armlock2_detect.IsFreshHold())
                {
                    armlock2.targetPosition = new Vector3(0.25f, 0, 0);
                }

                break;


            case jumperStates.resetting:
                if (MoveSlide(jumper_joint, Axis.y, 0, jump_speed / 10f) == 0)
                {
                    statemachine = jumperStates.off;
                };

                armlock1.targetPosition = new Vector3(0, 0, 0);
                armlock2.targetPosition = new Vector3(0, 0, 0);
                armlock1_detect.ClearHold();
                armlock2_detect.ClearHold();
       
                break;

            case jumperStates.off:
                // Do Nothing
                break;

            default:
                break;
        }

        return jumping;
    }


    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
        if (!client_mode) { return; }

        if (intake_statemachine == intakeStates.running)
        {

            float speed = intake_speed * Time.deltaTime;
            if (intake_wheels_1a) { intake_wheels_1a.Rotate(-1f * speed, 0, 0, Space.Self); }
            if (intake_wheels_1b) { intake_wheels_1b.Rotate(-1f * speed, 0, 0, Space.Self); }
        }

        if (intake_statemachine == intakeStates.reversing)
        {
            float speed = intake_speed * Time.deltaTime;
            if (intake_wheels_1a)
            {
                intake_wheels_1a.Rotate(speed, 0, 0, Space.Self);
            }
            if (intake_wheels_1b)
            {
                intake_wheels_1b.Rotate(speed, 0, 0, Space.Self);
            }
        }

        lightbulb.enabled = intake_statemachine == intakeStates.running;
    }

    // ********* State Sync for server/client
    // Make sure the string is a small, mainly numerical value (e.g. 1:2:0.11...) so that it doesn't increase packet length much, and won't restrict compression.
    override public string GetStates()
    {
        // Save states that we want to synchronize over internet
        string outstring = base.GetStates();

        // Add our intake state machine to it
        outstring += ";"; // different from base seperator

        outstring += (int)intake_statemachine;

        // Nothing else to add now
        return outstring;
    }

    // Read in the states that were turned into a string representation from GetStates()
    override public void SetStates(string instring)
    {
        string[] allstates = instring.Split(';');

        base.SetStates(allstates[0]);

        // Do holding position
        if (allstates.Length > 1)
        {
            intake_statemachine = (intakeStates)int.Parse(allstates[1]);
        }

    }
}



