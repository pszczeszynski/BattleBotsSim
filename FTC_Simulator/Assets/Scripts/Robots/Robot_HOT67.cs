
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for MiniDrone
// *******************************



public class Robot_HOT67 : RobotInterface3D
{

    private bool init_done = false;

    private bool sound_shooting = false;
    private bool playing_arm = false;
    private bool playing_intake = false;

    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;

    // **************************************
    // Intake stuff
    // 

    // Intake motors
    public float intake_speed = -4000f;  // Collector intake rotation speed
    public HingeJoint intake_motor_left1;
    public HingeJoint intake_motor_left2;
    public HingeJoint intake_motor_left3;
    public HingeJoint intake_motor_left4;
    public HingeJoint intake_motor_right1;
    public HingeJoint intake_motor_right2;
    public HingeJoint intake_motor_right3;
    public HingeJoint intake_motor_right4;
    public Transform intake_wheels_1a;
    public Transform intake_wheels_1b;
    public Transform intake_wheels_2a;
    public Transform intake_wheels_2b;

    // Intake Joints
    public HingeJoint intake_axel_left1;
    public HingeJoint intake_axel_left2;
    public HingeJoint intake_axel_right1;
    public HingeJoint intake_axel_right2;

    public float intake_axel_speed = 200f;
    public float intake_axel1_home = -60f;
    public float intake_axel2_home = -30f;

    //State Machine
    public enum intakeStates : int
    {
        retracting = 0, // Moving towards the retracted position, rollers off?
        retracted,      // Retracted and all rollers off
        extending,      // extending, rollers on?
        running,        // intaking 
        reversing,      // Reversing
        off            // off
    }

    public intakeStates intakeL_statemachine = intakeStates.retracting;
    public intakeStates intakeR_statemachine = intakeStates.retracting;



    // **************************************8
    // Indicator
    public GameObject indicator;
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;


    // *****************************************
    // Shooter
    public ballshooting_v2 ballforcer_left;
    public ballshooting_v2 ballforcer_right;
    public ballshooting_v2 ballforcer_entrance;
    public float ballforcer_in_speed = 8f;
    public float ballforcer_in_divider = 30f;
    public ballshooting_v2 ballforcer_out;
    public float ball_speed_max = 9f;
    public float ball_speed_min = 8f;

    // Aimer
    public HingeJoint ball_aimer;
    public float aim_target_pos = 0f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 10f;
    public float aim_tilt_speed = -20f;    // aimer tilting speed

    // *****************************************
    // Climber
    public HingeJoint climber_deploy;
    public HingeJoint climber_beam;
    public HingeJoint climber_hands1;
    public HingeJoint climber_hands2;

    public float climber_deploy_angle = -90f;
    public float climber_deploy_speed = 50f;
    public float climb_beam_extended = -32f;
    public float climb_hands_extended = 0f;
    public float climb_beam_speed = 90f;
    public float climb_hands_speed = 90f;

    public HandTracker hand1tracker;
    public HandLockBar hand1lockbar;

    public HandTracker hand2tracker;
    public HandLockBar hand2lockbar;

    public float[,] hand_map = new float[5, 2]
    {
        {-180f, 0f },
        {-90f,  0f },
        { 0f,   0f },
        { 90f,  0f },
        { 180f, 0f}
    };


    // eg
    // MoveHinge(bucket, MyUtils.GetInterpolatedValue.(arm.spring.targetPosition, arm_bucket_map), 10f * myarm_speed);

    public enum climberStates : int
    {
        parking = 0,
        parked,
        extending,
        extended
    }

    public climberStates climber_statemachine = climberStates.parked;

    public void Awake()
    {
        info =
            "D-Pad Up/Down: Aim Up/Down\n" +
            "D-Pad Left/Right: Climber extend/retract\n" +
            "Left/Right Trigger: Rotate climber\n" +
            "Button A: Reverse intake" +
            "Button B: Toggle right intake\n" +
            "Button X: Toggle left intake\n" +
            "Button Y: Shoot\n" +        
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
        // a button: reverse intake wheels - but only while it is pressed
        if (gamepad1_a)
        {
            if( intakeL_statemachine == intakeStates.running)
            {
                intakeL_statemachine = intakeStates.reversing;
            }
            if (intakeR_statemachine == intakeStates.running)
            {
                intakeR_statemachine = intakeStates.reversing;
            }
        }
        else 
        {
            if (intakeL_statemachine == intakeStates.reversing)
            {
                intakeL_statemachine = intakeStates.running;
            }
            if (intakeR_statemachine == intakeStates.reversing)
            {
                intakeR_statemachine = intakeStates.running;
            }
        }

        // Button x toggles left intake, bottle B toggle right intake
        if( gamepad1_x && gamepad1_x_changed )
        {
            if( intakeL_statemachine != intakeStates.retracting && intakeL_statemachine != intakeStates.retracted)
            {
                intakeL_statemachine = intakeStates.retracting;
            }
            else
            {
                intakeL_statemachine = intakeStates.extending;
            }
        }

        if (gamepad1_b && gamepad1_b_changed)
        {
            if (intakeR_statemachine != intakeStates.retracting && intakeR_statemachine != intakeStates.retracted)
            {
                intakeR_statemachine = intakeStates.retracting;
            }
            else
            {
                intakeR_statemachine = intakeStates.extending;
            }
        }


        bool play_intake = DoIntakeStateMachine(ref intakeL_statemachine, intake_axel_left1, intake_axel_left2, intake_motor_left1, intake_motor_left2, intake_motor_left3, intake_motor_left4, ballforcer_left);
        play_intake = DoIntakeStateMachine(ref intakeR_statemachine, intake_axel_right1, intake_axel_right2, intake_motor_right1, intake_motor_right2, intake_motor_right3, intake_motor_right4, ballforcer_right) || play_intake;
        DoClimberStateMachine();

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
        // y button: shoot balls out
        if (gamepad1_y)
        {
            ballforcer_entrance.speed = ballforcer_in_speed;
            activelly_shooting = true;

            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f, -1); }
                sound_shooting = true;
            }
        }
        else
        {
            ballforcer_entrance.speed = -0.1f * ballforcer_in_speed;
            activelly_shooting = false;

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }
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


        // ****************************
        // d_pad_right extends the climb arm, d_pad_left retracts it
        if (gamepad1_dpad_right)
        {
            if( (climber_statemachine == climberStates.parked) ||
                (climber_statemachine == climberStates.parking) )
            {
                climber_statemachine = climberStates.extending;
            }
       }
        if (gamepad1_dpad_left)
        {
            if ( (climber_statemachine == climberStates.extended) ||
                (climber_statemachine == climberStates.extending))
            {
                climber_statemachine = climberStates.parking;
            }
        }

        bool arm_rotated = false;


        // Left trigger + right trigger rotate the main beam
        if ( (gamepad1_left_trigger > 0f) || (gamepad1_right_trigger == 0f) && (Math.Abs(climber_beam.motor.targetVelocity) > 0))
        {
            JointMotor cb_motor = climber_beam.motor;
            cb_motor.targetVelocity = -1f * gamepad1_left_trigger * climb_beam_speed;
            climber_beam.motor = cb_motor;
            arm_rotated = true;
        }


        // Left trigger + right trigger rotate the main beam
        if ((gamepad1_right_trigger > 0f) || (gamepad1_left_trigger == 0f) && (Math.Abs(climber_beam.motor.targetVelocity) > 0))
        {
            JointMotor cb_motor = climber_beam.motor;
            cb_motor.targetVelocity =  gamepad1_right_trigger * climb_beam_speed;
            climber_beam.motor = cb_motor;
            arm_rotated = true;
        }

        if (arm_rotated && !playing_arm)
        {
            if (audio_manager) { audio_manager.Play("ArmRotate", 0.3f, -1); }
            playing_arm = true;
        }

        if (!arm_rotated && playing_arm)
        {
            if (audio_manager) { audio_manager.Stop("ArmRotate", 0.3f); }
            playing_arm = false;
        }

        // No buttons left for home position
        // if (gamepad1_b) // Seek home position 
        //{
        //    movedAim = true;
        //    curr_hinge_pos = MoveHinge(ball_aimer_cj, aim_target_pos, aim_tilt_speed);
        //}


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
    }

    // Climber state machine
    bool playing_hangerraise = false;
    public void DoClimberStateMachine()
    {
        float pos1=0, pos2=0, pos3=0, pos4=0;
        bool play_sound = false;

        switch (climber_statemachine)
        {
            case climberStates.parked:
                // Nothing to do if we got here
                break;

            case climberStates.parking:
                // Unlock all hands
                hand1lockbar.Lock(false);
                hand2lockbar.Lock(false);

                // Move bar to 0 position and hands to 0 position
    
                pos1=MoveHinge(climber_beam, 0, climb_beam_speed);
                pos2=MoveHinge(climber_hands1, 0, climb_hands_speed);
                pos3=MoveHinge(climber_hands2, 0, climb_hands_speed);
                play_sound = true;

                if ( Math.Abs(pos1)<5f && Math.Abs(pos2)<5f && Math.Abs(pos3) < 5f )
                {
                    pos4 = MoveHinge(climber_deploy, 0, climber_deploy_speed);

                    if (Math.Abs(pos1) < 0.1f && Math.Abs(pos2) < 0.1f && Math.Abs(pos3) < 0.1f && Math.Abs(pos4) < 0.1f)
                    {
                        climber_statemachine = climberStates.parked;
                        play_sound = false;
                    }
                }

                
                break;

            case climberStates.extending:
                // Move bar and Hands to starting position
                pos1 = MoveHinge(climber_beam, climb_beam_extended, climb_beam_speed);
                pos4 = MoveHinge(climber_deploy, climber_deploy_angle, climber_deploy_speed);

                // pos2 = MoveHinge(climber_hands1, climb_beam_extended, climb_beam_speed);

                if (Math.Abs(climb_beam_extended-pos1) < 0.1f && Math.Abs(pos2) < 0.1f && Math.Abs(pos3) < 0.1f && Math.Abs(climber_deploy_angle - pos4) < 0.1f)
                {
                    climber_statemachine = climberStates.extended;

                    // Change main arm hinge to become a motor hinge
                    climber_beam.useMotor = true;
                    climber_beam.useSpring = false;
                    
                }

                play_sound = true;
                break;

            case climberStates.extended:
                // Check if either hand had a new lock and if the lock was succesful
                if( hand1tracker.IsFreshHold())
                {
                    hand1lockbar.Lock(true);

                    if (hand1lockbar.IsLocked())
                    {
                        hand2lockbar.Lock(false);
                        hand1tracker.ClearHold();
                    }
                }
                if (hand2tracker.IsFreshHold())
                {
                    hand2lockbar.Lock(true);
                    if (hand2lockbar.IsLocked())
                    {
                        hand1lockbar.Lock(false);
                        hand2tracker.ClearHold();
                    }
                }

                // Clear locks if bar left detection for some reason
                if( !hand1tracker.IsHolding())
                {
                    hand1lockbar.Lock(false);
                }
                if (!hand2tracker.IsHolding())
                {
                    hand2lockbar.Lock(false);
                }

                play_sound = false;
                break;
        }

        if (play_sound && !playing_hangerraise)
        {
            if (audio_manager) { audio_manager.Play("hangerraise", 0.3f, -1); }
            playing_hangerraise = true;
        }
        
        if( !play_sound && playing_hangerraise)
        {
            if (audio_manager) { audio_manager.Stop("hangerraise", 0.3f); }
            playing_hangerraise = false;
        }
    }


    // Intake state machine
    // Returns true if the intake is activelly moving
    public bool DoIntakeStateMachine(ref intakeStates statemachine, HingeJoint intake_axel_1, HingeJoint intake_axel_2, 
                                    HingeJoint intake_motor_1, HingeJoint intake_motor_2, HingeJoint intake_motor_3, HingeJoint intake_motor_4, ballshooting_v2 ballforcer)
    {
        float axel1_pos = 0f;
        float axel2_pos = 0f;
        bool intake_moving = false;

        switch (statemachine)
        {
            case intakeStates.running:
                SetHingeSpeed(intake_motor_1, intake_speed);
                SetHingeSpeed(intake_motor_2, intake_speed);
                SetHingeSpeed(intake_motor_3, intake_speed);
                SetHingeSpeed(intake_motor_4, intake_speed);

                if( ballforcer.disable)
                {
                    ballforcer.disable = false;
                    ballforcer.Clear();
                }
                
                if ( ballforcer.speed < 0 ) { 
                    ballforcer.speed *= -1f;
                    ballforcer.force_divider = ballforcer_in_divider;
                }

                intake_moving = true;
                break;

            case intakeStates.reversing:
                SetHingeSpeed(intake_motor_1, -0.5f * intake_speed);
                SetHingeSpeed(intake_motor_2, -0.5f * intake_speed);
                if (ballforcer.disable)
                {
                    ballforcer.disable = false;
                    ballforcer.Clear();
                }

                if (ballforcer.speed > 0) { 
                    ballforcer.speed *= -1f;
                    ballforcer.force_divider = ballforcer_in_divider/10f;
                }

                intake_moving = true;
                break;

            case intakeStates.off:
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);
                SetHingeSpeed(intake_motor_4, 0);
                if (activelly_shooting)
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                break;

            case intakeStates.extending:
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);
                SetHingeSpeed(intake_motor_4, 0);
                axel1_pos = MoveHinge(intake_axel_1, 0, intake_axel_speed);
                axel2_pos = MoveHinge(intake_axel_2, 0, intake_axel_speed);
                if ((axel1_pos == 0) &&
                    (axel2_pos == 0))
                {
                    statemachine = intakeStates.running;
                }

                if (activelly_shooting)
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                intake_moving = true;
                break;

            case intakeStates.retracting:
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);
                SetHingeSpeed(intake_motor_4, 0);
                axel1_pos = MoveHinge(intake_axel_1, intake_axel1_home, intake_axel_speed);
                axel2_pos = MoveHinge(intake_axel_2, intake_axel2_home, intake_axel_speed);
                if( (axel1_pos == intake_axel1_home) &&
                    (axel2_pos == intake_axel2_home))
                {
                    statemachine = intakeStates.retracted;
                }

                if (activelly_shooting)
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;                    
                }
                break;

            default:
                if (activelly_shooting )
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);
                SetHingeSpeed(intake_motor_4, 0);
                break;
        }

        return intake_moving;
    }



    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
        if (!client_mode) { return; }

        if (intakeL_statemachine == intakeStates.running)
        {

            float speed = intake_speed * Time.deltaTime;
            intake_wheels_1a.Rotate(-1f * speed, 0, 0, Space.Self);
            intake_wheels_1b.Rotate(-1f * speed, 0, 0, Space.Self);
        }

        if (intakeL_statemachine == intakeStates.reversing)
        {
            float speed = intake_speed * Time.deltaTime;
            intake_wheels_1a.Rotate(speed, 0, 0, Space.Self);
            intake_wheels_1b.Rotate(speed, 0, 0, Space.Self);
        }

        if (intakeR_statemachine == intakeStates.running)
        {

            float speed = intake_speed * Time.deltaTime;
            intake_wheels_2a.Rotate(-1f * speed, 0, 0, Space.Self);
            intake_wheels_2b.Rotate(-1f * speed, 0, 0, Space.Self);
        }

        if (intakeR_statemachine == intakeStates.reversing)
        {
            float speed = intake_speed * Time.deltaTime;
            intake_wheels_2a.Rotate(speed, 0, 0, Space.Self);
            intake_wheels_2b.Rotate(speed, 0, 0, Space.Self);
        }
    }

    // ********* State Sync for server/client
    // Make sure the string is a small, mainly numerical value (e.g. 1:2:0.11...) so that it doesn't increase packet length much, and won't restrict compression.
    override public string GetStates()
    {
        // Save states that we want to synchronize over internet
        string outstring = base.GetStates();

        // Add our intake state machine to it
        outstring += ";"; // different from base seperator

        outstring += (int)intakeL_statemachine;
        outstring += ";";
        outstring += (int)intakeR_statemachine;
        outstring += ";" + ((hand1lockbar.IsLocked()) ? "1" : "0");
        outstring += ";" + ((hand2lockbar.IsLocked()) ? "1" : "0");

        // Nothing else to add now
        return outstring;
    }

    // Read in the states that were turned into a string representation from GetStates()
    override public void SetStates(string instring)
    {
        string[] allstates = instring.Split(';');

        base.SetStates(allstates[0]);

        // Do holding position
        if (allstates.Length > 2)
        {
            intakeL_statemachine = (intakeStates)int.Parse(allstates[1]);
            intakeR_statemachine = (intakeStates)int.Parse(allstates[2]);
            hand1lockbar.ForceLock( (allstates[3][0] == '1') ? true : false);
            hand2lockbar.ForceLock( (allstates[4][0] == '1') ? true : false );
        }

    }
}



