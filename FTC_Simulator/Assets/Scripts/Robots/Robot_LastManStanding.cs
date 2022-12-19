using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for LastManStanding
// *******************************



public class Robot_LastManStanding : RobotInterface3D {

    private bool init_done = false;

    // Speeds
    [Header("FRC Bot float settings")]
    public float intake_speed = -1000f;  // Collector intake rotation speed
    public float ball_feeder_speed = 1.5f; // Wheels that move ball from colelcted to shooting
    public float ball_feeder_hinge_speed = 50f; // Wheels that move ball from colelcted to shooting
    public float ball_out_speed = 2.0f;  // Velocity outake (final stage)
    public float aim_tilt_speed = -300f;    // aimer tilting speed
    public float aim_target_pos = 10f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 30f;

    // Objects that we need to manipulate
    [Header("FRC Bot Linked Objects")]
    public HingeJoint intake_collector;
    public HingeJoint ball_feeder_hinge1;
    public HingeJoint ball_feeder_hinge2;
    public ballshooting ball_feeder;
    public ballshooting ball_feeder2;
    public ballshooting ball_feeder3;
    public ballshooting ball_feeder4;
    public ballshooting ball_feeder5;
    public HingeJoint ball_aimer;
    public ballshooting ball_out_forcer;
    public GameObject indicator;
    public Transform ballFeedingPos;
    public ballshooting hopperBalls;

    [Header("Animation Aid")]
    public Transform intake_spinner1_zaxis;
    public Transform ball_feeder1_animation;
    public Transform ball_feeder2_animation;

    // Other variables for internal use
    private bool a_button_last = false;
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;
    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;
    private Animation death_animation;


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

    private bool ballfeeder_feeding = false;
    private bool death_animation_playing = false;


    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }
    public override void Start()
    {
        base.Start();
        indicator_starting_pos = indicator.transform.localPosition;
    }

    public double delta_angle_display = 0f;
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

    private bool pauseUpdates_old = false;
    private bool sound_shooting = false;

    public override void Update_Robot()
    {
        // Remember some variables that we scale later during init
        if (!init_done)
        {
            // Initialize output force velocity
            ball_out_forcer.speed = ball_out_speed;

            // Initialize local cache variables
            sound_controller = GetComponent<Sound_Controller_FRC_Shooter>();
            audio_manager = GetComponent<AudioManager>(); // Only 1 audio manager in scene.
            death_animation = GetComponent<Animation>();

            init_done = true;
        }

        // Update death animation state
        if (death_animation_playing)
        {
            death_animation_playing = death_animation.isPlaying;
            return;
        }
        else
        {
            pauseUpdates = false;
            if(pauseUpdates_old != pauseUpdates)
            {
                // Turn pause updates back on
                BandwidthHelper[] allhelpers = GetComponentsInChildren<BandwidthHelper>();
                foreach( BandwidthHelper currhelper in allhelpers )
                {
                    currhelper.pauseUpdates = false;
                }
            }
        }
        pauseUpdates_old = pauseUpdates;

        // If this is a client side, only do internal animations
        if ( GLOBALS.CLIENT_MODE)
        {
            DoAnimations();
            return;
        }

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

        // Do sound for intake state machine
        if (intake_statemachine != old_intake_statemachine)
        {
            // If intake got turned off, play sound
            if (intake_statemachine == intakeStates.off)
            {
                if( sound_controller) { sound_controller.revdown(); }
            }
            // If state machine got turned on, play sound
            else if (old_intake_statemachine == intakeStates.off)
            {
                 if (sound_controller) { sound_controller.revup(); }
            }
        }

        old_intake_statemachine = intake_statemachine;      

        // ****************************
        // d_pad_up/down moves aimer up/down
        bool movedAim = false;
        float curr_hinge_pos = 0f;

        if (gamepad1_dpad_down)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, outhinge_min, aim_tilt_speed * turning_scaler);
            
        }
        if (gamepad1_dpad_up)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, outhinge_max, aim_tilt_speed * turning_scaler);
           
        }
        if (movedAim && (curr_hinge_pos != outhinge_min) && (curr_hinge_pos != outhinge_max))
        {
            if (audio_manager && !audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Play("adjustangle", 0);
            }
        }
        else
        {
            if(audio_manager && audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Stop("adjustangle", 0.3f);
            }
        }


        // Feed balls in while x is on
        if (gamepad1_x)
        {
            ball_feeder.speed = ball_feeder_speed;
            ballfeeder_feeding = true;

            JointMotor outmotor;
            if (ball_feeder_hinge1)
            {
                outmotor = ball_feeder_hinge1.motor;
                outmotor.targetVelocity = ball_feeder_hinge_speed;
                ball_feeder_hinge1.motor = outmotor;
            }
            if (ball_feeder_hinge2)
            {
                outmotor = ball_feeder_hinge2.motor;
                outmotor.targetVelocity = ball_feeder_hinge_speed;
                ball_feeder_hinge2.motor = outmotor;
            }

            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f, -1); }
                sound_shooting = true;
            }


        }
        else
        {
            ball_feeder.speed = -0.1f * ball_feeder_speed;
            ballfeeder_feeding = false;

            // For the intake feeder in forward direction when just waiting but at half speed
            JointMotor outmotor;
            if (ball_feeder_hinge1)
            {
                outmotor = ball_feeder_hinge1.motor;
                outmotor.targetVelocity = -0.25f * ball_feeder_hinge_speed;
                ball_feeder_hinge1.motor = outmotor;
            }

            if (ball_feeder_hinge2)
            {
                outmotor = ball_feeder_hinge2.motor;
                outmotor.targetVelocity = -0.25f * ball_feeder_hinge_speed;
                ball_feeder_hinge2.motor = outmotor;
            }

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }

        }

        if (gamepad1_b)
        {
            MoveHinge(ball_aimer, aim_target_pos, aim_tilt_speed);
        }


        // ****************************
        // Intake Wheel State Machine
        if (intake_statemachine == intakeStates.onNormal)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = intake_speed;
            intake_collector.motor = outmotor;

     
        }
        if (intake_statemachine == intakeStates.off)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = 0;
            intake_collector.motor = outmotor;

        }
        if (intake_statemachine == intakeStates.reverse)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = -1f * intake_speed;
            intake_collector.motor = outmotor;

            // Also reverse the intake feeder hinges
            if (ball_feeder_hinge1)
            {
                outmotor = ball_feeder_hinge1.motor;
                outmotor.targetVelocity = -1f*ball_feeder_hinge_speed;
                ball_feeder_hinge1.motor = outmotor;
            }

            if (ball_feeder_hinge2)
            {
                outmotor = ball_feeder_hinge2.motor;
                outmotor.targetVelocity = -1f * ball_feeder_hinge_speed;
                ball_feeder_hinge2.motor = outmotor;
            }

        }

        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
            float rotation = ball_aimer.spring.targetPosition;
            rotation = (rotation - outhinge_min) / (outhinge_max - outhinge_min) * 180f - 90f;
           
            // Now rotation is to rotate around the x-asis before the robot was moved.
            indicator_rotation.eulerAngles = new Vector3(-1f * rotation, 0, 0);

            RobotFixedUpdate();
        }

        // Preload a ball if no balls in any of our ballhooters
        if( !ball_feeder.AnyBallsInside() && !ball_out_forcer.AnyBallsInside() && !ball_feeder2.AnyBallsInside() && !ball_feeder3.AnyBallsInside() && !ball_feeder4.AnyBallsInside() && !ball_feeder5.AnyBallsInside()
            && hopperBalls.AnyBallsInside())
        {
            // Get a new ball
            gameElement newball = hopperBalls.GetBallInside();
            if( !newball) { return; }

            // Move it to our loading position
            newball.transform.position = ballFeedingPos.position;
        }

    }

    // Do animations
    public void DoAnimations()
    {
        float speed = intake_speed * Time.deltaTime;

        // Do intake animations
        if (intake_statemachine != intakeStates.off)
        {
     
            if (intake_statemachine == intakeStates.reverse)
            { speed *= -1f; }


            if ( intake_spinner1_zaxis)
            {
                intake_spinner1_zaxis.Rotate(0, 0, speed, Space.Self);
        
            }
        }

        speed = ball_feeder_hinge_speed * Time.deltaTime;

        // Do ballfeeder animatin
        if (ballfeeder_feeding)
        {
            ball_feeder1_animation.Rotate(0, speed, 0 , Space.Self);
            ball_feeder2_animation.Rotate(0, speed, 0 , Space.Self);
        }
        else
        {
            ball_feeder1_animation.Rotate(0, -0.5f * speed, 0 , Space.Self);
            ball_feeder2_animation.Rotate(0, -0.5f * speed, 0 , Space.Self);
        }

    }


    // Moves joint by a small delta. If reached destination, return true
    // target is a positive number, but all positions in the spring are negative.
    // I probably should have left everything negative to avoid confusion...
    /*float MoveHinge(HingeJoint hinge, float target, float speed)
    {
        // Constrain target to limits
        if (hinge.limits.max != hinge.limits.min)
        {
            if (hinge.limits.max < target) { target = hinge.limits.max; }
            if (hinge.limits.min > target) { target = hinge.limits.min; }
        }

        // Quit if we reached our target
        if (hinge.spring.targetPosition == target) { return target; }

        float temppos;
        temppos = hinge.spring.targetPosition;

        float newvalue = MoveTowards(temppos, target, temppos, Time.deltaTime * speed);
        temppos = newvalue;

        JointSpring currspring = hinge.spring;
        currspring.targetPosition = temppos;
        hinge.spring = currspring;
        return temppos;
    }

    // Hinge as aconfigurable joint - x-axis being the one to rotate
    float MoveHingeCJ(ConfigurableJoint hinge, float target, float speed)
    {
        // Constrain target to limits
        if (hinge.lowAngularXLimit.limit != hinge.highAngularXLimit.limit)
        {
            if (hinge.highAngularXLimit.limit < target) { target = hinge.highAngularXLimit.limit; }
            if (hinge.lowAngularXLimit.limit > target) { target = hinge.lowAngularXLimit.limit; }
        }

        // Quit if we reached our target
        if (MyUtils.AngleWrap( hinge.targetRotation.eulerAngles.x) == target) { return target; }

        float temppos;
        temppos = (float) MyUtils.AngleWrap(hinge.targetRotation.eulerAngles.x);

        float newvalue = MoveTowards(temppos, target, temppos, Time.deltaTime * speed);
        temppos = newvalue;

        Vector3 rotation_euler = hinge.targetRotation.eulerAngles;
        rotation_euler.x = temppos;
        hinge.targetRotation = Quaternion.Euler(rotation_euler);

        return temppos;
    }
    */

    // ********* State Sync for server/client
    // Make sure the string is a small, mainly numerical value (e.g. 1:2:0.11...) so that it doesn't increase packet length much, and won't restrict compression.
    override public string GetStates()
    {
        // Save states that we want to synchronize over internet
        string outstring = base.GetStates();

        // Add our intake state machine to it
        outstring += ";"; // different from base seperator

        outstring += (int)intake_statemachine;

        // ball feeding
        outstring += ";" + ((ballfeeder_feeding) ? "1" : "0");

        // Animation
        outstring += ";" + ((death_animation_playing) ? "1" : "0");

        // Nothing else to add now
        return outstring;
    }

    private bool animation_playing_old = false;

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

        if( allstates.Length > 2)
        {
            ballfeeder_feeding = ((allstates[2][0] == '1') ? true : false);
        }

        if (allstates.Length > 3)
        {
            death_animation_playing = ((allstates[3][0] == '1') ? true : false);
        }

        // Trigger actions that need triggering
        if( death_animation_playing && (death_animation_playing != animation_playing_old) )
        {
            PlayDeathAnimation();
        }
        animation_playing_old = death_animation_playing;
    }

    // Reset first level children positions to that present at Start (plus an offset)
    // Set Parent to 0 position
    public void ResetPosition_w_y_3()
    {
        ResetPosition(0, 3f, 0);
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.identity;
    }

    public void PlayDeathAnimation()
    {
        death_animation_playing = true;

        // First set it to kinematic
        SetKinematic(true);

        // Mark as holding
        HoldRobot(true);

        // Do thigns that the server only needs to do
        if (!GLOBALS.CLIENT_MODE)
        {
            // Play Death Sound
            rb_body.GetComponentInParent<AudioManager>().Play("Death1", 0, 1);
        }
        else
        {
            // Otherwise we want to stop all robot updates while this is playing
            pauseUpdates = true;

            // Animation needs to turn this back on though!
        }

        // Since animations are not additive (they set the absolute position, not change in position),
        // need to re-center robot around current position (normally it's centered around starting position).

        // Make our position Body's current position (body is 0,0,0 when first instantiated)
        transform.position = rb_body.transform.position;
        transform.rotation = rb_body.transform.rotation;

        // Reset all subcomponents
        ResetPosition();

        // Play death animation
        death_animation.Play();

    }

    public override void EnableTopObjects()
    {
        // Stop animation if playing because it runs stuff at the end we dont want to be run
        if (death_animation && death_animation.isPlaying)
        {
            death_animation.Stop();
        }
        death_animation_playing = false;

        base.EnableTopObjects();

        // Reset their positions
        ResetPosition();
    }


}

