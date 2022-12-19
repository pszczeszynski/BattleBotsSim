
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for MiniDrone
// *******************************



public class Robot_RR_Bulldogs : RobotInterface3D
{

    private bool init_done = false;

    // Speeds
    [Header("Float settings")]
    public float intake_speed = -3000f;  // Collector intake rotation speed
    public float ball_feeder_speed = 1.5f; // Wheels that move ball from colelcted to shooting
    public float ball_feeder_hinge_speed = 50f; // Wheels that move ball from colelcted to shooting
    public float ball_out_speed = 2.0f;  // Velocity outake (final stage)
    public float aim_tilt_speed = -300f;    // aimer tilting speed
    public float arm_start = 0f; // lower starting position
    public float arm_end = -1f; // upper ending position 
    public float arm2_start = 0f; // lower starting position
    public float arm2_end = -0.45f; // upper ending position 
    public float arm_speed = 1f; // arm speed
    public float aim_target_pos = 10f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 30f;

    // Objects that we need to manipulate
    [Header("FRC Bot Linked Objects")]
    public HingeJoint intake_collector;
    public HingeJoint intake_collector2;
    public HingeJoint intake_collector3;

    public ballshooting_v2 ballforcer_1;
    public ballshooting_v2 ballforcer_2;
    public ballshooting_v2 ballforcer_3;
    public ballshooting_v2 ball_out_forcer;

    public ConfigurableJoint ball_aimer_cj;

    public GameObject indicator;
    public ConfigurableJoint arm;
    public HingeJoint arm1hinge;
    public ConfigurableJoint arm2;
    public HingeJoint arm2hinge;
    public float arm_hinge_min = -40f;
    public float arm_hinge_max = 0f;
    private bool arm1_going_down = true;
    private bool arm2_going_down = true;
    private bool invert_arm_rotation = false;
    public float arm_hinge_speed = 2f;



    [Header("Animation Aid")]
    public Transform intake_spinner1_zaxis;
    public Transform intake_spinner2_zaxis;
    public Transform intake_spinner3_zaxis;

    // Other variables for internal use
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;
    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;
    private bool a_button_last = false;

    private bool movedLift_sound = false;

    // Implement a dictionary??
    // protected Dictionary<string, int> userFlags = new Dictionary<string, int>();

    public enum intakeStates : int
    {
        off = 0,
        onNormal,
        reverse
    }
    public intakeStates intake_statemachine = intakeStates.off;
    intakeStates old_intake_statemachine = intakeStates.off;

    public AddTorque do_torque;
    public float rot_force = 10f;

    public void Awake()
    {
        info =
            "Gamepad Up/Down: Aim Up/Down\n" +
            "Gamepad Left/Right: Climber arm1 Down/Up\n" +
            "Gamepad Left/Right Trigger: Climber arm2 Down/Up\n" +
            "  Note: Arms tilt down if raised, tilt up if lowered.\n  Toggle Button-Y to reverse.\n" +
            "Button Y: Spit balls out (and toggle arm tilt direction)\n" +
            "Button X: Shoot balls\n" +
            "Button B: Aim to home position (while held)\n" +
            "Button A: Intake On/Off" +
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

    private bool sound_shooting = false;
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
        // Y button: reverse intake wheels - but only while it is pressed
        if (gamepad1_y)
        {
            intake_statemachine = intakeStates.reverse;
        }
        else if (intake_statemachine == intakeStates.reverse)
        {
            intake_statemachine = intakeStates.onNormal;
        }

        // ***************************
        // Toggle arm rotation
        if ( gamepad1_y_changed && gamepad1_y)
        {
            invert_arm_rotation = !invert_arm_rotation;
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
                if (sound_controller) { sound_controller.revdown(); }
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
            curr_hinge_pos = MoveHinge(ball_aimer_cj, outhinge_min, aim_tilt_speed * turning_scaler);

        }
        if (gamepad1_dpad_up)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer_cj, outhinge_max, aim_tilt_speed * turning_scaler);

        }

        if (gamepad1_b) // Seek home position 
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer_cj, aim_target_pos, aim_tilt_speed);
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


        // ****************************
        // Intake Wheel State Machine
        // Also controls the cpanel wheel
        ballforcer_1.speed = Math.Abs(ballforcer_1.speed);
        ballforcer_2.speed = Math.Abs(ballforcer_2.speed);
        ballforcer_3.speed = Math.Abs(ballforcer_3.speed);

        if (intake_statemachine == intakeStates.onNormal)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = intake_speed;
            intake_collector.motor = outmotor;

            if (intake_collector2)
            {
                outmotor = intake_collector2.motor;
                outmotor.targetVelocity = intake_speed;
                intake_collector2.motor = outmotor;
            }

            if (intake_collector3)
            {
                outmotor = intake_collector3.motor;
                outmotor.targetVelocity = intake_speed;
                intake_collector3.motor = outmotor;
            }
        }
        if (intake_statemachine == intakeStates.off)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = 0;
            intake_collector.motor = outmotor;

            if (intake_collector2)
            {
                outmotor = intake_collector2.motor;
                outmotor.targetVelocity = 0;
                intake_collector2.motor = outmotor;
            }

            if (intake_collector3)
            {
                outmotor = intake_collector3.motor;
                outmotor.targetVelocity = 0;
                intake_collector3.motor = outmotor;
            }

        }
        if (intake_statemachine == intakeStates.reverse)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = -1f * intake_speed;
            intake_collector.motor = outmotor;
            if (intake_collector2)
            {
                outmotor = intake_collector2.motor;
                outmotor.targetVelocity = -1f * intake_speed;
                intake_collector2.motor = outmotor;
            }

            if (intake_collector3)
            {
                outmotor = intake_collector3.motor;
                outmotor.targetVelocity = -1f * intake_speed;
                intake_collector3.motor = outmotor;
            }

            // Reverse all ballshooters
            ballforcer_1.speed = -1f * ballforcer_1.speed;
            ballforcer_2.speed = -1f * ballforcer_2.speed;
            ballforcer_3.speed = -1f * ballforcer_3.speed;
        }





        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
            //float
            rotation = MyUtils.GetPitchYawRollDeg(ball_aimer_cj.targetRotation).x;

            rotation = (rotation - outhinge_min) / (outhinge_max - outhinge_min) * 180f - 90f;

            // Now rotation is to rotate around the x-asis before the robot was moved.
            indicator_rotation.eulerAngles = new Vector3(-1f * rotation, 0, 0);

            RobotFixedUpdate();
        }


        // Shoot balls
        // Feed balls in while x is on
        if (gamepad1_x)
        {
            ball_out_forcer.hard_stop = false;

            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f, -1); }
                sound_shooting = true;
            }

        }
        else
        {
            ball_out_forcer.hard_stop = true;

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }
        }

        bool movedLift = false;
        bool movedLift2 = false;

        // Move lift up
        if (gamepad1_dpad_right)
        {
            movedLift = true;
            float targetpos = MoveTowards(arm_start, arm_end, arm.targetPosition.x, turning_scaler * Time.deltaTime * arm_speed);
            arm.targetPosition = new Vector3(targetpos, 0, 0);
            arm1_going_down = false;
        }

        // Move lift down
        if (gamepad1_dpad_left)
        {
            movedLift = true;
            float targetpos = MoveTowards(arm_end, arm_start, arm.targetPosition.x, turning_scaler * Time.deltaTime * arm_speed);
            arm.targetPosition = new Vector3(targetpos, 0, 0);
            arm1_going_down = true;
        }

        // Move lift2 up
        if (gamepad1_right_trigger > 0f)
        {
            movedLift2 = true;
            float targetpos = MoveTowards(arm2_start, arm2_end, arm2.targetPosition.x, turning_scaler * Time.deltaTime * arm_speed * gamepad1_right_trigger);
            arm2.targetPosition = new Vector3(targetpos, 0, 0);
            arm2_going_down = false;
        }

        // Move lift2 down
        if (gamepad1_left_trigger > 0f)
        {
            movedLift2 = true;
            float targetpos = MoveTowards(arm2_end, arm2_start, arm2.targetPosition.x, turning_scaler * Time.deltaTime * arm_speed * gamepad1_left_trigger);
            arm2.targetPosition = new Vector3(targetpos, 0, 0);
            arm2_going_down = true;
        }

       
 
        if (movedLift2 && (arm2.targetPosition.x != arm2_start) && (arm2.targetPosition.x != arm2_end) || movedLift && (arm.targetPosition.x != arm_start) && (arm.targetPosition.x != arm_end))
        {
            if (!movedLift_sound)
            {
                if (audio_manager) { audio_manager.Play("hangerraise", 0); }
                movedLift_sound = true;
            }
        }
        else 
        {
            if (audio_manager) { audio_manager.Stop("hangerraise", 0.3f); }
            movedLift_sound = false;
        }

      
        // Move the arm hinges
        if( (!invert_arm_rotation && arm1_going_down || invert_arm_rotation && !arm1_going_down ) && arm1hinge.spring.targetPosition > arm_hinge_min)
        {
            MoveHinge(arm1hinge, arm_hinge_min,  arm_hinge_speed);
        }
        else if ((!invert_arm_rotation && !arm1_going_down || invert_arm_rotation && arm1_going_down) && arm1hinge.spring.targetPosition < arm_hinge_max)
        {
            MoveHinge(arm1hinge, arm_hinge_max, arm_hinge_speed);
        }

        if ((!invert_arm_rotation && arm2_going_down || invert_arm_rotation && !arm2_going_down) && arm2hinge.spring.targetPosition > arm_hinge_min)
        {
            MoveHinge(arm2hinge, arm_hinge_min, arm_hinge_speed);
        }
        else if ((!invert_arm_rotation && !arm2_going_down || invert_arm_rotation && arm2_going_down) && arm2hinge.spring.targetPosition < arm_hinge_max)
        {
            MoveHinge(arm2hinge, arm_hinge_max, arm_hinge_speed);
        }




        /*

 

  

        // Feed balls in while x is on
        if (gamepad1_x)
        {
            ball_feeder.speed = ball_feeder_speed;
        
            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f, -1); }
                sound_shooting = true;
            }

        }
        else
        {
            ball_feeder.speed = -0.5f * ball_feeder_speed;

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }
        }

        */




    }

    public float rotation = 0f;

    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
        if (!client_mode) { return; }

        if (intake_statemachine != intakeStates.off)
        {

            float speed = intake_speed * Time.deltaTime;
            if (intake_statemachine == intakeStates.reverse)
            { speed *= -1f; }

            if (intake_spinner1_zaxis)
            {
                intake_spinner1_zaxis.Rotate(-1f*speed, 0, 0, Space.Self);

            }

            if (intake_spinner2_zaxis)
            {
                intake_spinner2_zaxis.Rotate(-1f*speed, 0, 0, Space.Self);
            }

            if (intake_spinner3_zaxis)
            {
                intake_spinner3_zaxis.Rotate(-1f*speed, 0, 0, Space.Self);
            }
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



