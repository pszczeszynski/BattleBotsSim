
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for MiniDrone
// *******************************



public class Robot_RR_MiniDrone : RobotInterface3D
{

    private bool init_done = false;

    // Speeds
    [Header("Float settings")]
    public float ball_feeder_speed = 1.5f; // Wheels that move ball from colelcted to shooting
    public float ball_feeder_hinge_speed = 50f; // Wheels that move ball from colelcted to shooting
    public float ball_out_speed = 2.0f;  // Velocity outake (final stage)
    public float aim_tilt_speed = -300f;    // aimer tilting speed
    public float arm_start = 0f; // lower starting position
    public float arm_end = -1f; // upper ending position 
    public float arm_speed = 1f; // arm speed
    public float aim_target_pos = 10f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 30f;

    // Objects that we need to manipulate
    [Header("FRC Bot Linked Objects")]
    public ballshooting ball_feeder;
    public HingeJoint ball_aimer;
    public ballshooting ball_out_forcer;
    public ballshooting ball_intake_forcer;
    public GameObject indicator;
    public ConfigurableJoint arm;
    public ConfigurableJoint armdummy;


    // Other variables for internal use
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;
    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;


    private bool movedLift_sound = false;

    // Implement a dictionary??
    // protected Dictionary<string, int> userFlags = new Dictionary<string, int>();

    public void Awake()
    {
        info =
            "Gamepad Up/Down: Aim Up/Down\n" +
            "Gamepad Left/Right: Lift Down/Up\n" +
            "Button Y: Spit balls out\n" +
            "Button X: Shoot balls\n" +
            "Button B: Aim to home position (while held)" +
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
            // Initialize output force velocity
            ball_out_forcer.speed = ball_out_speed;

            // Initialize local cache variables
            sound_controller = GetComponentInChildren<Sound_Controller_FRC_Shooter>();
            audio_manager = GetComponentInChildren<AudioManager>(); // Only 1 audio manager in scene.

            init_done = true;
        }

        // If this is a client side, only do animations
        DoAnimations(isKinematic);

        if (isKinematic)
        {
            return;
        }

        // ****************************
        // Y button: reverse intake wheels - but only while it is pressed
        if (gamepad1_y)
        {
            // TODO: REVERSE INTAKE
            ball_intake_forcer.speed = Math.Abs(ball_intake_forcer.speed) * -1f;
        }
        else
        {
            ball_intake_forcer.speed = Math.Abs(ball_intake_forcer.speed);
        }


        bool movedLift = false;
        // Move lift up
        if (gamepad1_dpad_right)
        {
            movedLift = true;
            float targetpos = MoveTowards(arm_start, arm_end, arm.targetPosition.x, Time.deltaTime * arm_speed);
            arm.targetPosition = new Vector3(targetpos, 0, 0);
            if (armdummy)
            {
                armdummy.targetPosition = new Vector3((targetpos - arm_start) / 2f + arm_start, 0, 0);
            }

        }

        // Move lift down
        if (gamepad1_dpad_left)
        {
            movedLift = true;
            float targetpos = MoveTowards(arm_end, arm_start, arm.targetPosition.x, Time.deltaTime * arm_speed);
            arm.targetPosition = new Vector3(targetpos, 0, 0);
            if (armdummy)
            {
                armdummy.targetPosition = new Vector3((targetpos - arm_start) / 2f + arm_start, 0, 0);
            }
        }
        if (movedLift && (arm.targetPosition.x != arm_start) && (arm.targetPosition.x != arm_end))
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

        if (gamepad1_b) // Seek home position 
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, aim_target_pos, aim_tilt_speed);
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

        float ball_out_speed_mult = 1f;
        if(ball_aimer.spring.targetPosition < ((outhinge_max+outhinge_min)*2f/3f-0.15f))
        {
            ball_out_speed_mult = (ball_aimer.spring.targetPosition - outhinge_min) / (outhinge_max - outhinge_min)*3f/2f + 0.15f;
        }
        ball_out_forcer.speed = ball_out_speed * ball_out_speed_mult;

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



        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
            //float
            rotation = ball_aimer.spring.targetPosition;
            
            rotation = (rotation - outhinge_min) / (outhinge_max - outhinge_min) * 180f - 90f;

            // Now rotation is to rotate around the x-asis before the robot was moved.
            indicator_rotation.eulerAngles = new Vector3(-1f * rotation, 0, 0);

            RobotFixedUpdate();
        }


    }

    public float rotation = 0f;

    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
       

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
        if (MyUtils.AngleWrap(hinge.targetRotation.eulerAngles.x) == target) { return target; }

        float temppos;
        temppos = (float)MyUtils.AngleWrap(hinge.targetRotation.eulerAngles.x);

        float newvalue = MoveTowards(temppos, target, temppos, Time.deltaTime * speed);
        temppos = newvalue;

        Vector3 rotation_euler = hinge.targetRotation.eulerAngles;
        rotation_euler.x = temppos;
        hinge.targetRotation = Quaternion.Euler(rotation_euler);

        return temppos;
    }


    // ********* State Sync for server/client
    // Make sure the string is a small, mainly numerical value (e.g. 1:2:0.11...) so that it doesn't increase packet length much, and won't restrict compression.
    override public string GetStates()
    {
        // Save states that we want to synchronize over internet
        string outstring = base.GetStates();

        // Nothing else to add now
        return outstring;
    }

    // Read in the states that were turned into a string representation from GetStates()
    override public void SetStates(string instring)
    {
        string[] allstates = instring.Split(';');

        base.SetStates(allstates[0]);


    }

}



