
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Ring Shooter, heavily based off FRC shooter
// *******************************



public class Robot_KP_ff : RobotInterface3D
{

    private bool init_done = false;

    // Speeds
    [Header("Bot float settings")]
    public float intake_speed = -750f;  // Collector intake rotation speed
   
    public float forearm_min = 0f;
    public float forearm_max = 180f;
    public float forearm_speed = 200f;
    public float platform_wheel_speed = 400f;


    // Objects that we need to manipulate
    [Header("Bot Linked Objects")]
    public HingeJoint intake_collector;

    public HingeJoint forearm;
    public HingeJoint bucket;
    public HingeJoint left_platform_wheel;
    public HingeJoint right_platform_wheel;
    public ballshooting ramp_force_box;
    public float ramp_speed = 1.5f;
    public HingeJoint out_flap;
    public float flap_speed = 200f;

    [Header("Animation Aid")]
    public Transform intake_spinner1_zaxis;



    // Other variables for internal use
    private AudioManager audio_manager;

    // Intake state machine
    enum intakeStates : int
    {
        off = 0,
        on,
        reverse
    }
    intakeStates intake_statemachine = intakeStates.off;
    intakeStates old_intake_statemachine = intakeStates.off;
    public void Awake()
    {
        info =
            "Gamepad Up/Down: Move Arm" +
            "\nGamepad Left/Right: Rotate Bucket" +
            "\nButton A: Toggle freight release flap" +
            "\nButton B: Toggle intake reverse" +
            "\nButton Y: Reverse intake while held" +
            "\nButton X: Toggle intake" +
            "\nLeft/Right trigger: Rotate left/right platform wheels" +
            info;
    }

    public override void Start()
    {
        base.Start();
    }

    public double delta_angle_display = 0f;
    public override void RobotFixedUpdate()
    {
        if (!rb_body || !myRobotID || !init_done || isKinematic)
        { return; }
    }

    private bool sound_shooting = false;
    private float spring_target_pos = 0f;
    public override void Update_Robot()
    {
        // Remember some variables that we scale later during init
        if (!init_done)
        {

            // Initialize local cache variables
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
        // d_pad_up/down moves forearm up/down

        if (gamepad1_dpad_down )
        {
            MoveHinge(forearm, forearm_min, forearm_speed * turning_scaler);
            MoveHinge(bucket,  forearm.spring.targetPosition, 2f * forearm_speed);

            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            // Reset spring target position for depositor flap
            spring_target_pos = 0f;

        }
        else if (gamepad1_dpad_up )
        {
            MoveHinge(forearm, forearm_max, forearm_speed * turning_scaler);
            MoveHinge(bucket, forearm.spring.targetPosition, 2f*forearm_speed);

            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            // Reset spring target position for depositor flap <steven recommended not while going up>
            //spring_target_pos = 0f;
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Stop("arm", 0.5f);
            }
        }



        // ****************************
        // Intake Machine

        // A (no longer)button runs intake
        if ( intake_statemachine == intakeStates.on)
        {
            // Make sure intake is on
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity != intake_speed)
            {         
                outmotor.targetVelocity = intake_speed;
                ramp_force_box.speed = ramp_speed;
                intake_collector.motor = outmotor;
                intake_collector.useMotor = true;


                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }

            }
        }

        // Y button reverses
        if (gamepad1_y || (intake_statemachine == intakeStates.reverse))
        {
            // make sure intake is reversed
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity !=  -1f * intake_speed)
            {
                outmotor.targetVelocity =  -1f * intake_speed;
                ramp_force_box.speed = -1f * ramp_speed;
                intake_collector.motor = outmotor;
                intake_collector.useMotor = true;

                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }
            }

            // If it was y button_a press, make sure to disengage state machine
            if (gamepad1_y) { intake_statemachine = intakeStates.off; }
        }
        
        // Check to turn off intake
        if(  !gamepad1_y && (intake_statemachine == intakeStates.off))
        {
            // Keep it off
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity != 0f)
            {
                outmotor.targetVelocity = 0f;
                ramp_force_box.speed = 0;
                JointSpring myspring = intake_collector.spring;
                myspring.targetPosition = intake_collector.angle;
                intake_collector.spring = myspring;
                intake_collector.motor = outmotor;
                intake_collector.useMotor = false;

                if (audio_manager && audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Stop("intake", 0.3f);
                }
            }
        }

        // *****************
        // x/b instead toggle intake on/off
        if (gamepad1_x_changed && gamepad1_x)
        {
            if (intake_statemachine == intakeStates.on)
            {
                intake_statemachine = intakeStates.off;
            }
            else
            {
                intake_statemachine = intakeStates.on;
            }
        }


        if (gamepad1_b_changed && gamepad1_b)
        {
            if (intake_statemachine == intakeStates.reverse)
            {
                intake_statemachine = intakeStates.off;
            }
            else
            {
                intake_statemachine = intakeStates.reverse;
            }
        }

        // **********************************
        // Depositor
        if (gamepad1_a_changed && gamepad1_a)
        {
            if(spring_target_pos == 0 )
            {
                spring_target_pos = -60f;
            }
            else
            {
                spring_target_pos = 0f;
            }
        }

        MoveHinge(out_flap, spring_target_pos, flap_speed);


        // ****************************************
        // Platform Rotation
        bool play_wheels = false;

        // Move platform arms up/down
        if (gamepad1_left_trigger > 0f)
        {
            JointMotor outmotor = left_platform_wheel.motor;
            outmotor.targetVelocity = gamepad1_left_trigger * platform_wheel_speed * turning_scaler;
            left_platform_wheel.motor = outmotor;
            play_wheels = true;
        }
        else
        {
            JointMotor outmotor = left_platform_wheel.motor;
            outmotor.targetVelocity = 0f;
            left_platform_wheel.motor = outmotor;
        }

        if (gamepad1_right_trigger > 0f)
        {
            JointMotor outmotor = right_platform_wheel.motor;
            outmotor.targetVelocity = gamepad1_right_trigger * platform_wheel_speed * turning_scaler;
            right_platform_wheel.motor = outmotor;
            play_wheels = true;
        }
        else
        {
            JointMotor outmotor = right_platform_wheel.motor;
            outmotor.targetVelocity = 0f;
            right_platform_wheel.motor = outmotor;
        }

        if (play_wheels)
        {
            if (audio_manager && !audio_manager.IsSoundStarted("wheels"))
            {
                audio_manager.Play("wheels", 0.5f);
            }
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("wheels"))
            {
                audio_manager.Stop("wheels", 0.2f);
            }
        }

    }

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

        // Add our intake state machine to it
        outstring += ";"; // different from base seperator

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



