
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Ring Shooter, heavily based off FRC shooter
// *******************************



public class Robot_DiscShooter : RobotInterface3D
{

    private bool init_done = false;

    // Speeds
    [Header("Bot float settings")]
    public float intake_speed = -750f;  // Collector intake rotation speed
    public float ring_feeder_speed = 1.5f; // Ballshootign script speed to move ring from bucket
    public float ring_out_speed = 8.0f;  // Velocity outake (final stage) maximum
    public float ring_out_speed_min = 5f; // Minimum speed
    public float aim_speed = 1.5f;  // Number of seconds to go from min to max

    public float aim_tilt_speed = -300f;    // aimer tilting speed
    public float aim_target_pos = 10f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 30f;
    public float ring_velocity = 6f;
    public float bucket_load_pos = 0f;
    public float bucket_shoot_pos = 25f;
    public float bucket_speed = 2f;


    //public float arm_start = 0f; // lower starting position
    //public float arm_end = -1f; // upper ending position 
    //public float arm_speed = 1f; // arm speed

    // Objects that we need to manipulate
    [Header("Bot Linked Objects")]
    public HingeJoint intake_collector;
    public HingeJoint intake_collector2;
    public HingeJoint intake_collector3;
    public HingeJoint bucket;
    public HingeJoint ring_aimer;  // Aims the rings up/down
    public ballshooting_v2 ring_out_forcer; // Shoot the final ring out
    public ballshooting_v2 ring_feeder; // Moves ring from bucket to shooter
    public GameObject indicator;
    private float indicator_position = 0f; // Percentage (0 to 1f) the aimer is rotated

    //public ConfigurableJoint arm;
    //public ConfigurableJoint armdummy;
    public Transform flywheel;
    public float flywheel_speed = 1f;

    public Transform rollerwheel;
    public HingeJoint rollerhinge;
    public float roller_speed = 6f;


    [Header("Animation Aid")]
    public Transform intake_spinner1_zaxis;
    public Transform intake_spinner2_zaxis;
    public Transform intake_spinner3_zaxis;


    // Other variables for internal use
    private bool a_button_last = false;
    private Vector3 indicator_starting_pos;
    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;

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


    enum bucketStates : int
    {
        loading = 0,
        shooting,
        other
    }
    bucketStates bucket_statemachine = bucketStates.loading;

    public void Awake()
    {
        info =
            "Button X: Shoot (while held)\n" +
            "Button A: Toggle intake\n" + 
            "Button Y: Reverse intake (while held)\n" +
            "Button B: Dump bucket\n" +
            "Left/Right Trigger: Rotate Rollers\n" +
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
            indicator.transform.localRotation = rb_body.transform.localRotation;
            indicator.transform.localPosition = rb_body.transform.localPosition + rb_body.transform.localRotation * (indicator_starting_pos+indicator_position*(new Vector3(0,0,-0.818f)));
        }
    }

    private bool sound_shooting = false;
    public override void Update_Robot()
    {
        // Remember some variables that we scale later during init
        if (!init_done)
        {
            // Initialize output force velocity
            ring_out_forcer.speed = ring_out_speed;

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
            intake_statemachine = intakeStates.reverse;
        }
        else if (intake_statemachine == intakeStates.reverse)
        {
            intake_statemachine = intakeStates.onNormal;
            bucket_statemachine = bucketStates.loading;
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
                bucket_statemachine = bucketStates.loading;
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

        if (gamepad1_dpad_down)
        {
            movedAim = true;
            ring_out_forcer.speed -= (Time.deltaTime / aim_speed) * (ring_out_speed - ring_out_speed_min);
            if(ring_out_forcer.speed < ring_out_speed_min)
            {
                ring_out_forcer.speed = ring_out_speed_min;
            }

        }
        if (gamepad1_dpad_up)
        {
            movedAim = true;
            ring_out_forcer.speed += (Time.deltaTime / aim_speed) * (ring_out_speed - ring_out_speed_min);
            if (ring_out_forcer.speed > ring_out_speed)
            {
                ring_out_forcer.speed = ring_out_speed;
            }
        }


        if (movedAim && (ring_out_forcer.speed != ring_out_speed) && (ring_out_forcer.speed != ring_out_speed_min) )
        {
            
            if (audio_manager && !audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Play("adjustangle", 0.1f);
            }
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Stop("adjustangle", 0.3f);
            }
        }

        // ****************************
        // d_pad_left/right moves ???
        bool movedLift = false;

        if (gamepad1_dpad_left)
        {
            //MoveHinge(forearm, forearm_min, forearm_speed);
            movedLift = true;

        }
        if (gamepad1_dpad_right)
        {
            //MoveHinge(forearm, forearm_max, forearm_speed);

            movedLift = true;
        }


      

        // Feed balls in while x is on
        // also disable intake
        if (gamepad1_x)
        {
            ring_feeder.speed = ring_feeder_speed;
            bucket_statemachine = bucketStates.shooting;
            intake_statemachine = intakeStates.off;

            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f); }
                sound_shooting = true;
            }

        }
        else
        {
            // ring_feeder.disable = true;
            ring_feeder.speed = 0f;

            // If intake state-machine is reversing, then also reverse the extra ball feeders.
            if (intake_statemachine == intakeStates.reverse)
            {
                ring_feeder.speed = -1f * ring_feeder_speed;
            }

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }
        }

        // If reversing and shooting buttons are pressed at the same time, then lift bucket way up to clear
        if(gamepad1_b)
        {
            bucket_statemachine = bucketStates.other;
        }


        // ****************************
        // Intake Wheel State Machine
        if (intake_statemachine == intakeStates.onNormal)
        {
            SetIntakeSpeed(intake_speed);           

        }
        if (intake_statemachine == intakeStates.off)
        {
            SetIntakeSpeed(0f);        
        }
        if (intake_statemachine == intakeStates.reverse)
        {
            SetIntakeSpeed(-1f * intake_speed);

        }

        // **********************
        // roller wheel
        if (gamepad1_left_trigger > 0f)
        {
            JointMotor rollermotor = rollerhinge.motor;
            rollermotor.targetVelocity = gamepad1_left_trigger * roller_speed * 120f;
            rollerhinge.motor = rollermotor;
        } else if (gamepad1_right_trigger > 0f)
        {
            JointMotor rollermotor = rollerhinge.motor;
            rollermotor.targetVelocity = -1f * gamepad1_right_trigger * roller_speed * 120f;
            rollerhinge.motor = rollermotor;
        }
        else if(rollerhinge.motor.targetVelocity != 0f )
        {
            JointMotor rollermotor = rollerhinge.motor;
            rollermotor.targetVelocity = 0f;
            rollerhinge.motor = rollermotor;
        }
            


        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
 

            indicator_position = (ring_out_forcer.speed - ring_out_speed_min) / (ring_out_speed - ring_out_speed_min);

            RobotFixedUpdate();
        }

        // ****************************
        // Bucket Wheel State Machine
        float bucket_speed_increaser = (bucket.spring.targetPosition > bucket_shoot_pos) ? 2f : 1f;

        if (bucket_statemachine == bucketStates.loading)
        {
            MoveHinge(bucket, bucket_load_pos, bucket_speed_increaser * bucket_speed);
        }
        else if (bucket_statemachine == bucketStates.shooting)
        {
            MoveHinge(bucket, bucket_shoot_pos, bucket_speed_increaser * bucket_speed);
        }
        else if (bucket_statemachine == bucketStates.other)
        {
            MoveHinge(bucket, bucket_shoot_pos + 70f, bucket_speed_increaser * bucket_speed);
        }
    }

    private void SetIntakeSpeed(float speed)
    {
        JointMotor outmotor;

        if (intake_collector.motor.targetVelocity != speed)
        {
            outmotor = intake_collector.motor;
            outmotor.targetVelocity = speed;
            intake_collector.motor = outmotor;
        }

        if (intake_collector2 && intake_collector2.motor.targetVelocity != 1.5f * speed)
        {
            outmotor = intake_collector2.motor;
            outmotor.targetVelocity = 1.5f * speed;
            intake_collector2.motor = outmotor;
        }

        if (intake_collector3 && intake_collector3.motor.targetVelocity != 2f * speed)
        {
            outmotor = intake_collector3.motor;
            outmotor.targetVelocity = 2f * speed;
            intake_collector3.motor = outmotor;
        }
    }

    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
        // Add flywheel animation (no physics here)
        if (flywheel)
        {
            Quaternion rotation = flywheel.transform.localRotation;
            Quaternion apply_rot = Quaternion.identity;
            apply_rot.y = flywheel_speed * Time.deltaTime;
            rotation = rotation * apply_rot;
            flywheel.transform.localRotation = rotation;
        }

        if (rollerwheel)
        {
            Quaternion rotation = rollerwheel.transform.localRotation;
            Quaternion apply_rot = Quaternion.identity;

            float scale_speed = 0f;
            if( gamepad1_left_trigger > 0f)
            {
                scale_speed = gamepad1_left_trigger;
            }
            if( gamepad1_right_trigger > 0f)
            {
                scale_speed = -1f * gamepad1_right_trigger;
            }
            apply_rot.y = scale_speed * roller_speed * Time.deltaTime;
            rotation = rotation * apply_rot;
            rollerwheel.transform.localRotation = rotation;
        }

        // Do animations to replace physics driven objects

        if (intake_statemachine != intakeStates.off)
        {

            float speed = -1f * intake_speed * Time.deltaTime;
            if (intake_statemachine == intakeStates.reverse)
            { speed *= -1f; }


            if (intake_spinner1_zaxis)
            {
                intake_spinner1_zaxis.Rotate(speed, 0, 0, Space.Self);

            }

            if (intake_spinner2_zaxis)
            {
                intake_spinner2_zaxis.Rotate(speed, 0, 0, Space.Self);
            }

            if (intake_spinner3_zaxis)
            {
                intake_spinner3_zaxis.Rotate(speed, 0, 0, Space.Self);
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



