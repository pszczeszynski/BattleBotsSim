
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Ring Shooter, heavily based off FRC shooter
// *******************************



public class Robot_Bailey : RobotInterface3D
{

    private bool init_done = false;

    // Speeds
    [Header("Bot float settings")]
    public float intake_speed = -750f;  // Collector intake rotation speed
   
    public float forearm_min = -90f;
    public float forearm_deployed = 0f;
    public float forearm_max = 50f;
    public float forearm_speed_start = 300f;
    public float forearm_speed_level = 100f;
    public float bucket_min = 5f;
    public float bucket_max = 35f;
    public float bucket_speed = 75f;
     

    // Objects that we need to manipulate
    [Header("Bot Linked Objects")]
    public HingeJoint intake_collector;

    public HingeJoint forearm;
    public HingeJoint forearm2;
    public HingeJoint hand;
    public HingeJoint finger_l;
    public HingeJoint finger_r;
    public float finger_open = 0f;
    public float finger_closed = -20f; // referenced to left
    public float finger_speed = 80f;



    public float[,] forearm2_map = new float[3, 2]
    {
        {-90,0 },
        {0f, -140f },
        {50f,-35f }
    };

    public float[,] hand_map = new float[5, 2]
    {
        {-90,88 },
        {-75,100 },
        {0f, -5f },
        {30f, -100 },
        {50f,-155f }
    };

    public HingeJoint bucket;


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

    enum armStates : int
    {
        home=0,
        closing,
        deploying,
        deployed,
        opening,
        returning
    }
    armStates arm_state = armStates.opening;

    enum bucketStates : int
    {
        down = 0,
        rising,
        up,
        falling
    }
    bucketStates bucket_state = bucketStates.rising;

    public void Awake()
    {
        info =
            "Gamepad Up/Down: Move Arm up/down when deployed" +
            "\nGamepad Left/Right: retract/deploy arm." +
            "\nButton A: Toggle intake" +
            "\nButton B: open fingers (while held)" +
            "\nButton Y: Toggle reverse intake" +
            "\nButton X: Raise/Lower bucket. Lower bucket to collect freight. Raise bucket for proper hand grab." +
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
        // Deal with arm-state
        bool play_arm_sound = false;
        bool done_l = false;
        bool done_r = false;

        switch (arm_state)
        {
            case armStates.closing:
                done_l = MoveHinge(finger_l, finger_closed, finger_speed) == finger_closed;
                done_r = MoveHinge(finger_r, -1f * finger_closed, finger_speed) == -1f * finger_closed;

                if ( done_l && done_r )
                {
                    arm_state = armStates.deploying;        
                }


                if (audio_manager && !audio_manager.IsSoundStarted("arm"))
                {
                    audio_manager.Play("arm", 0.5f);
                }
                play_arm_sound = true;

                break;


            case armStates.deploying:
                if( MoveArm(forearm_deployed) )
                {
                    arm_state = armStates.deployed;
                }

                play_arm_sound = true;
                break;

            case armStates.deployed:
                if (gamepad1_dpad_down)
                {
                    MoveArm(forearm_max, forearm_speed_level);
                    play_arm_sound = true;
                }
                else if (gamepad1_dpad_up)
                {
                    MoveArm(forearm_deployed, forearm_speed_level);
                    play_arm_sound = true;
                }
                
                break;

            case armStates.opening:
                done_l = MoveHinge(finger_l, finger_open, finger_speed) == finger_open;
                done_r = MoveHinge(finger_r, -1f * finger_open, finger_speed) == -1f * finger_open;

                if (done_l && done_r)
                {
                    arm_state = armStates.returning;
                }

                if (audio_manager && !audio_manager.IsSoundStarted("arm"))
                {
                    audio_manager.Play("arm", 0.5f);
                }
                play_arm_sound = true;

                break;

            case armStates.returning:
                // Double make sure fingers are open
                MoveHinge(finger_l, finger_open, finger_speed);
                MoveHinge(finger_r, -1f * finger_open, finger_speed);

                if (MoveArm(forearm_min))
                {
                    arm_state = armStates.home;
                }

                play_arm_sound = true;
                break;

        }

        // Global overrides for arm
        if (gamepad1_dpad_right)
        {
            arm_state = armStates.opening;
        }

        if (gamepad1_dpad_left)
        {
            arm_state = armStates.closing;
        }


        // Open fingers
        if (gamepad1_b)
        {
            MoveHinge(finger_l, 2f * finger_open, finger_speed);
            MoveHinge(finger_r, -2f * finger_open, finger_speed);

            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }
            play_arm_sound = true;
        }

        // d_pad_up/down moves forearm up/down
        if (!play_arm_sound && audio_manager && audio_manager.IsSoundStarted("arm"))
        {
            audio_manager.Stop("arm", 0.5f);
        }

        // *****************************\
        // Bucket states
        // ****************************
        // Deal with arm-state
        bool play_bucket_sound = false;
        switch (bucket_state)
        {
            case bucketStates.rising:
                if( MoveHinge(bucket, bucket_max, bucket_speed) == bucket_max )
                {
                    bucket_state = bucketStates.up;
                }
                play_bucket_sound = true;

                if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
                {
                    audio_manager.Play("bucket", 0.5f);
                }

                break;

            case bucketStates.falling:
                if (MoveHinge(bucket, bucket_min, bucket_speed) == bucket_min)
                {
                    bucket_state = bucketStates.down;
                }
                play_bucket_sound = true;

                if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
                {
                    audio_manager.Play("bucket", 0.5f);
                }

                break;

        }

        // Initiate rising/falling if x is pressed
        if( gamepad1_x_changed && gamepad1_x)
        {
            if( bucket_state == bucketStates.down || bucket_state == bucketStates.falling)
            {
                bucket_state = bucketStates.rising;
            }
            else
            {
                bucket_state = bucketStates.falling;
            }
        }

        // d_pad_up/down moves forearm up/down
        if (!play_bucket_sound && audio_manager && audio_manager.IsSoundStarted("bucket"))
        {
            audio_manager.Stop("bucket", 0.5f);
        }

        // ****************************
        // Intake Machine

        if ( intake_statemachine == intakeStates.on)
        {
            // Make sure intake is on
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity != intake_speed)
            {         
                outmotor.targetVelocity = intake_speed;

                intake_collector.motor = outmotor;
                intake_collector.useMotor = true;


                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }

            }
        }


        if (intake_statemachine == intakeStates.reverse)
        {
            // make sure intake is reversed
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity !=  -1f * intake_speed)
            {
                outmotor.targetVelocity =  -1f * intake_speed;

                intake_collector.motor = outmotor;
                intake_collector.useMotor = true;

                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }
            }

        }
        
        // Check to turn off intake
        if(  intake_statemachine == intakeStates.off)
        {
            // Keep it off
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity != 0f)
            {
                outmotor.targetVelocity = 0f;

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
        // a toggles intake
        if (gamepad1_a_changed && gamepad1_a)
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


        if (gamepad1_y_changed && gamepad1_y)
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

   



    }

    public bool MoveArm(float target, float forearm_speed = -1f)
    {
        // Default to fast deploy speed
        if( forearm_speed < 0 )
        {
            forearm_speed = forearm_speed_start;
        }

        float curr_position = MoveHinge(forearm, target, forearm_speed * turning_scaler);
        MoveHinge(forearm2, MyUtils.GetInterpolatedValue(forearm.spring.targetPosition, forearm2_map), 10f * forearm_speed);
        MoveHinge(hand, MyUtils.GetInterpolatedValue(forearm.spring.targetPosition, hand_map), 10f * forearm_speed);

        if (audio_manager && !audio_manager.IsSoundStarted("arm"))
        {
            audio_manager.Play("arm", 0.5f);
        }

        if( curr_position == target )
        {
            return true;
        }

        return false;
    }

    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
   
   
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



