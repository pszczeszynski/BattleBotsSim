
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Ring Shooter, heavily based off FRC shooter
// *******************************



public class Robot_goBilda : RobotInterface3D
{

    private bool init_done = false;

    // Speeds
    [Header("Bot float settings")]
    public float intake_speed = -750f;  // Collector intake rotation speed
   
 
    public float platform_wheel_speed = 400f;


    // Objects that we need to manipulate
    [Header("Bot Linked Objects")]
    public HingeJoint intake_collector;

    public HingeJoint bucket;
    public HingeJoint left_platform_wheel;
    public HingeJoint right_platform_wheel;

    public ConfigurableJoint slide_last;
    public Transform slide2;
    public Transform slide0;
    public float slide_max = 1f;
    public float slide_min = 0f;
    public float slide_speed = 10f;

    public float bucket_min = -40f;
    public float bucket_max = 180f;
    public float bucket_speed = 200f;

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

    enum bucketStates : int
    {
        manual = 0,
        intake,
        drop
    }
    bucketStates bucket_statemachine = bucketStates.intake;

    public void Awake()
    {
        info =
            "Gamepad Up/Down: Move Lift" +
            "\nGamepad Left/Right: Rotate Bucket" +
            "\nButton A: Rotate Bucket to loading position" +
            "\nButton B: Reverse intake while held" +
            "\nButton X: Toggle intake" +
            "\nButton Y: Rotate Bucket to max out position" +
            "\nLeft / Right trigger: Rotate left/ right platform wheels" + 
            info;

        // We support 6-wheel, so add it in
        valid_DriveTrains.Add("6-Wheel Tank");
    }

    public override void Init_Robot()
    {
        base.Init_Robot();

        // Increase suspension for mecanum wheels
        if( DriveTrain != "6-Wheel Tank")
        {
            string[] springs = { "TRWheelSpring", "TLWheelSpring", "BLWheelSpring", "BRWheelSpring" };

            foreach (string currspring in springs)
            {
                ConfigurableJoint myjoint = transform.Find(currspring).GetComponent<ConfigurableJoint>();
                JointDrive mydrive = myjoint.xDrive;
                mydrive.positionSpring *= 4f;
                myjoint.xDrive = mydrive;
            }
        }

    }

    public double delta_angle_display = 0f;
    //public override void RobotFixedUpdate()
    //{
    //    if (!rb_body || !myRobotID || !init_done || isKinematic)
    //    { return; }
    //}

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
        // d_pad_up/down moves forearm up/down

        if (gamepad1_dpad_down )
        {
       
            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            // If we are near bottom, rotate bucket as well
            if( MoveSlide( slide_last, Axis.x, slide_min, slide_speed * turning_scaler) < (slide_min+slide_max)*0.25f )
            {
                MoveHinge(bucket, bucket_min, bucket_speed * turning_scaler);
            }

        }
        else if (gamepad1_dpad_up )
        {     
            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            MoveSlide(slide_last, Axis.x, slide_max, slide_speed * turning_scaler);
           

            // If we are moving up, then also make sure bucket is tiled up at least ~45 degrees above minimum
            if( bucket.spring.targetPosition < (slide_min+45f))
            {
                MoveHinge(bucket, bucket_min+45f, bucket_speed );
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
            
                intake_collector.motor = outmotor;
                intake_collector.useMotor = true;


                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }

            }
        }

        // B button reverses
        if (gamepad1_b || (intake_statemachine == intakeStates.reverse))
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

            // If it was y button_a press, make sure to disengage state machine
            if (gamepad1_b) { intake_statemachine = intakeStates.off; }
        }
        
        // Check to turn off intake
        if(  !gamepad1_b && (intake_statemachine == intakeStates.off))
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


        // **********************************
        // Depositor
        if (gamepad1_dpad_right)
        {
            bucket_statemachine = bucketStates.manual;

            if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Play("bucket", 0.5f);
            }

            MoveHinge(bucket, bucket_max, bucket_speed * turning_scaler);
        }
        else if (gamepad1_dpad_left)
        {
            bucket_statemachine = bucketStates.manual;

            if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Play("bucket", 0.5f);
            }

            MoveHinge(bucket, bucket_min, bucket_speed * turning_scaler);
        }
        else if (bucket_statemachine == bucketStates.manual)
        {
            if (audio_manager && audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Stop("bucket", 0.5f);
            }
        }

        if( gamepad1_y)
        {
            bucket_statemachine = bucketStates.drop;
            if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Play("bucket", 0.5f);
            }
        }

        if( gamepad1_a)
        {
            bucket_statemachine = bucketStates.intake;
            if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Play("bucket", 0.5f);
            }
        }

        // Deal with bucket state machine
        switch (bucket_statemachine)
        {
            case bucketStates.intake:
                    if(MoveHinge(bucket, bucket_min, bucket_speed) == bucket_min)
                    {
                        bucket_statemachine = bucketStates.manual;
                    }
                break;

            case bucketStates.drop:
                if (MoveHinge(bucket, bucket_max, bucket_speed) == bucket_max)
                {
                    bucket_statemachine = bucketStates.manual;
                }
                break;
        }


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
        // Adjust slide 2,1 position to match main slide
        float delta_distance = -1f * (slide_last.transform.position - slide0.transform.position).magnitude;

        Vector3 slide2_pos = slide2.localPosition;
        float slide0_pos_x = (Quaternion.Inverse(slide0.localRotation) * slide0.transform.localPosition).x;
        slide2_pos = Quaternion.Inverse(slide2.localRotation) * slide2_pos;
        slide2_pos.x = slide0_pos_x + delta_distance * 0.5f;
        slide2.localPosition = slide2.localRotation * slide2_pos;
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



