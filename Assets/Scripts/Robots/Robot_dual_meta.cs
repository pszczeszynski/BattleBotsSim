
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Ring Shooter, heavily based off FRC shooter
// *******************************

public class Robot_dual_meta : RobotInterface3D
{

    // Buckets and collectors
    public HingeJoint bucket1;
    public HingeJoint collector1_hinge;
    public PossessionTracker detector1;

    public HingeJoint bucket2;
    public HingeJoint collector2_hinge;
    public PossessionTracker detector2;

    public float bucket_down = 0f;
    public float bucket_up = 90f;
    public float bucket_speed = 200f;
    public float intake_speed = -1400f;  // Collector intake rotation speed


    public ConfigurableJoint slide_last;
    public Transform slide2;
    public Transform slide1;
    public Transform slide0;

    public float slide_max = 1.6f;
    public float slide_min = 0f;
    public float slide_speed = 1f;

    public HingeJoint out_bucket;
    public float out_bucket_down = 0f;
    public float out_bucket_up = 40f;
    public float out_bucket_speed = 200f;

    public HingeJoint flap1;
    public HingeJoint flap2;
    public float flap_down = 0f;
    public float flap_up = 90f;
    public float flap_speed = 200f;

    public HingeJoint floor;
    public float floor_up = 0f;
    public float floor_down = 60f;
    public float floor_speed = 400f;


    public HingeJoint platform_wheel;
    public float platform_wheel_speed = 1800f;


    // Other variables for internal use
    private AudioManager audio_manager;

    // Intake state machine
    enum intakeStates : int
    {
        off = 0,
        on
    }
    intakeStates collector1_statemachine = intakeStates.off;
    intakeStates collector2_statemachine = intakeStates.off;

    enum bucketStates : int
    {
        down = 0,
        movingUp,
        up,
        movingDown

    }
    bucketStates bucket1_statemachine = bucketStates.movingDown;
    bucketStates bucket2_statemachine = bucketStates.movingDown;

    bool doing_reset = false;

    public void Awake()
    {
        info = 

            "Gamepad Up/Down: Move Arm up/down when deployed" +
            "\nGamepad Left/Right: Rotate depositor when arm is extended. Useful for level 1/2 in team hub." +
            "\nButton A: Open depositor floor (while held)" + 
            "\nButton B: Right intake control. Hold to reverse and rotate up. Press to toggle on/off. Will auto rotate up when freight detected." +
            "\nButton Y: While held, opens depositor floor (same as Button A). When released, auto-retract arm." +
            "\nButton X: Leftt intake control. Hold to reverse and rotate up. Press to toggle on/off. Will auto rotate up when freight detected." +
            "\nLeft trigger: Rotate platform wheel counter-clockwise." +
            "\nRight trigger: Rotate platform wheel clockwise." + 
            info;     
    }


    public override void Start()
    {
        base.Start();
    }


    public override void Init_Robot()
    {
        // Initialize local cache variables
        audio_manager = GetComponentInChildren<AudioManager>(); // Only 1 audio manager in scene.       
    }


    private bool force_slide_down = false;
    public override void Update_Robot()
    {
        // Button X: Left collector off/on hold for reverse. If item detected, auto lifts up. Press collector button to put in depositor.
        // Button B: Right collector off/on hold for reverse. If item detected, auto lifts up. Press collector button to put in depositor.

        // Y: Drop freight
        // A: Retract depositor

        // Dpad Up/Down: move lift up down, deploy if retracted
        // Dpad left/right: move bucket in/out.


        // If this is a client side, only do animations
        DoAnimations(isKinematic);

        if (isKinematic)
        {
            return;
        }


        // *******************************
        // **** INTAKE
        // ****************************
        // Intake Machine
        force_slide_down = false;

        ProcessIntake(gamepad1_x, gamepad1_x_changed, ref collector1_statemachine, collector1_hinge, bucket1, ref bucket1_statemachine, detector1, flap1, ref collector2_statemachine);
        ProcessIntake(gamepad1_b, gamepad1_b_changed, ref collector2_statemachine, collector2_hinge, bucket2, ref bucket2_statemachine, detector2, flap2, ref collector1_statemachine);


        // *****************************
        // Reset Button
        // ****************

        if (gamepad1_y_changed && !gamepad1_y)
        {
            bucket1_statemachine = bucketStates.movingDown;
            bucket2_statemachine = bucketStates.movingDown;

            doing_reset = true;
        }

        if (doing_reset)
        {
            force_slide_down = true;

            // Allow cancelling of reset
            if (gamepad1_dpad_up)
            {
                doing_reset = false;
            }
        }


        // *******************************
        // **** Linear Slide
        // ****************************
        if (gamepad1_dpad_down || force_slide_down)
        {

            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            if( MoveSlide(slide_last, Axis.x, slide_min, slide_speed * turning_scaler) == slide_min)
            {
                doing_reset = false;
            }

        }
        else if (gamepad1_dpad_up)
        {
            if (audio_manager && !audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            MoveSlide(slide_last, Axis.x, slide_max, slide_speed * turning_scaler);
          
            // Close any flaps
            MoveHinge(flap1, flap_down, flap_speed);
            MoveHinge(flap2, flap_down, flap_speed);
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Stop("arm", 0.5f);
            }
        }

        // *****************************
        // Floor
        // ******************************
        if ((gamepad1_a || gamepad1_y) && (slide_last.targetPosition.x > ((slide_max - slide_min) * 0.1f + slide_min)))
        {
            MoveHinge(floor, floor_down - out_bucket.spring.targetPosition, floor_speed);
        }
        else
        {
            MoveHinge(floor, floor_up, floor_speed);
        }

        // *****************************
        // Depositor/Out Bucket
        // ******************************

        if (!doing_reset && (gamepad1_dpad_right && slide_last.targetPosition.x > 0.35f))
        {

            if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            MoveHinge(out_bucket, out_bucket_up, out_bucket_speed);

        }
        else if (gamepad1_dpad_left)
        {
            if (audio_manager && !audio_manager.IsSoundStarted("bucket"))
            {
                audio_manager.Play("arm", 0.5f);
            }

            MoveHinge(out_bucket, out_bucket_down, out_bucket_speed);

        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("arm"))
            {
                audio_manager.Stop("bucket", 0.5f);
            }

            if (doing_reset || (slide_last.targetPosition.x < 0.35f))
            {
                MoveHinge(out_bucket, out_bucket_down, 2f * out_bucket_speed);
            }

        }

        // *****************************
        // Platform Wheel
        // ******************************

        bool play_wheels = false;

        if (gamepad1_left_trigger > 0f)
        {
            JointMotor outmotor = platform_wheel.motor;
            outmotor.targetVelocity = -1f * gamepad1_left_trigger * platform_wheel_speed * turning_scaler;
            platform_wheel.motor = outmotor;
            play_wheels = true;
        }
        else if (gamepad1_right_trigger > 0f)
        {
            JointMotor outmotor = platform_wheel.motor;
            outmotor.targetVelocity = gamepad1_right_trigger * platform_wheel_speed * turning_scaler;
            platform_wheel.motor = outmotor;
            play_wheels = true;
        }
        else
        {
            JointMotor outmotor = platform_wheel.motor;
            outmotor.targetVelocity = 0f;
            platform_wheel.motor = outmotor;
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

    private void ProcessIntake(bool button, bool button_changed, ref intakeStates statemachine, HingeJoint my_hinge, HingeJoint bucket, ref bucketStates bucketstate, PossessionTracker detector, HingeJoint flap, ref intakeStates other_statemachine )
    {
        // First process a change in state machine

        // *****************
        // a toggles intake
        if (button_changed && button)
        {
            switch (statemachine)
            {
                case intakeStates.on:
                    statemachine = intakeStates.off;
                    break;

                case intakeStates.off:
                    statemachine = intakeStates.on;
                    other_statemachine = intakeStates.off;
                    break;
            }
        }

        // Next do what the state machine asks for
        JointMotor outmotor = my_hinge.motor;

        // While button is pressed, reverse intake
        // Also IF the bucket is up, make sure lift is down, if not, move it down
        if (button)
        {
            // First get lift position

            float lift_pos = slide_last.targetPosition.x;

            // If Bucket is below it's 50% til point or the lift is at home, just revere intake
            if ((bucket.spring.targetPosition < ((bucket_up - bucket_down) / 2f + bucket_down))   ||
                lift_pos <= slide_min )

            {
                // make sure intake is reversed
                if (outmotor.targetVelocity != -1f * intake_speed)
                {
                    outmotor.targetVelocity = -1f * intake_speed;

                    my_hinge.motor = outmotor;
                    my_hinge.useMotor = true;

                    if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                    {
                        audio_manager.Play("intake", 0.3f);
                    }
                }
            }
            else
            {
                // Otherwise we need to move the slide down
                force_slide_down = true;
            }

            // While intake is reversed, open flap
            MoveHinge(flap, flap_up, flap_speed);
        }
        // Otherwise run it if its supposed to be on
        else if (statemachine == intakeStates.on)
        {           
            if (outmotor.targetVelocity != intake_speed)
            {
                outmotor.targetVelocity = intake_speed;

                my_hinge.motor = outmotor;
                my_hinge.useMotor = true;

                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }
            }
        }

        // Otherwise turn off intake if button isn't pressed
        else 
        {
            if (outmotor.targetVelocity != 0f)
            {
                outmotor.targetVelocity = 0f;

                JointSpring myspring = my_hinge.spring;
                myspring.targetPosition = my_hinge.angle;
                my_hinge.spring = myspring;
                my_hinge.motor = outmotor;
                my_hinge.useMotor = false;

                if (audio_manager && audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Stop("intake", 0.3f);
                }
            }

            MoveHinge(flap, flap_down, flap_speed);
        }

        // Bucket states:
        // OnlyCollection implement these states while the reverse button isn't pressed
        if (!button)
        {
            switch (bucketstate)
            {
                // If down, wait for element to appear then move bucket automatically up
                case bucketStates.down:
                    // Make sure the bucket is moved down if for some reason it was up
                    MoveHinge(bucket, bucket_down, bucket_speed);

                    // If down, wait for object to appear then move up
                    if (detector.IsAnyGameElementInside())
                    {
                        bucketstate = bucketStates.movingUp;
                    }
                    break;

                // Move bucket up until reached the end
                case bucketStates.movingUp:
                    if (MoveHinge(bucket, bucket_up, bucket_speed) == bucket_up)
                    {
                        bucketstate = bucketStates.up;
                    }
                    break;

                // If up, button released moves bucket back down
                case bucketStates.up:
                    if (button_changed && !button)
                    {
                        bucketstate = bucketStates.movingDown;
                    }
                    break;

                case bucketStates.movingDown:
                    if (MoveHinge(bucket, bucket_down, bucket_speed) == bucket_down)
                    {
                        bucketstate = bucketStates.down;
                    }
                    break;
            }
        }
        else
        {
            // Otherwise while reversing, slowly move bucket up 
            MoveHinge(bucket, bucket_up, bucket_speed / 2f);
        }
       

    }
   

    // Do animations
    public void DoAnimations(bool client_mode = false)
    {

        Vector3 slide2_pos = slide0.position + 2f / 3f * (slide_last.transform.position - slide0.position);
        slide2.position = slide2_pos;

        Vector3 slide1_pos = slide0.position + 1f / 3f * (slide_last.transform.position - slide0.position);
        slide1.position = slide1_pos;   
    }

   

}



