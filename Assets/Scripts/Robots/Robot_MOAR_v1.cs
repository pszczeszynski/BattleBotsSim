using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_MOAR_v1 : RobotInterface3D {

    // ****** Collector items ****
    public ConfigurableJoint collector_slide_1;   // smallest travel distance
    public ConfigurableJoint collector_slide_2;
    public ConfigurableJoint collector_slide_3;
    public HingeJoint collector_hinge;
    public HingeJoint collector_sweeper;
    public HingeJoint collector_sweeper2;

    // Collector variables - reference the smallest travel distance/arm when in doubt
    public float collector_slide_startx = 0; 
    public float collector_slide_stopx = 1f;
    public float collector_slide_speed = 1f;
    public float collector_sweeper_speed = -1000f;
    public float collector_hinge_start = 0;
    public float collector_hinge_down = 0;
    public float collector_hinge_speed = 1f;

    // ****** left bucket Arm Items *******
    public HingeJoint bucket_left_slide_1;   // smallest travel distance
    public ConfigurableJoint bucket_left_slide_2;
    public ConfigurableJoint bucket_left_slide_3;
    public HingeJoint bucket_left_hinge;

    // ****** right bucket Arm Items *******
    public HingeJoint bucket_right_slide_1;   // smallest travel distance
    public ConfigurableJoint bucket_right_slide_2;
    public ConfigurableJoint bucket_right_slide_3;
    public HingeJoint bucket_right_hinge;

    // Bucket arm variables
    public float bucket_slide_startx = 0;
    public float bucket_slide_stopx = 1f;
    public float bucket_slide_speed = 1f;
    public float bucket_left_hinge_start = 0;
    public float bucket_right_hinge_start = 0;
    public float bucket_hinge_down = 0;
    public float bucket_hinge_upmax = 0;
    public float bucket_hinge_speed = 1f;
    public float bucket_tilt_down = 60f;
    public float bucket_tilt_up = -60f;

    // Tracking info
    public float bucket_col_tracking_slope = 1.5f;
    public float bucket_col_tracking_offset = 0.1578f;

    
    private bool init_done = false;
    private float turn_scale_original = -1f;

    private bool state_collector_down = false;
    private bool button_y = false;

    private enum sweeper_states
    {
        IN,
        OFF,
        OUT
    };

    private sweeper_states sweeper_state = sweeper_states.OFF;
    private bool button_x = false;
    private bool button_a = false;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }
    // Do Curiosity updates
    public override void Update_Robot()
    {
        // Initialize variables
        if (!init_done)
        {
            init_done = true;
            turn_scale_original = turn_scale;
        }

        // Collector rotation down
        if (gamepad1_dpad_left)
        {
            Vector3 target = collector_slide_1.targetPosition;

            if ( target.x > collector_slide_stopx )
            {
                target.x -= Time.deltaTime * collector_slide_speed;

                if( target.x < collector_slide_stopx)
                { target.x = collector_slide_stopx; }

                collector_slide_1.targetPosition = target;
                collector_slide_2.targetPosition = target * 2f;
                collector_slide_3.targetPosition = target * 3f;
            }
        }

  
        // Collector rotation up
        if (gamepad1_dpad_right)
        {
            Vector3 target = collector_slide_1.targetPosition;

            if (target.x < 0)
            {
                target.x += Time.deltaTime * collector_slide_speed;

                if (target.x > 0)
                { target.x = 0; }

                collector_slide_1.targetPosition = target;
                collector_slide_2.targetPosition = target * 2f;
                collector_slide_3.targetPosition = target * 3f;
            }
        }

       
        // Arm motion will be bumpers for slides, dpad up/down for rotation
        if (gamepad1_left_trigger > 0.1)
        {
            Vector3 target = bucket_left_slide_2.targetPosition;
            target.x = MoveTowards(bucket_slide_startx, bucket_slide_stopx, bucket_left_slide_2.targetPosition.x, Time.deltaTime * bucket_slide_speed * gamepad1_left_trigger);
            bucket_left_slide_2.targetPosition = target;
            bucket_left_slide_3.targetPosition = target*2f;
        }

        if (gamepad1_left_bumper)
        {
            Vector3 target = bucket_left_slide_2.targetPosition;
            target.x = MoveTowards(bucket_slide_stopx, bucket_slide_startx, bucket_left_slide_2.targetPosition.x, Time.deltaTime * bucket_slide_speed );
            bucket_left_slide_2.targetPosition = target;
            bucket_left_slide_3.targetPosition = target * 2f;
        }

        if (gamepad1_dpad_up )
        {
            JointSpring spring = bucket_left_slide_1.spring;
            float target = MoveTowards(bucket_hinge_down+bucket_left_hinge_start, bucket_hinge_upmax + bucket_left_hinge_start, spring.targetPosition, Time.deltaTime * bucket_hinge_speed );
            spring.targetPosition = target;
            bucket_left_slide_1.spring = spring;

            // Set bucket tilt
            JointSpring bucket_spring = bucket_left_hinge.spring;
            bucket_spring.targetPosition = (target - bucket_hinge_down) / (bucket_hinge_upmax - bucket_hinge_down) * (bucket_tilt_up - bucket_tilt_down) + bucket_tilt_down;
            bucket_left_hinge.spring = bucket_spring;
        }
        if (gamepad1_dpad_down)
        {
            JointSpring spring = bucket_left_slide_1.spring;
            float target = MoveTowards(bucket_hinge_upmax + bucket_left_hinge_start, bucket_hinge_down + bucket_left_hinge_start, spring.targetPosition, Time.deltaTime * bucket_hinge_speed);
            spring.targetPosition = target;
            bucket_left_slide_1.spring = spring;

            // Set bucket tilt
            JointSpring bucket_spring = bucket_left_hinge.spring;
            bucket_spring.targetPosition = (target - bucket_hinge_down) / (bucket_hinge_upmax - bucket_hinge_down) * (bucket_tilt_up - bucket_tilt_down) + bucket_tilt_down;
            bucket_left_hinge.spring = bucket_spring;
        }

        // Turn sweeper off/on
        if (gamepad1_x)
        {
            if(button_x== false)
            {
                if (sweeper_state == sweeper_states.IN)
                {
                    sweeper_state = sweeper_states.OFF;
                }
                else
                    sweeper_state = sweeper_states.IN;
            }

            button_x = true;
        }
        else
        {
            button_x = false;
        }

        // Turn the sweeper on/off
        if (gamepad1_a)
        { 
            if (button_a == false)
            {
                if (sweeper_state == sweeper_states.OUT)
                {
                    sweeper_state = sweeper_states.OFF;
                }
                else
                    sweeper_state = sweeper_states.OUT;
            }

            button_a = true;
        }
        else
        {
            button_a = false;
        }


        JointMotor sweeper_motor = collector_sweeper.motor;

        switch (sweeper_state)
        {
            case sweeper_states.IN:
                sweeper_motor.targetVelocity = collector_sweeper_speed;
                break;
            case sweeper_states.OUT:
                sweeper_motor.targetVelocity = -collector_sweeper_speed;
                break;
            default:
                sweeper_motor.targetVelocity =0;
                break;
        }

        collector_sweeper.motor = sweeper_motor;
        collector_sweeper2.motor = sweeper_motor;

        if (gamepad1_b)
        {
            // Set bucket tilt to deliver
            JointSpring bucket_spring = bucket_left_hinge.spring;
            bucket_spring.targetPosition = MoveTowards(bucket_spring.targetPosition, bucket_tilt_down, bucket_spring.targetPosition, Time.deltaTime * bucket_hinge_speed * 10f );
            bucket_left_hinge.spring = bucket_spring;
        }

        // Toggle collector state when Y is pressed down
        if (gamepad1_y)  {
            if( !button_y) {state_collector_down = !state_collector_down; }
            button_y = true;
        }
        else {
            button_y = false;
        }

        // Execute on collector state
        if( state_collector_down )
        {
            JointSpring joint = collector_hinge.spring;
            joint.targetPosition = MoveTowards(collector_hinge_start, collector_hinge_down, collector_hinge.spring.targetPosition, Time.deltaTime * collector_hinge_speed);
            collector_hinge.spring = joint;
        }

        if (!state_collector_down)
        {
            JointSpring joint = collector_hinge.spring;
            joint.targetPosition = MoveTowards(collector_hinge_down, collector_hinge_start, collector_hinge.spring.targetPosition, Time.deltaTime * collector_hinge_speed);
            collector_hinge.spring = joint;
        }

        // Set arm extention 
        // From the halfway rotation point to the 1/4 rotation, we will
        // linearly extend the arm to align with the bucket. Once we are at 1/4, we will follow the bucket
        float bucket_left_pos = bucket_left_slide_1.spring.targetPosition;
        float bucket_percent = (bucket_left_pos - bucket_hinge_down) / (bucket_hinge_upmax - bucket_hinge_down);
        if( bucket_percent < 0.3 )
        {
            float target_position = collector_slide_1.targetPosition.x * bucket_col_tracking_slope + bucket_col_tracking_offset;

            if(gamepad1_dpad_down)
            {
                target_position += 0.05f;
            }

            Vector3 target = bucket_left_slide_2.targetPosition;
            target.x = MoveTowards(bucket_left_slide_2.targetPosition.x, target_position, bucket_left_slide_2.targetPosition.x, Time.deltaTime * bucket_slide_speed);
            bucket_left_slide_2.targetPosition = target;
            bucket_left_slide_3.targetPosition = target * 2f;
        }

        // Scale turning as a function of how far we are extended
        turn_scale = turn_scale_original * ((collector_slide_2.targetPosition.x + 1.8f)/1.8f * 0.66f + 0.33f);
    }

}