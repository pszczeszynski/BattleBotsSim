using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_Ri30H_RoboRaider : RobotInterface3D {

    public ConfigurableJoint lift_arm;
    public ConfigurableJoint lift_arm2;
    public ConfigurableJoint Bucket;
    public ConfigurableJoint BucketHinge;

    public ConfigurableJoint mainCollectorArm;
    public ConfigurableJoint secondCollectorArm;

    public HingeJoint CollectorBody;
    public HingeJoint collector;

    public float lift_speed = 3f;
    public float bucket_speed = 3f;
    public float collector_lift_speed = 3f;
    public float collector_reach_speed = 3f;
    public float collector_speed = -1000f;

    public float collector_collect_pos = 0;
    public float collector_deposit_pos = 0;
    public float collector_max_reach = -1.2f;
    public float collector_spring = 500;

    public float bucket_deposit_pos = 0;
    public float bucket_arm_reach = -0.48f;
    public float bucket_arm_reach_scaler = 3f;

    bool state_col_down = false;

    private bool init_done = false;
    private float turn_scale_original = -1f;

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
            state_col_down = true;

            // If we reached the target, then start expanding the arm
            if (CollectorBody.spring.targetPosition < collector_collect_pos)
            {
                Vector3 target = mainCollectorArm.targetPosition;

                target.x = MoveTowards(target.x, collector_max_reach, target.x, Time.deltaTime * collector_reach_speed);
                mainCollectorArm.targetPosition = target;
                secondCollectorArm.targetPosition = target / 2f;
            }
        }

        if( state_col_down )
        { 
            // Set target position
            JointSpring col_spring = CollectorBody.spring;
      
            // If we reached our target, disable forces (let the collector fall to the ground)
            if (col_spring.targetPosition < collector_collect_pos )
            {
                col_spring.spring = 0;
            }
            else
            {
                col_spring.spring = collector_spring;
                col_spring.targetPosition -= Time.deltaTime * collector_lift_speed;
            }
            CollectorBody.spring = col_spring;
        }

        // Collector rotation up
        if (gamepad1_dpad_right)
        {
            // If we are still in the down state, than bring in the arm
            if (state_col_down && CollectorBody.spring.targetPosition < 0)
            {
                Vector3 target = mainCollectorArm.targetPosition;

                // Pull in collector
                if (target.x < 0)
                {
                    target.x = MoveTowards(target.x, 0, target.x, Time.deltaTime * collector_reach_speed);
                    mainCollectorArm.targetPosition = target;
                    secondCollectorArm.targetPosition = target / 2f;
                }
                else
                {
                    state_col_down = false;
                }
            }
            else
            {
                state_col_down = false;
            }     
        }

        if( !state_col_down )
        { 
            // Set target position
            JointSpring col_spring = CollectorBody.spring;
            col_spring.spring = collector_spring;

            col_spring.targetPosition = MoveTowards(col_spring.targetPosition, collector_deposit_pos, col_spring.targetPosition, Time.deltaTime * collector_lift_speed);
            CollectorBody.spring = col_spring;
        }

        // Move bucket up/down
        if (gamepad1_dpad_down)
        {
            lift_arm.targetPosition = new Vector3(MoveTowards(lift_arm.targetPosition.x, 0, lift_arm.targetPosition.x, Time.deltaTime * 2f * lift_speed), 0, 0);
            lift_arm2.targetPosition = lift_arm.targetPosition / 2f;

            BucketHinge.targetPosition = new Vector3(MoveTowards(BucketHinge.targetPosition.x, 0, BucketHinge.targetPosition.x, Time.deltaTime * bucket_arm_reach_scaler * lift_speed), 0, 0);

            // Make sure to reset the bucket position
            Quaternion bucket_pos = Bucket.targetRotation;
            bucket_pos.x =0;
            Bucket.targetRotation = bucket_pos;
        }

        if (gamepad1_dpad_up)
        {
            lift_arm.targetPosition = new Vector3(MoveTowards(lift_arm.targetPosition.x, 2f * bucket_arm_reach, lift_arm.targetPosition.x, Time.deltaTime * 2f * lift_speed), 0, 0);
            lift_arm2.targetPosition = lift_arm.targetPosition / 2f;

            float bucket_speed = Time.deltaTime * bucket_arm_reach_scaler * lift_speed;

            // Limit the speed of the bucket once it gets close to the top
            if (Math.Abs(BucketHinge.targetPosition.x ) > Math.Abs(bucket_arm_reach * bucket_arm_reach_scaler * 0.75f))
            {
                bucket_speed *= 0.33f;
            }

            BucketHinge.targetPosition = new Vector3(MoveTowards(BucketHinge.targetPosition.x, bucket_arm_reach * bucket_arm_reach_scaler, BucketHinge.targetPosition.x, bucket_speed), 0, 0);

           


            // Make sure Bucket is in up position (unless user is pressing x)
            if (!gamepad1_x)
            {
                // Make sure to reset the bucket position
                Quaternion bucket_pos = Bucket.targetRotation;
                bucket_pos.x = 0;
                Bucket.targetRotation = bucket_pos;
            }
 
        }
        // Dump the load
        if (gamepad1_x)
        {
            Quaternion bucket_pos = Bucket.targetRotation;
            bucket_pos.x = bucket_deposit_pos;
            Bucket.targetRotation = bucket_pos;
        }

        // Turn the sweeper on/off
        if (gamepad1_a)
        {
            JointMotor motor = collector.motor;
            motor.targetVelocity = collector_speed;
            collector.motor = motor;
 
        }

        if (gamepad1_b)
        {
            JointMotor motor = collector.motor;
            motor.targetVelocity = -collector_speed;
            collector.motor = motor;
        }

        if (gamepad1_y)
        {
            JointMotor motor = collector.motor;
            motor.targetVelocity = 0;
            collector.motor = motor;
        }

        // Scale turning as a function of how far we are extended
        turn_scale = turn_scale_original * ((collector_max_reach- mainCollectorArm.targetPosition.x) / collector_max_reach * 0.66f + 0.33f);
    }

}