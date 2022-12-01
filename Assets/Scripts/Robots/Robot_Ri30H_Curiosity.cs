using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_Ri30H_Curiosity : RobotInterface3D {

    public ConfigurableJoint lift_arm;
    public ConfigurableJoint lift_arm2;
    public ConfigurableJoint Bucket;
    public ConfigurableJoint BucketHinge;

    public HingeJoint collector;

    public Quaternion bucket_load_pos = new Quaternion(0, 0, 0, 0);
    public Quaternion bucket_up_pos = new Quaternion(0, 0, 0, 0);
    public Quaternion bucket_down_pos = new Quaternion(0, 0, 0, 0);

    public float lift_speed = 3f;
    public float bucket_speed = 3f;
    public float collector_speed = -1000f;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    // Do Curiosity updates
    public override void Update_Robot()
    {

        if (gamepad1_dpad_down)
        {
            if( lift_arm.targetPosition.x < 0.66 )
            {
                lift_arm.targetPosition += new Vector3(Time.deltaTime * 2f*lift_speed, 0, 0);
            }

            if (lift_arm2.targetPosition.x < 0.33)
            {
                lift_arm2.targetPosition += new Vector3(Time.deltaTime * lift_speed, 0, 0);
            }

            if (BucketHinge.targetPosition.x < 0.9)
            {
                BucketHinge.targetPosition += new Vector3(Time.deltaTime * 3f * 0.4f/0.3f*lift_speed, 0, 0);
            }

            // If all arms are near bottom, tilt the bucket down
            if(lift_arm.targetPosition.x > 0.25 &&
               lift_arm2.targetPosition.x > 0.25 &&
               BucketHinge.targetPosition.x > 0.85 )
            {
                // Set the target of the Bucket
                Bucket.targetRotation = bucket_load_pos;
            }
            else
            {
                Bucket.targetRotation = bucket_up_pos;
            }
        }

        if (gamepad1_dpad_up)
        {
            if (lift_arm.targetPosition.x > -0.30)
            {
                lift_arm.targetPosition -= new Vector3(Time.deltaTime * 2f*lift_speed, 0, 0);
            }

            if (lift_arm2.targetPosition.x > -0.15)
            {
                lift_arm2.targetPosition -= new Vector3(Time.deltaTime * lift_speed, 0, 0);
            }

            if (BucketHinge.targetPosition.x > -0.9)
            {
                BucketHinge.targetPosition -= new Vector3(Time.deltaTime * 3f * lift_speed, 0, 0);
            }

            // Make sure Bucket is in up position (unless user is pressing x)
            if (!gamepad1_x)
            {
                Bucket.targetRotation = bucket_up_pos;
            }
 
        }

        // Dump the load
        if (gamepad1_x)
        {
            Bucket.targetRotation = bucket_down_pos;
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
    }

}