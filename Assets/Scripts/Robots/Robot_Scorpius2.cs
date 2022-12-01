using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Scorpius 2
// *******************************



public class Robot_Scorpius2 : RobotInterface3D {

    public ConfigurableJoint lift_arm;
    public ConfigurableJoint lift_arm2;
    public ConfigurableJoint Bucket;
    public ConfigurableJoint BucketHinge;

    public ConfigurableJoint mainCollectorArm;
    public ConfigurableJoint secondCollectorArm;

    public HingeJoint CollectorBody;
    public HingeJoint collector;

    public float lift_speed = 3f;
    private float bucket_speed = 3f;
    public float bucket_rot_speed = 3f;
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

    public Transform Robot_position;
    public bool enable_orbit = true;
    public float target_x_pos = 96.0f;
    public float target_z_pos = -165.3f;
    public float target_angle = 0f;
    // Pointing up: y = 180 down=0
    // pointing right: y = -90/270 left: 90
    // Z = horizontal. Right is positive
    // X = vertical.

    bool gamepad_a_previous = false;
    bool gamepad_b_previous = false;
    bool gamepad_y_previous = false;

    enum Collector_States
    {
        off,
        on,
        reverse
    };
    Collector_States collector_state = Collector_States.off;

    enum Delivery_States
    {
        off,
        raising_arm,
        rotating_bucket,
        waiting,
        lowering_arm,
    };
    Delivery_States auto_mode = Delivery_States.off;
    long time_wait_start = 0;
    public long auto_dumping_wait = 1000;

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


        // ** AUTO state variable change logic ****
        // Execute on the automatic delivery
        switch (auto_mode)
        {
            case Delivery_States.off:
                // Do nothing
                break;

            case Delivery_States.raising_arm:
                // This is triggered by y button.
                // Keep raising until arm reaches target
                if(Math.Abs(lift_arm2.targetPosition.x) > Math.Abs(0.999f*bucket_arm_reach) &&
                   Math.Abs(BucketHinge.targetPosition.x) > Math.Abs(0.999f* bucket_arm_reach * bucket_arm_reach_scaler))
                {
                    auto_mode = Delivery_States.rotating_bucket;
                }
                break;

            case Delivery_States.rotating_bucket:
                // Here we will just make sure the target position is programmed in and rely on
                // the wait state to give it time
                if (Bucket.targetRotation.x == bucket_deposit_pos )
                {
                    auto_mode = Delivery_States.waiting;
                    time_wait_start = MyUtils.GetTimeMillis();
                }
                break;

            case Delivery_States.waiting:
                if(MyUtils.GetTimeMillis() - time_wait_start > auto_dumping_wait)
                {
                    auto_mode = Delivery_States.lowering_arm;
                }
                break;

            case Delivery_States.lowering_arm:
                if(Math.Abs(lift_arm2.targetPosition.x) < 0.01f)
                {
                    auto_mode = Delivery_States.off;
                }
                break;

            default:
                break;
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
        if (gamepad1_dpad_down || auto_mode == Delivery_States.lowering_arm)
        {
            lift_arm.targetPosition = new Vector3(MoveTowards(lift_arm.targetPosition.x, 0, lift_arm.targetPosition.x, Time.deltaTime * 2f * lift_speed), 0, 0);
            lift_arm2.targetPosition = lift_arm.targetPosition / 2f;

            BucketHinge.targetPosition = new Vector3(MoveTowards(BucketHinge.targetPosition.x, 0, BucketHinge.targetPosition.x, Time.deltaTime * bucket_arm_reach_scaler * lift_speed), 0, 0);

            // Make sure to reset the bucket position
            Quaternion bucket_pos = Bucket.targetRotation;
            bucket_pos.x =0;
            Bucket.targetRotation = bucket_pos;
        }

        if (gamepad1_dpad_up || auto_mode == Delivery_States.raising_arm)
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
        if (gamepad1_x || auto_mode == Delivery_States.rotating_bucket)
        {
            Quaternion bucket_pos = Bucket.targetRotation;
            bucket_pos.x += Time.deltaTime * bucket_rot_speed * bucket_deposit_pos;
            if((bucket_deposit_pos > 0 && bucket_pos.x >= bucket_deposit_pos) ||
               (bucket_deposit_pos < 0 && bucket_pos.x <= bucket_deposit_pos))
            { bucket_pos.x = bucket_deposit_pos;  }

            Bucket.targetRotation = bucket_pos;
        }

        // Determine collector state
        if (gamepad1_b)
        {
            if (!gamepad_b_previous)
            {
                if( collector_state != Collector_States.off)
                {
                    collector_state = Collector_States.off;
                }
                else
                {
                    collector_state = Collector_States.reverse;
                }
            }
            gamepad_b_previous = true;
        }
        else
        {
            gamepad_b_previous = false;
        }

        if (gamepad1_a)
        {
            if (!gamepad_a_previous)
            {
                if (collector_state != Collector_States.off)
                {
                    collector_state = Collector_States.off;
                }
                else
                {
                    collector_state = Collector_States.on;
                }
            }
            gamepad_a_previous = true;
        }
        else
        {
            gamepad_a_previous = false;
        }

        float motor_speed = 0f;

        switch (collector_state)
        {
            case Collector_States.off:
                break;
            case Collector_States.on:
                motor_speed = collector_speed;
                break;
            case Collector_States.reverse:
                motor_speed = -collector_speed;
                break;
        }
        JointMotor motor = collector.motor;
        motor.targetVelocity = motor_speed;
        collector.motor = motor;



        // Scale turning as a function of how far we are extended
        turn_scale = turn_scale_original * ((collector_max_reach- mainCollectorArm.targetPosition.x) / collector_max_reach * 0.5f + 0.5f);

        // Calculate target angle
        Vector3 pos_robot = Robot_position.position;
        float deltax = target_x_pos - pos_robot.x;
        float deltay = target_z_pos - pos_robot.z;
        float angle_target = - (float) (Math.Atan2(deltay, deltax) * 180f/Math.PI)-90f;

        float robot_angle = Robot_position.eulerAngles.y;
        float delta_angle = angle_target - robot_angle;

        while(delta_angle > 180f)
        {
            delta_angle -= 360f;
        }

        while (delta_angle < -180f)
        {
            delta_angle += 360f;
        }

        target_angle = delta_angle;

        // Auto-Mode
        if (gamepad1_y)
        {
            if (!gamepad_y_previous)
            {
                if (auto_mode != Delivery_States.off)
                {
                    auto_mode = Delivery_States.off;
                }
                else
                {
                    auto_mode = Delivery_States.raising_arm;
                }
            }
            gamepad_y_previous = true;
        }
        else
        {
            gamepad_y_previous = false;
        }

        // Turn the robot towards the goal (orbit mode)
        if (enable_orbit && 
            (auto_mode == Delivery_States.raising_arm ||
            auto_mode == Delivery_States.rotating_bucket ||
            auto_mode == Delivery_States.waiting))
        {
            turning_overide = delta_angle / 10f;
            if( turning_overide > 1f)
            {
                turning_overide = 1f;
            }

            if( turning_overide < -1f )
            {
                turning_overide = -1f;
            }
        }
        else
        {
            turning_overide = 0f;
        }
    }

}