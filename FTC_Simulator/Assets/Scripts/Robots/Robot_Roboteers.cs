using UnityEngine;
using UnityEditor;
using System;

public class Robot_Roboteers : RobotInterface3D {

    private bool init_done = false;

    // Speeds
    [Header("FRC Bot float settings")]
    public float intake_speed = -1000f;  // Collector intake rotation speed
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
    public float ball_velocity = 6f;

    // Objects that we need to manipulate
    [Header("FRC Bot Linked Objects")]
    public HingeJoint intake_collector;
    public HingeJoint intake_collector2;
    public HingeJoint intake_collector3;
    public ballshooting ball_feeder;
    public ballshooting ball_feeder2;
    public ballshooting ball_feeder3;
    public HingeJoint ball_feeder_hinge1;
    public HingeJoint ball_feeder_hinge2;
    public HingeJoint ball_aimer;
    public ConfigurableJoint ball_aimer_cj;
    public ballshooting ball_out_forcer;
    public GameObject indicator;
    public ConfigurableJoint arm;
    public ConfigurableJoint armdummy;
    public Transform flywheel;
    public float flywheel_speed = 1f;

    [Header("Turret Options")]
    public ConfigurableJoint turret;
    public float turret_averager = 10f;
    public float turret_angle_correction = 0f;
    public Vector3 red_target;
    public Vector3 blue_target;

    [Header("Turret Visuals")]
    public HingeJoint inertia_shooter_flywheel;
    public PullyData[] inertia_shooter_pulleys;

    [Header("Animation Aid")]
    public Transform intake_spinner1_zaxis;
    public Transform intake_spinner2_zaxis;
    public Transform intake_spinner3_zaxis;



    // Other variables for internal use
    private bool a_button_last = false;
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;
    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;


    [Serializable]
    public struct PullyData { public Transform myTransform; public bool isReversed; }

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

    private bool movedLift_sound = false;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
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

        // Do turret function
        if (turret && myRobotID)
        {
            // Get angle between turret and destination
            Vector2 point_origin; // the point common between target line and turret shooting line
            Vector2 point_target; // the end point of the line to the target
            Vector2 point_robot;  // a point in the direction of the turret current shooting 

            // turret origin is center of ball_out_forcer box
            point_origin.x = ball_out_forcer.transform.position.x;
            point_origin.y = ball_out_forcer.transform.position.z;

            // Extrapolate straight line in the ball_forcer's -x direction to get the pointing direction
            Vector3 extrapolated_point = ball_out_forcer.transform.localPosition;
            extrapolated_point.x -= 1f;
            extrapolated_point = ball_out_forcer.transform.TransformPoint(extrapolated_point);
            point_robot.x = extrapolated_point.x;
            point_robot.y = extrapolated_point.z;

            // Debug.DrawLine(extrapolated_point, ball_out_forcer.transform.position, Color.green);
            // Debug.DrawLine(red_target, ball_out_forcer.transform.position);

            if (myRobotID.is_red)
            {
                point_target.x = red_target.x;
                point_target.y = red_target.z;
            }
            else
            {
                point_target.x = blue_target.x;
                point_target.y = blue_target.z;
            }

            // Get flight distance to goal
            Vector2 distance_to_goal = (point_target - point_origin);

            // Flight time to goal
            float time_to_goal = distance_to_goal.magnitude / ball_velocity;

            // Perpendicular distance vector to ball travel
            Vector2 perpendicular_vector = Vector2.Perpendicular(distance_to_goal);
            perpendicular_vector.Normalize();

            // Now get the speed of ball perpendicular to the line connecting robot to goal
            Vector2 velocity_2d;
            velocity_2d.x = rb_body.velocity.x;
            velocity_2d.y = rb_body.velocity.y;

            float speed_perpendicular = Vector2.Dot(velocity_2d, perpendicular_vector);

            // Now calculate the point_target to be offset by the velocity
            point_target -= perpendicular_vector * speed_perpendicular * time_to_goal;

            // Now get delta angle change required
            float delta_angle = Vector2.SignedAngle(point_robot - point_origin, point_target - point_origin) + turret_angle_correction;

            // Rotate the existing Rotation be the new amount.
            // Need to keep it in quaternions because the target is a quaternion, and sometimes returning euler's back results in wierd values.
            // We could keep track of final rotations ourselves, but this is easier and includes and "delay" in the turret arriving at final value.
            Quaternion rotated_quaternion = turret.targetRotation;
            Quaternion amount_to_rotate = Quaternion.Euler(-1f * delta_angle / turret_averager, 0, 0);
            rotated_quaternion *= amount_to_rotate;
            turret.targetRotation = rotated_quaternion;

        }


    }

    private bool gamepad1_x_old = false;

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
        if (isKinematic)
        {
            DoAnimations();
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
            if (audio_manager && audio_manager.IsSoundStarted("hangerraise")) { audio_manager.Stop("hangerraise", 0.3f); }
            movedLift_sound = false;
        }


        // ****************************
        // d_pad_up/down moves aimer up/down
        bool movedAim = false;
        float curr_hinge_pos = 0f;

        if (gamepad1_dpad_down)
        {
            movedAim = true;
            if (ball_aimer_cj)
            {
                curr_hinge_pos = MoveHinge(ball_aimer_cj, outhinge_min, aim_tilt_speed * turning_scaler);
            }
            else
            {
                curr_hinge_pos = MoveHinge(ball_aimer, outhinge_min, aim_tilt_speed * turning_scaler);
            }
        }
        if (gamepad1_dpad_up)
        {
            movedAim = true;
            if (ball_aimer_cj)
            {
                curr_hinge_pos = MoveHinge(ball_aimer_cj, outhinge_max, aim_tilt_speed * turning_scaler);
            }
            else
            {
                curr_hinge_pos = MoveHinge(ball_aimer, outhinge_max, aim_tilt_speed * turning_scaler);
            }
        }
        if (movedAim && (curr_hinge_pos != outhinge_min) && (curr_hinge_pos != outhinge_max))
        {
            if (audio_manager && !audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Play("adjustangle", 0);
            }
        }
        else
        {
            if (audio_manager && audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Stop("adjustangle", 0.3f);
            }
        }


        // Feed balls in while x is on
        if (gamepad1_x)
        {
            ball_feeder.speed = ball_feeder_speed;
            if (ball_feeder2) { ball_feeder2.speed = ball_feeder_speed; }
            if (ball_feeder3) { ball_feeder3.speed = ball_feeder_speed; }

            JointMotor outmotor;
            if (ball_feeder_hinge1)
            {
                outmotor = ball_feeder_hinge1.motor;
                outmotor.targetVelocity = ball_feeder_hinge_speed;
                ball_feeder_hinge1.motor = outmotor;
            }
            if (ball_feeder_hinge2)
            {
                outmotor = ball_feeder_hinge2.motor;
                outmotor.targetVelocity = ball_feeder_hinge_speed;
                ball_feeder_hinge2.motor = outmotor;
            }

        }
        else
        {
            ball_feeder.speed = -0.5f * ball_feeder_speed;

            // If intake state-machine is reversing, then also reverse the extra ball feeders.
            float reverse_speed = 0f;
            if (intake_statemachine == intakeStates.reverse)
            {
                reverse_speed = -1f * ball_feeder_speed;
            }

            if (ball_feeder2) { ball_feeder2.speed = reverse_speed; }
            if (ball_feeder3) { ball_feeder3.speed = reverse_speed; }

            // For the intake feeder wheels, run them always in reverse when not feeding
            JointMotor outmotor;
            if (ball_feeder_hinge1)
            {
                outmotor = ball_feeder_hinge1.motor;
                outmotor.targetVelocity = -1f * ball_feeder_hinge_speed;
                ball_feeder_hinge1.motor = outmotor;
            }

            if (ball_feeder_hinge2)
            {
                outmotor = ball_feeder_hinge2.motor;
                outmotor.targetVelocity = -1f * ball_feeder_hinge_speed;
                ball_feeder_hinge2.motor = outmotor;
            }
        }

        gamepad1_x_old = gamepad1_x;

        if (gamepad1_b)
        {
            if (ball_aimer_cj)
            {
                MoveHinge(ball_aimer_cj, aim_target_pos, aim_tilt_speed);
            }
            else
            {
                MoveHinge(ball_aimer, aim_target_pos, aim_tilt_speed);
            }
        }


        // ****************************
        // Intake Wheel State Machine
        if (intake_statemachine == intakeStates.onNormal)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = intake_speed;
            intake_collector.motor = outmotor;

            if (intake_collector2)
            {
                outmotor = intake_collector2.motor;
                outmotor.targetVelocity = intake_speed;
                intake_collector2.motor = outmotor;
            }

            if (intake_collector3)
            {
                outmotor = intake_collector3.motor;
                outmotor.targetVelocity = intake_speed;
                intake_collector3.motor = outmotor;
            }
        }
        if (intake_statemachine == intakeStates.off)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = 0;
            intake_collector.motor = outmotor;

            if (intake_collector2)
            {
                outmotor = intake_collector2.motor;
                outmotor.targetVelocity = 0;
                intake_collector2.motor = outmotor;
            }

            if (intake_collector3)
            {
                outmotor = intake_collector3.motor;
                outmotor.targetVelocity = 0;
                intake_collector3.motor = outmotor;
            }

        }
        if (intake_statemachine == intakeStates.reverse)
        {
            JointMotor outmotor = intake_collector.motor;
            outmotor.targetVelocity = -1f * intake_speed;
            intake_collector.motor = outmotor;
            if (intake_collector2)
            {
                outmotor = intake_collector2.motor;
                outmotor.targetVelocity = -1f * intake_speed;
                intake_collector2.motor = outmotor;
            }

            if (intake_collector3)
            {
                outmotor = intake_collector3.motor;
                outmotor.targetVelocity = -1f * intake_speed;
                intake_collector3.motor = outmotor;
            }

        }

        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
            float rotation = (ball_aimer_cj) ? ball_aimer_cj.targetRotation.eulerAngles.x : ball_aimer.spring.targetPosition;
            rotation = (rotation - outhinge_min) / (outhinge_max - outhinge_min) * 180f - 90f;

            // Now rotation is to rotate around the x-asis before the robot was moved.
            indicator_rotation.eulerAngles = new Vector3(-1f * rotation, 0, 0);

            RobotFixedUpdate();
        }
    }

    // Do animations
    public void DoAnimations()
    {
        // Add flywheel animation
        if (flywheel)
        {
            Quaternion rotation = flywheel.transform.localRotation;
            Quaternion apply_rot = Quaternion.identity;
            apply_rot.y = flywheel_speed * Time.deltaTime;
            rotation = rotation * apply_rot;
            flywheel.transform.localRotation = rotation;
        }

        if (intake_statemachine != intakeStates.off)
        {

            float speed = intake_speed * Time.deltaTime;
            if (intake_statemachine == intakeStates.reverse)
            { speed *= -1f; }


            if (intake_spinner1_zaxis)
            {
                intake_spinner1_zaxis.Rotate(0, speed, 0, Space.Self);

            }

            if (intake_spinner2_zaxis)
            {
                intake_spinner2_zaxis.Rotate(0, speed, 0, Space.Self);
            }

            if (intake_spinner3_zaxis)
            {
                intake_spinner3_zaxis.Rotate(0, speed, 0, Space.Self);
            }
        }
    }


    // Moves joint by a small delta. If reached destination, return true
    // target is a positive number, but all positions in the spring are negative.
    // I probably should have left everything negative to avoid confusion...
    /*float MoveHinge(HingeJoint hinge, float target, float speed)
    {
        // Constrain target to limits
        if (hinge.limits.max != hinge.limits.min)
        {
            if (hinge.limits.max < target) { target = hinge.limits.max; }
            if (hinge.limits.min > target) { target = hinge.limits.min; }
        }

        // Quit if we reached our target
        if (hinge.spring.targetPosition == target) { return target; }

        float temppos;
        temppos = hinge.spring.targetPosition;

        float newvalue = MoveTowards(temppos, target, temppos, Time.deltaTime * speed);
        temppos = newvalue;

        JointSpring currspring = hinge.spring;
        currspring.targetPosition = temppos;
        hinge.spring = currspring;
        return temppos;
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
    */

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


