
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Greybots 973
// *******************************



public class Robot_Greybots : RobotInterface3D
{

    private bool init_done = false;

    private bool sound_shooting = false;
    private bool playing_arm = false;
    private bool playing_intake = false;

    private Sound_Controller_FRC_Shooter sound_controller;
    private AudioManager audio_manager;

    // **************************************
    // Intake stuff
    // 

    // Intake motors
    public float intake_speed = -4000f;  // Collector intake rotation speed
    public HingeJoint intake_motor1;
    public HingeJoint intake_motor2;
    public HingeJoint intake_motor3;
    public Transform intake_wheels_1;
    public Transform intake_wheels_2;
    public Transform intake_wheels_3;


    // Intake Joints
    public HingeJoint intake_axel1;
    public HingeJoint intake_axel2;


    public float intake_axel_speed = 200f;
    public float intake_axel1_home = 0f;
    public float intake_axel2_home = 0f;
    public float intake_axel1_extended = -60f;
    public float intake_axel2_extended = -30f;

    //State Machine
    public enum intakeStates : int
    {
        retracting = 0, // Moving towards the retracted position, rollers off?
        retracted,      // Retracted and all rollers off
        extending,      // extending, rollers on?
        running,        // intaking 
        reversing,      // Reversing
        off            // off
    }

    public intakeStates intake_statemachine = intakeStates.retracting;



    // **************************************8
    // Indicator
    public GameObject indicator;
    private Vector3 indicator_starting_pos;
    private Quaternion indicator_rotation;
    public MeshRenderer lightbulb;


    // *****************************************
    // Shooter
    public ballshooting_v2 ballforcer_in;
    public ballshooting_v2 ballforcer_entrance;
    public float ballforcer_in_speed = 8f;
    public float ballforcer_in_divider = 30f;
    public ballshooting_v2 ballforcer_out;
    public float ballforcer_out_curr_speed = 8f;
    public float ball_speed_max = 9f;
    public float ball_speed_min = 8f;

    // Aimer
    public HingeJoint ball_aimer;
    public float aim_target_pos = 0f; // Button B target position
    public float outhinge_min = -30f;
    public float outhinge_max = 10f;
    public float aim_tilt_speed = -20f;    // aimer tilting speed

    // *****************************************
    // Climber
    public HingeJoint climber_deploy;
    public ConfigurableJoint climber_beam;


    public float climber_deploy_angle = -90f;
    public float climber_deploy_speed = 50f;
    public float climb_beam_extended = -0.8f;
    public float climb_beam_speed = 4f;


    // *****************************************
    // Turret

    [Header("Turret Options")]
    public ConfigurableJoint turret;
    public float turret_gain = 8f;
    public float turret_angle_correction = 0f;
    public Vector3 red_target;
    public Vector3 blue_target;
    public float ball_velocity = 6f;
    public double delta_angle_display = 0f;

    public void Awake()
    {
        info =
            "D-Pad Up/Down: Aim Up/Down\n" +
            "D-Pad Left/Right: Climber extend/retract\n" +
            "Left/Right Trigger: Rotate Hooks\n" +
            "Button A: Reverse intake" +
            "Button B: Point Turret Forwards\n" +
            "Button X: Toggle intake\n" +
            "Button Y: Shoot\n" +        
            info;
        
    }

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

        if( isKinematic) { return; }

        // Do turret function
        if (turret && myRobotID && !gamepad1_b)
        {
            // Get angle between turret and destination
            Vector2 point_origin; // the point common between target line and turret shooting line
            Vector2 point_target; // the end point of the line to the target
            Vector2 point_robot;  // a point in the direction of the turret current shooting 

            // turret origin is center of ball_out_forcer box
            point_origin.x = ballforcer_out.transform.position.x;
            point_origin.y = ballforcer_out.transform.position.z;

            // Extrapolate straight line in the ball_forcer's -x direction to get the pointing direction
            Vector3 extrapolated_point = ballforcer_out.transform.localPosition;
            extrapolated_point.x -= 1f;
            extrapolated_point = ballforcer_out.transform.TransformPoint(extrapolated_point);
            point_robot.x = extrapolated_point.x;
            point_robot.y = extrapolated_point.z;

            //Debug.DrawLine(extrapolated_point, ballforcer_out.transform.position, Color.green);
            // Debug.DrawLine(red_target, ballforcer_out.transform.position);

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
            delta_angle_display = delta_angle;

            // Rotate the existing Rotation be the new amount.
            // Need to keep it in quaternions because the target is a quaternion, and sometimes returning euler's back results in wierd values.
            // We could keep track of final rotations ourselves, but this is easier and includes and "delay" in the turret arriving at final value.
            Quaternion rotated_quaternion = turret.targetRotation;
            Quaternion amount_to_rotate = Quaternion.Euler(Time.fixedDeltaTime * -1f * delta_angle * turret_gain, 0, 0);
            rotated_quaternion *= amount_to_rotate;
            turret.targetRotation = rotated_quaternion;

        }

        // If button b is pressed, then just align turret facing forward
        if (turret && myRobotID && gamepad1_b)
        {
            Quaternion rotated_quaternion = turret.targetRotation;
            float delta_angle = MyUtils.AngleWrap( 0f - rotated_quaternion.eulerAngles.x);
            Quaternion amount_to_rotate = Quaternion.Euler(Time.fixedDeltaTime * -1f * delta_angle * turret_gain, 0, 0);
            rotated_quaternion *= amount_to_rotate;
            turret.targetRotation = rotated_quaternion;
        }
    }


    public override void Start()
    {
        base.Start();
        indicator_starting_pos = indicator.transform.localPosition;

        // Intialize hinge
        ballforcer_out_curr_speed = ball_speed_max;
        JointSpring hingespring = ball_aimer.spring;
        hingespring.targetPosition = outhinge_min;
        ball_aimer.spring = hingespring;

    }

    private bool activelly_shooting = false;

    public override void Update_Robot()
    {
        // Remember some variables that we scale later during init
        if (!init_done)
        {
            // Initialize local cache variables
            sound_controller = GetComponentInChildren<Sound_Controller_FRC_Shooter>();
            audio_manager = GetComponentInChildren<AudioManager>(); // Only 1 audio manager in scene.

            init_done = true;
        }

        // If this is a client side, only do animations
        DoAnimations(true); // All animations need to be applied in single player mode as well for this bot

        if (isKinematic)
        {
            return;
        }
 

        // ****************************
        // a button: reverse intake wheels - but only while it is pressed
        if (gamepad1_a)
        {
            if( intake_statemachine == intakeStates.running)
            {
                intake_statemachine = intakeStates.reversing;
            }

        }
        else 
        {
            if (intake_statemachine == intakeStates.reversing)
            {
                intake_statemachine = intakeStates.running;
            }

        }

        // Button x toggles left intake, bottle B toggle right intake
        if( gamepad1_x && gamepad1_x_changed )
        {
            if( intake_statemachine != intakeStates.retracting && intake_statemachine != intakeStates.retracted)
            {
                intake_statemachine = intakeStates.retracting;
            }
            else
            {
                intake_statemachine = intakeStates.extending;
            }
        }



        bool play_intake = DoIntakeStateMachine(ref intake_statemachine, intake_axel1, intake_axel2, intake_motor1, intake_motor2, intake_motor3, ballforcer_in);

        if( play_intake && !playing_intake)
        { 
            if (audio_manager) { audio_manager.Play("collector", 0.3f, -1); }
            playing_intake = true;
        }
        if (!play_intake && playing_intake)
        {
            if (audio_manager) { audio_manager.Stop("collector", 0.3f); }
            playing_intake = false;
        }

        // ****************************
        // y button: shoot balls out
        if (gamepad1_y)
        {
            ballforcer_entrance.speed = ballforcer_in_speed;
            ballforcer_entrance.hard_stop = false;
            activelly_shooting = true;

            if (!sound_shooting)
            {
                if (audio_manager) { audio_manager.Play("shooting", 0.3f, -1); }
                sound_shooting = true;
            }
        }
        else
        {
            ballforcer_entrance.hard_stop = true;
            // ballforcer_entrance.speed = -0.1f * ballforcer_in_speed;
            activelly_shooting = false;

            if (sound_shooting)
            {
                if (audio_manager) { audio_manager.Stop("shooting", 0.3f); }
                sound_shooting = false;
            }
        }

        // ****************************
        // d_pad_up/down moves aimer up/down
        bool movedAim = false;
        float curr_hinge_pos = 0f;

        if (gamepad1_dpad_down)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, outhinge_min, aim_tilt_speed * turning_scaler);
            ballforcer_out_curr_speed = (1f- (curr_hinge_pos - outhinge_min) / (outhinge_max - outhinge_min)) * (ball_speed_max - ball_speed_min) + ball_speed_min;
        }
        if (gamepad1_dpad_up)
        {
            movedAim = true;
            curr_hinge_pos = MoveHinge(ball_aimer, outhinge_max, aim_tilt_speed * turning_scaler);
            ballforcer_out_curr_speed = (1f-(curr_hinge_pos - outhinge_min) / (outhinge_max - outhinge_min)) * (ball_speed_max - ball_speed_min) + ball_speed_min;

        }

        if( gamepad1_b)
        {
            ballforcer_out.speed = 4f;
        }
        else
        {
            ballforcer_out.speed = ballforcer_out_curr_speed;
        }

        bool arm_rotated = false;


        // Left trigger + right trigger rotate the main arms
        if ( gamepad1_left_trigger > 0f )
        {
            float final_value = MoveHinge(climber_deploy, climber_deploy_angle, climber_deploy_speed * gamepad1_left_trigger);
            arm_rotated = final_value != climber_deploy_angle;
        }


        // Left trigger + right trigger rotate the main beam
        if (gamepad1_right_trigger > 0f)
        {
            float final_value = MoveHinge(climber_deploy, 0f,  climber_deploy_speed * gamepad1_right_trigger);
            arm_rotated = arm_rotated || (final_value != 0);
        }

        // ****************************
        // d_pad_right extends the climb arm, d_pad_left retracts it
        if (gamepad1_dpad_right)
        {
            float currpos = MoveSlide(climber_beam, Axis.y, climb_beam_extended, climb_beam_speed * turning_scaler);

            arm_rotated = arm_rotated || (currpos != climb_beam_extended);
        }
        if (gamepad1_dpad_left)
        {
            float currpos = MoveSlide(climber_beam, Axis.y, 0f, climb_beam_speed * turning_scaler);
            arm_rotated = arm_rotated || (currpos != 0f);
        }

        if (arm_rotated && !playing_arm)
        {
            if (audio_manager) { audio_manager.Play("ArmRotate", 0.3f, -1); }
            playing_arm = true;
        }

        if (!arm_rotated && playing_arm)
        {
            if (audio_manager) { audio_manager.Stop("ArmRotate", 0.3f); }
            playing_arm = false;
        }

        // No buttons left for home position
        // if (gamepad1_b) // Seek home position 
        //{
        //    movedAim = true;
        //    curr_hinge_pos = MoveHinge(ball_aimer_cj, aim_target_pos, aim_tilt_speed);
        //}


        if (movedAim && (curr_hinge_pos != outhinge_min) && (curr_hinge_pos != outhinge_max) && (curr_hinge_pos != aim_target_pos))
        {

            if (audio_manager && !audio_manager.IsSoundStarted("adjustangle"))
            {
                audio_manager.Play("adjustangle", 0);
            }
        }
        else
        {
            if (audio_manager)
            {
                audio_manager.Stop("adjustangle", 0.3f);
            }
        }

        // Update indicator
        if (!(indicator.GetComponent<interpolation>() && indicator.GetComponent<interpolation>().enabled))
        {
            //float
            float rotation = ball_aimer.spring.targetPosition;

            rotation = (rotation - outhinge_min) / (outhinge_max - outhinge_min) * 180f - 90f;

            // Now rotation is to rotate around the x-asis before the robot was moved.
            indicator_rotation.eulerAngles = new Vector3(-1f * rotation, 0, 0);

            RobotFixedUpdate();
        }
    }



    // Intake state machine
    // Returns true if the intake is activelly moving
    public bool DoIntakeStateMachine(ref intakeStates statemachine, HingeJoint intake_axel_1, HingeJoint intake_axel_2, 
                                    HingeJoint intake_motor_1, HingeJoint intake_motor_2, HingeJoint intake_motor_3, ballshooting_v2 ballforcer)
    {
        float axel1_pos = -999f;
        float axel2_pos = -999f;
        bool intake_moving = false;

        switch (statemachine)
        {
            case intakeStates.running:
                SetHingeSpeed(intake_motor_1, intake_speed);
                SetHingeSpeed(intake_motor_2, intake_speed);
                SetHingeSpeed(intake_motor_3, intake_speed);

                if (ballforcer.disable)
                {
                    ballforcer.disable = false;
                    ballforcer.Clear();
                }

                if ( ballforcer.speed < 0 ) { 
                    ballforcer.speed *= -1f;
                    ballforcer.force_divider = ballforcer_in_divider;
                }

                if (intake_axel_1.spring.targetPosition != intake_axel1_extended)
                {
                    MoveHinge(intake_axel_1, intake_axel1_extended, intake_axel_speed);
                }
                if (intake_axel_2.spring.targetPosition != intake_axel2_extended)
                {
                    MoveHinge(intake_axel_2, intake_axel2_extended, intake_axel_speed);
                }

                intake_moving = true;
                break;

            case intakeStates.reversing:
                SetHingeSpeed(intake_motor_1, -0.5f * intake_speed);
                SetHingeSpeed(intake_motor_2, -0.5f * intake_speed);
                if (ballforcer.disable)
                {
                    ballforcer.disable = false;
                    ballforcer.Clear();
                }

                if (ballforcer.speed > 0) { 
                    ballforcer.speed *= -1f;
                    // ballforcer.force_divider = ballforcer_in_divider/10f;
                }

                // Move hinge 1 to top and hinge 2 to extended
                MoveHinge(intake_axel_1, intake_axel1_home, intake_axel_speed);
                MoveHinge(intake_axel_2, intake_axel2_extended, intake_axel_speed);

                intake_moving = true;
                break;

            case intakeStates.off:
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);
                if (activelly_shooting)
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                break;

            case intakeStates.extending:
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);

                // When extending, extend second axel half way first before moving second one
                axel2_pos = MoveHinge(intake_axel_2, intake_axel2_extended, intake_axel_speed);             
                if (Math.Abs(axel2_pos - intake_axel2_extended) < Math.Abs(axel2_pos - intake_axel2_home))
                {
                    axel1_pos = MoveHinge(intake_axel_1, intake_axel1_extended, intake_axel_speed);
                }

                if ((axel1_pos == intake_axel1_extended) &&
                    (axel2_pos == intake_axel2_extended))
                {
                    statemachine = intakeStates.running;
                }

                if (activelly_shooting)
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                intake_moving = true;
                break;

            case intakeStates.retracting:
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);

                // Retracting make sure axel1 is half way before starting axel 2
                axel1_pos = MoveHinge(intake_axel_1, intake_axel1_home, intake_axel_speed);
                
                if (Math.Abs(axel1_pos - intake_axel1_home) < Math.Abs(axel1_pos - intake_axel1_extended))
                {
                    axel2_pos = MoveHinge(intake_axel_2, intake_axel2_home, 2f * intake_axel_speed);
                }
                
                if( (axel1_pos == intake_axel1_home) &&
                    (axel2_pos == intake_axel2_home))
                {
                    statemachine = intakeStates.retracted;
                }

                if (activelly_shooting)
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                break;

            default:
                if (activelly_shooting )
                {
                    if (ballforcer.disable)
                    {
                        ballforcer.disable = false;
                        ballforcer.Clear();
                    }
                }
                else
                {
                    ballforcer.disable = true;
                }
                
                SetHingeSpeed(intake_motor_1, 0);
                SetHingeSpeed(intake_motor_2, 0);
                SetHingeSpeed(intake_motor_3, 0);
                break;
        }

        return intake_moving;
    }



    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
        if (!client_mode) { return; }

        if (intake_statemachine == intakeStates.running)
        {

            float speed = intake_speed * Time.deltaTime;
            intake_wheels_1.Rotate(-1f * speed, 0, 0, Space.Self);
            intake_wheels_2.Rotate(-1f * speed, 0, 0, Space.Self);
            intake_wheels_3.Rotate(-1f * speed, 0, 0, Space.Self);
            
         
        }

        if (intake_statemachine == intakeStates.reversing)
        {
            float speed = intake_speed * Time.deltaTime;
            intake_wheels_1.Rotate(speed, 0, 0, Space.Self);
            intake_wheels_2.Rotate(speed, 0, 0, Space.Self);
            intake_wheels_3.Rotate(speed, 0, 0, Space.Self);
        }

        lightbulb.enabled = intake_statemachine == intakeStates.running;


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



