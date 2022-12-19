
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for Ring Shooter, heavily based off FRC shooter
// *******************************

public class Robot_Cody : RobotInterface3D
{

    // Speeds
    public float intake_speed = -1200f;  // Collector intake rotation speed
    public HingeJoint stopbar;
    public float stopbar_closed = 0;
    public float stopbar_open = -90f;
    public float stopbar_open_reverse = 90f;
    public float stopbar_speed = 100f;

    public HingeJoint intake_collector;

    public HingeJoint bucket;
    public float bucket_start = 0f;
    public float bucket_speed = 200f;

    public HingeJoint arm;
    public float arm_speed = 10f;
    public float arm_start = 0f;

    public ConfigurableJoint slide_last;
    public Transform slide5;
    public Transform slide4;
    public ConfigurableJoint slide3;
    public Transform slide2;
    public float slide_max = 1.6f;
    public float slide_min = 0f;
    public float slide_speed = 1f;


    public float arm_min = -60f;
    public float arm_deployed = 80f;
    public float arm_max = 160f;
    public float arm_speed_start = 30f;
    public float arm_speed_level = 10f;
    public float bucket_min = -90f;
    public float bucket_max = 90f;

    [Header("Turret Options")]
    public HingeJoint turret;
    public float turret_averager = 10f;
    public float turret_angle_correction = 0f;
    public float turret_speed = 100f;
    public float turret_distance_offset = 1.42f;
    public Transform team_hub;
    public Transform shared_hub;
    public Transform target_hub;
    public MeshRenderer lightbulb;


    public float[,] arm_bucket_map = new float[10, 2]
    {
        {-60f,0 },
        {-50f, -18f },
        {-40f, -28f },
        {-30f, -35f },
        {70f, 60f },
        {80f, 60f },
        {90f, 50f },
        {110f,30f },
        {130f, 10 },
        {160f, -15f }
    };

    public float[,] arm_slide_map = new float[9, 2]
    {
        {-60f, 0f },
        {-10f, 0.42f },
        {70f, 0.42f },
        {80f, 0.32f },
        {90f, 0.18f },
        {100f, 0.12f },
        {120f, 0f },
        {140f, -0.05f },
        {160f, -0.06f }
    };


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
        rotatingout,
        extending,
        deployed,
        opening,
        retracting,
        rotatingin,
        returning,
        spawn
    }
    armStates arm_state = armStates.spawn;

    private float arm_deployed_saved = 0f;

    public void Awake()
    {
        info =
            "Gamepad Up/Down: Move Arm up/down when deployed" +
            "\nGamepad Left/Right: Move arm extended position towards/away from target" +
            "\nButton A: Toggle intake, reverse intake while held" + 
            "\n          Arm needs to be retracted for intake to collect" + 
            "\nButton B: Deploy arm" +
            "\nButton Y: toggle auto-aim" +
            "\nButton X: Retract arm" +
            "\nLeft trigger: Switch to team hub, enable auto-aim if disabled" +
            "\nRight trigger: Switch to shared hub, enable auto-aim if disabled" + 
            info;
    }


    public override void Start()
    {
        base.Start();
        arm_deployed_saved = arm_deployed;

    }


    public override void Init_Robot()
    {
        // Find our Hubs
        if( !myRobotID) { return; }

        // Initialize local cache variables
        audio_manager = GetComponentInChildren<AudioManager>(); // Only 1 audio manager in scene.

        string team = "Red";
        if( !myRobotID.is_red)
        {
            team = "Blue";
        }
        GameObject team_hub_go = GameObject.Find("AllianceHub" + team);
        if( team_hub_go )
        {
            team_hub = team_hub_go.transform;
        }

        GameObject shared_hub_go = GameObject.Find("SharedHub" + team);
        if(shared_hub_go)
        {
            shared_hub = shared_hub_go.transform;
        }

        target_hub = team_hub;

        turret_distance_offset = 1.42f;
    }

    public double delta_angle_display = 0f;

    public float turret_target = 0f;
    public float turret_curr_position = 0f;
    public float turret_distance = 0f;
    public float loop_gain = 1f;
    public float loop_slew_limit = 8f;


    public override void RobotFixedUpdate()
    {
        if (!rb_body || !myRobotID ||  isKinematic || !team_hub || !shared_hub || !turret)
        { return; }

        if( !engage_auto)
        {
            return;
        }

        // Get angle between turret and destination
        Vector2 point_origin; // the point common between target line and turret shooting line
        Vector2 point_target; // the end point of the line to the target
        Vector2 point_robot;  // a point in the direction of the turret current shooting 

        // turret origin is the turrets center
        point_origin.x = turret.transform.position.x;
        point_origin.y = turret.transform.position.z;

        point_target.x = target_hub.position.x;
        point_target.y = target_hub.position.z;


        // Extrapolate straight line in the ball_forcer's -x direction to get the pointing direction
        point_robot = point_origin;
        point_robot.x -= turret.transform.right.x;
        point_robot.y -= turret.transform.right.z;

        turret_distance = (point_target - point_origin).magnitude;

        //Debug.DrawRay(new Vector3(point_origin.x, 1f, point_origin.y), -1f*turret.transform.right);
        //Debug.DrawLine(new Vector3(point_target.x, 1f, point_target.y), new Vector3(point_origin.x, 1f, point_origin.y));

        // Now get delta angle change required
        float delta_angle = Vector2.SignedAngle(point_target - point_origin, point_robot - point_origin) + turret_angle_correction;
        delta_angle_display = delta_angle;

        delta_angle = loop_gain * (float)MyUtils.AngleWrap(delta_angle) / turret_averager; ;
        if (delta_angle < -1f *loop_slew_limit)
        {
            delta_angle = -1f * loop_slew_limit;
        }
        if (delta_angle > loop_slew_limit)
        {
            delta_angle = loop_slew_limit;
        }

        // Rotate the existing Rotation to be the new amount.
        // turret_target = turret_curr_position +  delta_angle;
        turret_target = turret.spring.targetPosition +  delta_angle;
        if ( turret_target > 180f)  { turret_target = 180f; }
        if( turret_target < -180f) { turret_target = -180f; }
    }

    private bool sound_shooting = false;
    public float left_trigger = 0f;
    public float right_trigger = 0f;
    public bool engage_auto = true;
    public string arm_state_string = "";
    public float arm_pos = 0f;
    public Vector3 arm_axis = new Vector3(0, 0, 0);
    public float arm_target = 0f;
    public override void Update_Robot()
    {
  
        arm_state_string = arm_state.ToString();
        arm_pos = Quaternion.Angle(arm.connectedBody.rotation, arm.transform.rotation);



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

        if ((intake_statemachine == intakeStates.on) && !gamepad1_a)
        {
            JointMotor outmotor = intake_collector.motor;

            // Make sure intake is on if arm is in correct state
            if (arm_state == armStates.home)
            {               
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
            else
            {
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
        }

        // Reversing
        if (gamepad1_a)
        {
            // make sure intake is reversed
            JointMotor outmotor = intake_collector.motor;
            if (outmotor.targetVelocity != -1f * intake_speed)
            {
                outmotor.targetVelocity = -1f * intake_speed;

                intake_collector.motor = outmotor;
                intake_collector.useMotor = true;

                if (audio_manager && !audio_manager.IsSoundStarted("intake"))
                {
                    audio_manager.Play("intake", 0.3f);
                }
            }
        }

        // Check to turn off intake
        if (!gamepad1_a && (intake_statemachine == intakeStates.off))
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
            switch(intake_statemachine)
            {
                case intakeStates.on:
                    intake_statemachine = intakeStates.off;
                    break;

                case intakeStates.off:
                    intake_statemachine = intakeStates.on;
                    break;

                // Not doing reverse at this moment so this is not required
                case intakeStates.reverse:
                    intake_statemachine = intakeStates.on;
                    break;

            }
        }

        if (gamepad1_y_changed && gamepad1_y)
        {
            engage_auto = !engage_auto;         
        }

        // *******************************************
        // Switch between target and team hub
        left_trigger = gamepad1_left_trigger;
        right_trigger = gamepad1_right_trigger;

        if ( gamepad1_left_trigger > 0.5f)
        {
            target_hub = team_hub;
            engage_auto = true;
            arm_deployed_saved = arm_deployed;
        }

        if (gamepad1_right_trigger > 0.5f)
        {
            target_hub = shared_hub;
            engage_auto = true;
            arm_deployed_saved = arm_max - 30f;
        }

        if( lightbulb.enabled != engage_auto)
        {
            lightbulb.enabled = engage_auto;
        }

        // ****************************
        // Deal with arm-state
        bool play_arm_sound = false;
        bool done_stop = false;
        bool turret_arrived = false;

        switch (arm_state)
        {
            case armStates.closing:
                done_stop = MoveHinge(stopbar, stopbar_closed, stopbar_speed) == stopbar_closed;

                if (done_stop)
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
                if (MoveArm(arm_deployed_saved))
                {
                    arm_state = armStates.rotatingout;
                }

                play_arm_sound = true;
                break;

            case armStates.rotatingout:
                if (Math.Abs(turret.spring.targetPosition - turret_target) < 2f)
                {
                    arm_state = armStates.extending;
                }
                break;

            case armStates.extending:
                if (CodeMoveSlide(turret_distance - turret_distance_offset, slide_speed))
                {
                    arm_state = armStates.deployed;
                }
                break;

            case armStates.deployed:
                if (gamepad1_dpad_down)
                {
                    MoveArm(arm_max, arm_speed_level);
                    play_arm_sound = true;
                }
                else if (gamepad1_dpad_up)
                {
                    MoveArm(arm_deployed, arm_speed_level);
                    play_arm_sound = true;
                }

                break;

            case armStates.opening:
                done_stop = MoveHinge(stopbar, (gamepad1_x) ? stopbar_open_reverse : stopbar_open, stopbar_speed) == stopbar_open;

                if (done_stop && !gamepad1_x)
                {
                    arm_state = armStates.retracting;
                }

                if (audio_manager && !audio_manager.IsSoundStarted("arm"))
                {
                    audio_manager.Play("arm", 0.5f);
                }
                play_arm_sound = true;

                break;

            case armStates.retracting:
                MoveArm(-10f);
                if (CodeMoveSlide(0, slide_speed))
                {
                    arm_state = armStates.rotatingin;
                }
                break;

            case armStates.rotatingin:
                MoveArm(-10f);
                if ((turret_curr_position = MoveHinge(turret, 0, turret_speed)) == 0)
                {
                    arm_state = armStates.returning;
                }
                break;

            case armStates.returning:
                if (MoveArm(arm_min))
                {
                    arm_state = armStates.home;
                }

                play_arm_sound = true;
                break;

        }

        // Rotate the turret
        switch (arm_state)
        {
            case armStates.deploying:
            case armStates.rotatingout:
            case armStates.extending:
            case armStates.deployed:
            case armStates.opening:
                // Only allow movement of turret if the Bucket is clear of the home position
                if (Quaternion.Angle(arm.connectedBody.rotation, arm.transform.rotation) > 40f)
                {
                    turret_curr_position = MoveHinge(turret, turret_target, turret_speed);
                }
             
                break;
        }

        switch (arm_state)
        {
            // case armStates.extending:
            case armStates.deployed:
            case armStates.opening:
                CodeMoveSlide(turret_distance - turret_distance_offset, slide_speed, true);
                break;
        }

        // Offset adjustment
        if (gamepad1_dpad_left)
        {
            turret_distance_offset = MoveTowards(turret_distance_offset, 2f, turret_distance_offset, Time.deltaTime * slide_speed * 0.5f * turning_scaler);
        }

        if (gamepad1_dpad_right)
        {
            turret_distance_offset = MoveTowards(turret_distance_offset, 0.9f, turret_distance_offset, Time.deltaTime * slide_speed * 0.5f * turning_scaler);
        }


        // Global overrides for arm
        if (gamepad1_x_changed && gamepad1_x)
        {
            if( arm_state == armStates.deployed)
            {
                arm_deployed_saved = arm.spring.targetPosition;
            }

            arm_state = armStates.opening;
        }

        if (gamepad1_b_changed && gamepad1_b)
        {
            arm_state = armStates.closing;
        }



        // d_pad_up/down moves forearm up/down
        if (!play_arm_sound && audio_manager && audio_manager.IsSoundStarted("arm"))
        {
            audio_manager.Stop("arm", 0.5f);
        }


       
   



    }

    public bool MoveArm(float target, float myarm_speed = -1f)
    { 

        // Default to fast deploy speed
        if(myarm_speed < 0 )
        {
            myarm_speed = arm_speed_start;
        }

        if( !arm ) { return false; }
        float curr_position = MoveHinge(arm, target, myarm_speed * turning_scaler);
        MoveHinge(bucket, MyUtils.GetInterpolatedValue(arm.spring.targetPosition, arm_bucket_map), 10f * myarm_speed);

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

    public bool CodeMoveSlide(float target, float speed, bool correct_for_height = false)
    {
        // Correct target for bucket extension
        if (correct_for_height)
        {
            target += MyUtils.GetInterpolatedValue(arm.spring.targetPosition, arm_slide_map);
        }

        if ( target < slide_min)
        {
            target = slide_min;
        }

        if( target > slide_max)
        {
            target = slide_max;
        }

        Vector3 curr_position = slide_last.targetPosition;

        // Slow down speed as we get close
        if( (speed > slide_speed * 0.5f) && (Math.Abs(curr_position.x - target) < 0.1f) )
        {
            speed = 0.5f * slide_speed + (speed - 0.5f * slide_speed) * Math.Abs(curr_position.x - target) / 0.1f;
        }

        curr_position.x = MoveSlide(slide_last, Axis.x, target, speed);
        MoveSlide(slide3, Axis.x, curr_position.x * 2f / 5f, speed * 2f/5f);


        //    slide_last.targetPosition = curr_position;
        //slide5.targetPosition = curr_position * 4f / 5f;
        //slide4.targetPosition = curr_position * 3f / 5f;
        //    slide3.targetPosition = curr_position * 2f / 5f;
        //slide2.targetPosition = curr_position * 1f / 5f;

        if(Math.Abs(curr_position.x - target) < 0.01f ) { return true; }
        return false;
    }

    // Do animations
    public void DoAnimations(bool client_mode = false)
    {
        // Adjust slide 2,4,5 position to match main slide
        float delta_distance = -1f * (slide_last.transform.position - turret.transform.position).magnitude;

        Vector3 slide5_pos = slide5.localPosition;
        slide5_pos.x = delta_distance * 4f / 5f;
        slide5.localPosition = slide5_pos;

        Vector3 slide4_pos = slide4.localPosition;
        slide4_pos.x = delta_distance * 3f / 5f;
        slide4.localPosition = slide4_pos;

        Vector3 slide2_pos = slide2.localPosition;
        slide2_pos.x = delta_distance / 5f;
        slide2.localPosition = slide2_pos;
    }


    // ********* State Sync for server/client
    // Make sure the string is a small, mainly numerical value (e.g. 1:2:0.11...) so that it doesn't increase packet length much, and won't restrict compression.
    override public string GetStates()
    {
        // Save states that we want to synchronize over internet
        string outstring = base.GetStates();

        // Add the extended arm position
        outstring += ";" + ((engage_auto) ? "1" : "0"); // different from base seperator

        // Nothing else to add now
        return outstring;
    }

    // Read in the states that were turned into a string representation from GetStates()
    float client_slider_pos = 0f;
    override public void SetStates(string instring)
    {
        string[] allstates = instring.Split(';');

        base.SetStates(allstates[0]);
        if(allstates.Length > 1)
        {
            engage_auto = (allstates[1][0] == '1') ? true : false;

            lightbulb.enabled = engage_auto;
        }
    }
}



