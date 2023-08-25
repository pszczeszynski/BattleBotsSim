using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for FRC Shooter, Intertia, Roboteers...
// *******************************



public class Robot_BB_Roundhouse : RobotInterface3D
{

    private bool init_done = false;

    public float self_righting_max = 180f;
    public float self_righting_min = 0f;
    public float self_righting_speed = 40f;
    public float spinner_speed = -33000f;

    public HingeJoint spinner1;
    public HingeJoint spinner2;
    public HingeJoint arm;
    public float arm_speed = 45f;
    public float arm_min = 0f;
    public float arm_max = 180f;

    public bool spinners_on = false;
    public float downForce = 100f;

    public bool use_downforce = false;

    // Implement a dictionary??
    // protected Dictionary<string, int> userFlags = new Dictionary<string, int>();

    public void Awake()
    {
        info =
            "Button B: Spinner on/off\n" +
            "Button A: Rotate self-righting arm" +
            info;
    }

    public override void Start()
    {
        base.Start();
    }

    protected override void FixedUpdate()
    {
        if (use_downforce) { rb_body.AddForce(Vector3.down * downForce); }

        // call the base class method
        base.FixedUpdate();
    }

    private bool sound_shooting = false;
    public override void Update_Robot()
    {

        if (isKinematic)
        {
            return;
        }

        UpdateTankDriveScript();

        // ****************************
        // B button: turns spinner off-on
        if (gamepad1_b_changed && gamepad1_b)
        {
            spinners_on = !spinners_on;
        }

        if( !spinners_on && (spinner1.motor.targetVelocity !=0) )
        {
            if (spinner1)
            {
                JointMotor motor = spinner1.motor;
                motor.targetVelocity = 0f;
                spinner1.motor = motor;
            }

            if (spinner2)
            {
                JointMotor motor = spinner2.motor;
                motor.targetVelocity = 0f;
                spinner2.motor = motor;
            }

        }

        if( spinners_on && (spinner1.motor.targetVelocity == 0) )
        {
            if (spinner1)
            {
                JointMotor motor = spinner1.motor;
                motor.targetVelocity = spinner_speed;
                spinner1.motor = motor;
            }

            if (spinner2)
            {
                JointMotor motor = spinner2.motor;
                motor.targetVelocity = spinner_speed;
                spinner2.motor = motor;
            }
        }

        if (arm)
        {
            if (gamepad1_a)
            {
                MoveHinge(arm, arm_max, arm_speed);
            }
            else
            {
                MoveHinge(arm, arm_min, arm_speed);
            }
        }


        if( gamepad1_dpad_down)
        {
            use_downforce = true;
        }
        if( gamepad1_dpad_up)
        {
            use_downforce = false;
        }

    }

    // private bool qLast = false;
    void UpdateTankDriveScript()
    {
        // movement with w and s
        float movement_amount = (Input.GetKey(KeyCode.W) ? 1.0f : 0) - (Input.GetKey(KeyCode.S) ? 1.0f : 0);
        // turn with j and l
        float turn_amount = (Input.GetKey(KeyCode.A) ? 1.0f : 0) - (Input.GetKey(KeyCode.D) ? 1.0f : 0);
    
        gamepad1_left_stick_y = -movement_amount;
        gamepad1_right_stick_x = -turn_amount;

        // bool qCurr = Input.GetKey(KeyCode.Q);
        // if (qCurr && !qLast)
        // {
        //     spinners_on = !spinners_on;
        // }

        // qLast = qCurr;

        base.UpdateMovement();

    }
}