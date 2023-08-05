
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface for FRC Shooter, Intertia, Roboteers...
// *******************************



public class Robot_BB_Orbitron : RobotInterface3D
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

    bool spinners_on = false;
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

        // ****************************
        // B button: turns spinner off-on
        if (gamepad1_b_changed && gamepad1_b)
        {
            spinners_on = !spinners_on;
        }

        if( !spinners_on && (spinner1.motor.targetVelocity !=0) )
        {
            JointMotor motor = spinner1.motor;
            motor.targetVelocity = 0f;
            spinner1.motor = motor;

            motor = spinner2.motor;
            motor.targetVelocity = 0f;
            spinner2.motor = motor;
        }

        if( spinners_on && (spinner1.motor.targetVelocity == 0) )
        {
            JointMotor motor = spinner1.motor;
            motor.targetVelocity = spinner_speed;
            spinner1.motor = motor;

            motor = spinner2.motor;
            motor.targetVelocity = spinner_speed;
            spinner2.motor = motor;
        }  

        if ( gamepad1_a)
        {
            MoveHinge(arm, arm_max, arm_speed);
        }
        else
        {
            MoveHinge(arm, arm_min, arm_speed);
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

}



