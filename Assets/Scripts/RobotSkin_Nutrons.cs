using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotSkin_Nutrons : RobotSkin
{
    // List of gears to rotate
    public List<RotateObject> turret_gears = new List<RotateObject>();
    public List<float> turret_gears_speed = new List<float>();

    public List<RotateObject> intake_gears = new List<RotateObject>();
    public List<float> intake_gears_speed = new List<float>();

    public bool gamepad1_x_old = false;
    public int intake_state = 0;

    Robot_FRC_shooter myrobot;

    override public void InitSkin()
    {
        base.InitSkin();

        // Find all the gears to rotate with shooter
        RotateObject[] found_gears = GetComponentsInChildren<RotateObject>();

        foreach( RotateObject currgear in found_gears)
        {
            if( currgear.tag == "turret")
            {
                turret_gears.Add(currgear);
                turret_gears_speed.Add(currgear.speed);

                // Initialize run to be always on, btu set speed to 0
                currgear.run = true;
                currgear.speed = 0f;
            }

            if (currgear.tag == "intake")
            {
                intake_gears.Add(currgear);
                intake_gears_speed.Add(currgear.speed);

                // Initialize run to be always on, btu set speed to 0
                currgear.run = true;
                currgear.speed = 0f;
            }
        }

        myrobot = GetComponent<Robot_FRC_shooter>();
        gamepad1_x_old = false;
    }

    void Update()
    {
        if( GLOBALS.CLIENT_MODE) { return;  }

        // If gamepad1_x is pressed, then speed up all wheels
        if ((gamepad1_x_old != ri3d.gamepad1_x) && (ri3d.gamepad1_x) )
        {
            Inertia_SpinUpFlywheel();
        }

        // Otherwise if gamepad1 is released, then slow them down
        if(!ri3d.gamepad1_x)
        {
            Inertia_StopFlywheel();
        }

        if( myrobot )
        {
            intake_state = (int)myrobot.intake_statemachine;
            DoIntakeWheels();
        }

        gamepad1_x_old = ri3d.gamepad1_x;
    }

    private void DoIntakeWheels()
    {
        // Go througgh all the turret gears and set their speed
        for (int i = intake_gears.Count - 1; i >= 0; i--)
        {
            switch( intake_state )
            {
                case (int) Robot_FRC_shooter.intakeStates.off:
                    intake_gears[i].speed = 0;
                    break;
                case (int) Robot_FRC_shooter.intakeStates.onNormal:
                    intake_gears[i].speed = intake_gears_speed[i];
                    break;
                case (int) Robot_FRC_shooter.intakeStates.reverse:
                    intake_gears[i].speed = -1f * intake_gears_speed[i];
                    break;
            }
        }
    }

    private void Inertia_SpinUpFlywheel()
    {
        // Go througgh all the turret gears and speed them up
        for ( int i = turret_gears.Count-1; i >=0; i--)
        {
            turret_gears[i].speed = turret_gears_speed[i];
        }
    }

    private float time_previous = 0f;
    public float Koff = 1f;
    public float Kexp = 1f;
    private void Inertia_StopFlywheel()
    {
        // Let all gears slow down
        // Do x^2 decay + fixed off amout
        // Go througgh all the turret gears and speed them up
        for (int i = turret_gears.Count - 1; i >= 0; i--)
        {
            turret_gears[i].speed = turret_gears[i].speed * Mathf.Pow(1f / (1f + Time.deltaTime * Kexp), 2f);
            turret_gears[i].speed += Time.deltaTime * Koff * ((turret_gears_speed[i] > 0) ? -1f : 1f);

            // If speed reversed direction, then make it 0
            if ((turret_gears[i].speed * turret_gears_speed[i]) < 0)
            {
                turret_gears[i].speed = 0f;
            }
        }
    }

    // Get a string representation of the internal states
    // So base state string uses ":" as seperator
    // Derived GetState addes ";" as seperator
    // Thus here we need to use something unique: will use "|" as seperator
    public override string GetState()
    {
        return (gamepad1_x_old) ? "1" : "0" + "|" + intake_state;
    }

    public string skin_state;

    // Set the internal states based on the string
    public override void SetState(string instate)
    {
        string[] raw_states = instate.Split('|');

        // Check we got at least 2 items
        if (raw_states.Length < 2) { return; }

        skin_state = raw_states[0];

        // Make sure skin_state has at least 1 character
        if( skin_state.Length < 1) { return; }

        bool new_gamepad_x = skin_state.StartsWith("1");

        // If gamepad1_x is pressed, then speed up all wheels
        if ((gamepad1_x_old != new_gamepad_x) && (new_gamepad_x))
        {
            Inertia_SpinUpFlywheel();
        }

        // Otherwise if gamepad1 is released, then slow them down
        if (!new_gamepad_x)
        {
            Inertia_StopFlywheel();
        }

        gamepad1_x_old = new_gamepad_x;

        int.TryParse(raw_states[1], out intake_state);

        DoIntakeWheels();
    }
}
