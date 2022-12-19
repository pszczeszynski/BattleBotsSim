using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotSkin_Roboteers : RobotSkin
{
    // List of gears to rotate
    public List<RotateObject> shooter_gears = new List<RotateObject>();
    private bool gamepad1_x_old = false;


    public List<RotateObject> intake_gears = new List<RotateObject>();
    public List<float> intake_gears_speed = new List<float>();

    Robot_FRC_shooter myrobot;
    public int intake_state = 0;

    override public void InitSkin()
    {
        base.InitSkin();

        // Find all the gears to rotate with shooter
        RotateObject[] found_gears = GetComponentsInChildren<RotateObject>();

        foreach (RotateObject currgear in found_gears)
        {
            if (currgear.tag == "shooter")
            {
                shooter_gears.Add(currgear);
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

        // Initialize rotation state
        foreach (RotateObject currgear in shooter_gears)
        {
            currgear.run = true;
            currgear.speed = 600f;
        }

        myrobot = GetComponent<Robot_FRC_shooter>();
        gamepad1_x_old = false;
    }

    void Update()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        if (gamepad1_x_old != ri3d.gamepad1_x)
        {
            foreach (RotateObject currgear in shooter_gears)
            {
                currgear.speed = (ri3d.gamepad1_x) ? -1200f : 600f;
            }

            gamepad1_x_old = ri3d.gamepad1_x;
        }


        if (myrobot)
        {
            intake_state = (int)myrobot.intake_statemachine;
            DoIntakeWheels();
        }


    }

    private void DoIntakeWheels()
    {
        // Go througgh all the turret gears and set their speed
        for (int i = intake_gears.Count - 1; i >= 0; i--)
        {
            switch (intake_state)
            {
                case (int)Robot_FRC_shooter.intakeStates.off:
                    intake_gears[i].speed = 0;
                    break;
                case (int)Robot_FRC_shooter.intakeStates.onNormal:
                    intake_gears[i].speed = intake_gears_speed[i];
                    break;
                case (int)Robot_FRC_shooter.intakeStates.reverse:
                    intake_gears[i].speed = -1f * intake_gears_speed[i];
                    break;
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


    // Set the internal states based on the string
    public override void SetState(string instate)
    {
        string[] raw_states = instate.Split('|');

        // Check we got at least 2 items
        if (raw_states.Length < 2) { return; }

        // Make sure has at least 1 character
        if (raw_states[0].Length < 1) { return; }

        bool new_gamepad_x = raw_states[0].StartsWith("1");

        if (gamepad1_x_old != new_gamepad_x)
        {
            foreach (RotateObject currgear in shooter_gears)
            {
                currgear.speed = (new_gamepad_x) ? -1200f : 600f;
            }

            gamepad1_x_old = new_gamepad_x;
        }

        int.TryParse(raw_states[1], out intake_state);

        DoIntakeWheels();

    }
}
