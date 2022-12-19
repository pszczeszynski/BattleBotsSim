using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotSkin_FRC_shooter : RobotSkin
{
    // List of gears to rotate
    public List<RotateObject> shooter_gears = new List<RotateObject>();
    private bool gamepad1_x_old = false;

    override public void InitSkin()
    {
        base.InitSkin();

        // Find all the gears to rotate with shooter
        RotateObject[] found_gears = GetComponentsInChildren<RotateObject>();

        foreach( RotateObject currgear in found_gears)
        {
            if( currgear.tag == "shooter")
            {
                shooter_gears.Add(currgear);
            }
        }

        // Initialize rotation state
        foreach (RotateObject currgear in shooter_gears)
        {
            currgear.run = false;
        }

        gamepad1_x_old = false;
    }

    void Update()
    {
        if( GLOBALS.CLIENT_MODE) { return; }

        if (gamepad1_x_old != ri3d.gamepad1_x)
        {
            foreach (RotateObject currgear in shooter_gears)
            {
                currgear.run = ri3d.gamepad1_x;
            }

            gamepad1_x_old = ri3d.gamepad1_x;
        }
    }
    
    // Get a string representation of the internal states
    public override string GetState()
    {
        return (gamepad1_x_old) ? "1" : "0";
    }


    // Set the internal states based on the string
    public override void SetState(string instate)
    {
        if( instate.Length < 1) { return;  }

        bool new_gamepad_x = instate.StartsWith("1");

        if( new_gamepad_x != gamepad1_x_old)
        {
            foreach (RotateObject currgear in shooter_gears)
            {
                currgear.run = new_gamepad_x;
            }

            gamepad1_x_old = new_gamepad_x;
        }
    }
}
