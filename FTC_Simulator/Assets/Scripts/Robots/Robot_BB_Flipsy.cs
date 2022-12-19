using UnityEngine;
using System.Diagnostics;

public class Robot_BB_Flipsy : RobotInterface3D {
    BB_RobotControllerLink robotControllerLink = new BB_RobotControllerLink("127.0.0.1", 11115);

    public void Awake()
    {
        
    }

    public override void Update_Robot()
    {
        robotControllerLink.SendMessage("Hello");
        RobotControllerMessage? input = robotControllerLink.GetLatestControlInput();
        
        if (input.HasValue) {
            UnityEngine.Debug.Log("input has a value");
            gamepad1_left_stick_y = (float) input.Value.drive_amount;
            gamepad1_right_stick_x = (float) input.Value.turn_amount;
        }
    }
}