using UnityEngine;
using System.Diagnostics;

[System.Serializable]
public struct BattleBotState
{
    public Vector3 position;
    public double orientation;
}

public class Robot_BB_Flipsy : RobotInterface3D
{
    BB_RobotControllerLink robotControllerLink;

    public void Awake()
    {
        robotControllerLink = new BB_RobotControllerLink("127.0.0.1", 11115);
    }

    public override void Update_Robot()
    {
        BattleBotState state = new BattleBotState
        {
            position = transform.Find("Body").position,
            orientation = transform.Find("Body").rotation.eulerAngles[1]
        };

        // send the robot controller the current state
        string message = JsonUtility.ToJson(state);
        robotControllerLink.SendMessage(message);

        // receive latest control input from the robot controller
        RobotControllerMessage? input = robotControllerLink.Receive();


        // apply the control input to the robot
        if (input.HasValue)
        {
            UnityEngine.Debug.Log("input has a value: " + input.Value.drive_amount + ", " + input.Value.turn_amount);
            gamepad1_left_stick_y = (float)input.Value.drive_amount;
            gamepad1_right_stick_x = (float)input.Value.turn_amount;
        }
    }
}