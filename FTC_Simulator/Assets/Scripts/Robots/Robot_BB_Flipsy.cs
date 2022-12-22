using UnityEngine;

// for now, just send the other robot's position + orientation
// eventually: send camera images so that robot controller deduces other robot position + orientation
[System.Serializable]
public struct BattleBotState
{
    // our robot
    public Vector3 robot_position;
    public double robot_orientation;

    // other robot
    public Vector3 opponent_position;
    public double opponent_orientation;
}

public class Robot_BB_Flipsy : RobotInterface3D
{
    // our robot
    public Transform robot_body;
    // opponent's robot
    public Transform opponent_body;
    private BB_RobotControllerLink _robotControllerLink;
    public void Awake()
    {
        _robotControllerLink = new BB_RobotControllerLink("127.0.0.1", 11115);
    }

    public override void Update_Robot()
    {
        BattleBotState state = new BattleBotState
        {
            robot_position = robot_body.position,
            robot_orientation = robot_body.rotation.eulerAngles[1],
            opponent_position = opponent_body.position,
            opponent_orientation = opponent_body.rotation.eulerAngles[1]
        };

        // send the robot controller the current state
        string message = JsonUtility.ToJson(state);
        _robotControllerLink.SendMessage(message);

        // receive latest control input from the robot controller
        RobotControllerMessage? input = _robotControllerLink.Receive();

        // apply the control input to the robot
        if (input.HasValue)
        {
            UnityEngine.Debug.Log("input has a value: " + input.Value.drive_amount + ", " + input.Value.turn_amount);
            gamepad1_left_stick_y = (float)input.Value.drive_amount;
            gamepad1_right_stick_x = (float)input.Value.turn_amount;
        }
    }
}