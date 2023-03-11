using System.Collections.Generic;
using UnityEngine;
using System.Linq;

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
    public int SPINNER_SPEED = 36000;
    // our robot
    public Transform robot_body;
    // opponent's robot
    public Transform opponent_body;
    public HingeJoint spinner1;
    public HingeJoint spinner2;
    private BB_RobotControllerLink _robotControllerLink;

    // cached enemies, sorted by distance
    private List<Transform> _enemyRobotBodies = new List<Transform> { };
    // search for enemies every so often
    private const float ENEMY_SEARCH_INTERVAL_SECONDS = 2.0f;

    private bool _spinnersOn;

    public void Awake()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }
        _robotControllerLink = new BB_RobotControllerLink("127.0.0.1", 11115);
        InvokeRepeating("CacheEnemiesAndChooseOneToTrack", 0.0f, ENEMY_SEARCH_INTERVAL_SECONDS);
    }

    private void RobotControllerUpdate()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }

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
            gamepad1_left_stick_y = (float)input.Value.drive_amount;
            gamepad1_right_stick_x = (float)input.Value.turn_amount;
        }
    }

    protected override void FixedUpdate()
    {
        RobotControllerUpdate();
        // call the base class method
        base.FixedUpdate();
    }

    void Update()
    {
        if (gamepad1_b)
        {
            UnityEngine.Debug.Log("pszczesz: b pressed!");
            float velocity = _spinnersOn ? 0 : -SPINNER_SPEED;
            JointMotor motor = spinner1.motor;
            motor.targetVelocity = velocity;
            spinner1.motor = motor;

            motor = spinner2.motor;
            motor.targetVelocity = velocity;
            spinner2.motor = motor;
            _spinnersOn = !_spinnersOn;
        }
    }

    // this method is slow since it searches THE WHOLE SCENE for enemy robots
    // it is called once every so often
    private void CacheEnemiesAndChooseOneToTrack()
    {
        // find all enemy robot bo transforms.
        // order by distance to us
        _enemyRobotBodies = FindObjectsOfType<RobotInterface3D>()
                .Select(c => c.transform)
                .Where(c => c != this.transform)
                .Select(c => c.Find("Body"))
                .OrderBy(c => Vector3.Distance(robot_body.position, c.position))
                .ToList();

        // choose the closest to track
        if (_enemyRobotBodies.Count > 0)
        {
            Debug.Log("found enemy robot: " + _enemyRobotBodies.First().parent.name);
            opponent_body = _enemyRobotBodies.First();
        }
    }
}