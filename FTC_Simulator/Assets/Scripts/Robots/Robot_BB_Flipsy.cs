using System.Collections.Generic;
using UnityEngine;
using System.Linq;

// for now, just send the other robot's position + orientation
// eventually: send camera images so that robot controller deduces other robot position + orientation
[System.Serializable]
public struct BattleBotState
{
    // our robot
    public Vector3 robot_velocity;
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
    public HingeJoint selfRight;
    public int selfRightPosExtended = 100;
    private BB_RobotControllerLink _robotControllerLink;
    private TankDrive _tankDrive;
    public float downForce = 100f;

    // cached enemies, sorted by distance
    private List<Transform> _enemyRobotBodies = new List<Transform> { };
    // search for enemies every so often
    private const float ENEMY_SEARCH_INTERVAL_SECONDS = 2.0f;

    private bool _spinnersOn;

    // adds a force to the robot body to keep it on the ground
    void ApplyDownForce()
    {
        // apply down force
        robot_body.GetComponent<Rigidbody>().AddForce(Vector3.down * downForce);
    }


    public void Awake()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }
        _robotControllerLink = new BB_RobotControllerLink("127.0.0.1", 11115);
        InvokeRepeating("CacheEnemiesAndChooseOneToTrack", 0.0f, ENEMY_SEARCH_INTERVAL_SECONDS);
        // find the tank drive script
        _tankDrive = GetComponent<TankDrive>();
    }

    private void RobotControllerUpdate()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }

        ApplyDownForce();

        double input_rotation = Mathf.Atan2(Input.GetAxis("J1Axis2"), Input.GetAxis("J1Axis1")) * 180.0f / Mathf.PI;

        if (Mathf.Abs(Input.GetAxis("J1Axis2")) < 0.1 && Mathf.Abs(Input.GetAxis("J1Axis1")) < 0.1)
        {
            input_rotation = 0;
        }

        input_rotation = -180;
        // if k is pressed
        if (Input.GetKey(KeyCode.K))
        {
            input_rotation = 0;
        }

        double actual_rotation = opponent_body.rotation.eulerAngles[1];
        BattleBotState state = new BattleBotState
        {
            robot_velocity = robot_body.GetComponent<Rigidbody>().velocity,
            robot_orientation = robot_body.rotation.eulerAngles[1],
            opponent_position = opponent_body.position,
            opponent_orientation = input_rotation//actual_rotation
        };

        // send the robot controller the current state
        string message = JsonUtility.ToJson(state);
        _robotControllerLink.SendMessage(message);

        // receive latest control input from the robot controller
        RobotControllerMessage? input = _robotControllerLink.Receive();
 

        // movement with w and s
        float movement_amount = (Input.GetKey(KeyCode.W) ? 1.0f : 0) - (Input.GetKey(KeyCode.S) ? 1.0f : 0);
        // movement_amount += Input.GetAxis("J1Axis5");
        // turn with j and l
        float turn_amount = (Input.GetKey(KeyCode.A) ? 1.0f : 0) - (Input.GetKey(KeyCode.D) ? 1.0f : 0);
        // turn_amount -= Input.GetAxis("J1Axis1");

        // IF has input from robot controller, use that
        if (input.HasValue)
        {
            movement_amount += (float)input.Value.drive_amount;
            turn_amount += (float)input.Value.turn_amount;
        }

        _tankDrive.Drive(movement_amount, turn_amount);

        // q to self right
        if (Input.GetKey(KeyCode.Q))
        {
            JointSpring spring = selfRight.spring;
            spring.targetPosition = selfRightPosExtended;
            selfRight.spring = spring;
        }
        else
        {
            JointSpring spring = selfRight.spring;
            spring.targetPosition = 0;
            selfRight.spring = spring;
        }
    }

    protected override void FixedUpdate()
    {
        RobotControllerUpdate();
        // call the base class method
        base.FixedUpdate();
    }
    private Vector3 posLast = new Vector3(0, 0, 0);
    private Vector3 velocityLast = new Vector3(0, 0, 0);
    private bool wasMovingLast = false;
    private float lastMoveStartTime = 0;
    private Vector3 lastMovePos = new Vector3(0, 0, 0);
    void MeasureVelocity()
    {
        Vector3 currPos = robot_body.position * 2.658f / 3.28f; // hewo
        // get delta
        Vector3 delta = currPos - posLast;
        posLast = currPos;

        Vector3 currVelocity = delta / Time.deltaTime;
        Vector3 deltaVelocity = currVelocity - velocityLast;
        velocityLast = currVelocity;

        Vector3 acceleration = deltaVelocity / Time.deltaTime;

        if (currVelocity.magnitude > 0.1 && !wasMovingLast)
        {
            lastMoveStartTime = Time.timeSinceLevelLoad;
            lastMovePos = currPos;
            wasMovingLast = true;
        }

        if (currVelocity.magnitude < 0.1)
        {
            wasMovingLast = false;
        }

        Debug.Log("average acceleration = " + ((currPos - lastMovePos).magnitude / Mathf.Pow(Time.timeSinceLevelLoad - lastMoveStartTime, 2)));
        // Debug.Log("time since last stopped = " + (Time.timeSinceLevelLoad - lastMoveStartTime));


        // Debug.Log("acceleration: " + acceleration.magnitude);
        // Debug.Log("velocity: " + currVelocity.magnitude);
        // Debug.Log("position: " + currPos.magnitude);
    }
    void Update()
    {
        MeasureVelocity();

        if (gamepad1_b)
        {
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