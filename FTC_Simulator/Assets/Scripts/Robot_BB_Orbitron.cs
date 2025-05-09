﻿using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Diagnostics;

// for now, just send the other robot's position + orientation
// eventually: send camera images so that robot controller deduces other robot position + orientation
[System.Serializable]
public struct RobotControllerMessage
{
    // our robot
    public Vector3 robot_position;
    public double robot_rotation;
    public double robot_rotation_velocity;

    // other robot
    public Vector3 opponent_position;
    public double opponent_rotation;
    public double opponent_rotation_velocity;

    // spinner RPMs
    public double spinner_1_RPM;
    public double spinner_2_RPM;
}

public class Robot_BB_Orbitron : RobotInterface3D
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

    public static double opponent_movement_amount = 0;
    public static double opponent_turn_amount = 0;

    // adds a force to the robot body to keep it on the ground
    void ApplyDownForce()
    {
        // apply down force
        robot_body.GetComponent<Rigidbody>().AddForce(Vector3.down * downForce);
    }

    private List<Vector3> _ourInitialPositions = new List<Vector3> { };
    private List<Quaternion> _ourInitialRotations = new List<Quaternion> { };
    private List<Vector3> _opponentInitialPositions = new List<Vector3> { };
    private List<Quaternion> _opponentInitialRotations = new List<Quaternion> { };

    public void Awake()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }
        _robotControllerLink = new BB_RobotControllerLink("127.0.0.1", 11115);
        InvokeRepeating("CacheEnemiesAndChooseOneToTrack", 0.0f, ENEMY_SEARCH_INTERVAL_SECONDS);
        // find the tank drive script
        _tankDrive = GetComponent<TankDrive>();


        stopwatch.Start();

        // go one level up, find all children and save the initial positions and rotations
        foreach (Transform child in robot_body.parent)
        {
            _ourInitialPositions.Add(child.position);
            _ourInitialRotations.Add(child.rotation);
        }

        foreach (Transform child in opponent_body.parent)
        {
            _opponentInitialPositions.Add(child.position);
            _opponentInitialRotations.Add(child.rotation);
        }

    }


    // simulated gyro rotation
    private double gyroRotationRad = 0.0f;
    

    private int frames = 0;
    private Stopwatch stopwatch = new Stopwatch();

    private double maxInterval = 0;
    private long prevTime = 0;


    /// <summary>
    /// Send the robot's current state to the robot controller
    /// </summary> 
    private void SendRobotDataToRC()
    {
        // get the robot's rigidbody
        Rigidbody rb = robot_body.GetComponent<Rigidbody>();
        Rigidbody rbo = opponent_body.GetComponent<Rigidbody>();

        double robot_rotation = robot_body.rotation.eulerAngles[1];
        double robot_rotation_velocity = rb.angularVelocity.y;

        const float GYRO_NOISE_PERCENTAGE = 0.2f;
        // add noise to the robot rotation to simulate gyro drift
        double robotRotationWithNoise = Random.Range(1.0f - GYRO_NOISE_PERCENTAGE, 1.0f + GYRO_NOISE_PERCENTAGE) * robot_rotation_velocity;
        // integrate the gyro velocity to get the gyro rotation
        gyroRotationRad += Time.deltaTime * robotRotationWithNoise;

        double opponent_rotation_velocity = rbo.angularVelocity.y;



        // get the spinner RPMs
        double spinner_1_RPM = spinner1.velocity * 60 / 360;
        double spinner_2_RPM = spinner2.velocity * 60 / 360;

        // create a message to send to the robot controller
        RobotControllerMessage rcMessage = new RobotControllerMessage
        {
            robot_position = robot_body.position,
            robot_rotation = robot_rotation * Mathf.Deg2Rad,
            robot_rotation_velocity = robotRotationWithNoise,
            opponent_position = opponent_body.position,
            opponent_rotation = opponent_body.rotation.eulerAngles[1],
            opponent_rotation_velocity = opponent_rotation_velocity,
            spinner_1_RPM = spinner_1_RPM,
            spinner_2_RPM = spinner_2_RPM
        };

        // send the robot controller the current state
        string message = JsonUtility.ToJson(rcMessage);
        _robotControllerLink.SendMessage(message);
    }

    private void RobotControllerUpdate()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }

        ApplyDownForce();

        SendRobotDataToRC();

        frames ++;

        // // log elapsedmilliseconds
        // UnityEngine.Debug.Log("Elapsed milliseconds: " + stopwatch.ElapsedMilliseconds);

        if (stopwatch.ElapsedMilliseconds - prevTime > maxInterval)
        {
            maxInterval = stopwatch.ElapsedMilliseconds - prevTime;
        }

        prevTime = stopwatch.ElapsedMilliseconds;

        if (stopwatch.ElapsedMilliseconds > 1000)
        {
            UnityEngine.Debug.Log("Sending to RC fps: " + (float) frames  / (stopwatch.ElapsedMilliseconds / 1000.0));
            UnityEngine.Debug.Log("Max interval: " + maxInterval);
            frames = 0;
            stopwatch.Restart();
            maxInterval = 0;
        }


        // receive latest control input from the robot controller
        RobotControllerDriveCommand? input = _robotControllerLink.Receive();

        if (input.HasValue && input.Value.reset)
        {
            // move all our components back to the initial position
            for (int i = 0; i < robot_body.parent.childCount; i++)
            {
                Transform child = robot_body.parent.GetChild(i);
                child.position = _ourInitialPositions[i];
                child.rotation = _ourInitialRotations[i];

                // set the velocity to 0 only if it's a rigidbody
                Rigidbody rb = child.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.velocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }

            for (int i = 0; i < opponent_body.parent.childCount; i++)
            {
                Transform child = opponent_body.parent.GetChild(i);
                child.position = _opponentInitialPositions[i];
                child.rotation = _opponentInitialRotations[i];

                // set the velocity to 0 only if it's a rigidbody
                Rigidbody rb = child.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.velocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }
        }
 
         // movement with arrow keys + joystick
        float movement_amount = (Input.GetKey(KeyCode.UpArrow) ? 1.0f : 0) - (Input.GetKey(KeyCode.DownArrow) ? 1.0f : 0);
        //movement_amount += Input.GetAxis("J1Axis5");
        float turn_amount = (Input.GetKey(KeyCode.LeftArrow) ? 1.0f : 0) - (Input.GetKey(KeyCode.RightArrow) ? 1.0f : 0);
        //turn_amount -= Input.GetAxis("J1Axis1");
        turn_amount *= 0.8f;

        // float movement_amount = 0;
        // float turn_amount = 0;
        // IF has input from robot controller, use that
        if (input.HasValue)
        {
            movement_amount += (float)input.Value.drive_amount;
            turn_amount += (float)input.Value.turn_amount;

            opponent_movement_amount = (float) input.Value.opponent_drive_amount;
            opponent_turn_amount = (float) input.Value.opponent_turn_amount;

            UnityEngine.Debug.Log("spinner 1 power: " + input.Value.front_weapon_power);
            UnityEngine.Debug.Log("spinner 2 power: " + input.Value.back_weapon_power);
            JointMotor motor1 = spinner1.motor;
            motor1.targetVelocity = (float)input.Value.front_weapon_power / 300.0f * SPINNER_SPEED;
            spinner1.motor = motor1;
            
            JointMotor motor2 = spinner2.motor;
            motor2.targetVelocity = (float)input.Value.back_weapon_power / 300.0f * SPINNER_SPEED;
            spinner2.motor = motor2;
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

        // UnityEngine.Debug.Log("average acceleration = " + ((currPos - lastMovePos).magnitude / Mathf.Pow(Time.timeSinceLevelLoad - lastMoveStartTime, 2)));
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
            // UnityEngine.Debug.Log("found enemy robot: " + _enemyRobotBodies.First().parent.name);
            opponent_body = _enemyRobotBodies.First();
        }
    }
}