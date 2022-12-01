using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_GF_Test1 : RobotInterface3D {

    public HingeJoint GF_armJoint;

    public HingeJoint GF_collector1;
    public HingeJoint GF_collector2;
    public HingeJoint GF_collector3;
    public ConfigurableJoint mainExtensionJoint;
    public ConfigurableJoint doorToMainStorage;
    public ConfigurableJoint mainStorageJoint;

    long stateStartTime = 0;
    Stopwatch dumperStateTimer = new Stopwatch();
    public enum dumperStates
    {
        dumping,
        resetting
    };
    dumperStates dumperState;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    Stopwatch mainExtensionStateTimer = new Stopwatch();
    public enum mainExtensionStates
    {
        goingForwards,
        resetting
    };
    mainExtensionStates mainExtensionState;



    //this is the main storage that goes back and fourth independently of the main extension of the collector
    Stopwatch mainStorageStateTimer = new Stopwatch();
    public enum mainStorageStates
    {
        goingForwards,
        resetting
    };
    mainStorageStates mainStorageState;

    // Do Pushbot updates
    public override void Update_Robot()
    {
        if (gamepad1_left_bumper)
        {
            //when we press left_bumper, this will open the door for the cubes to fall into the main storage
            doorToMainStorage.targetPosition = new Vector3(-0.2f, 0, 0);
        }
        if (gamepad1_right_bumper)
        {
            doorToMainStorage.targetPosition = new Vector3(0, 0, 0);
        }
        if (gamepad1_dpad_up)
        {
            mainExtensionState = mainExtensionStates.goingForwards;
            mainExtensionStateTimer.Reset();
            mainExtensionStateTimer.Start();
        }


        if (gamepad1_dpad_down)
        {
            mainExtensionState = mainExtensionStates.resetting;
            mainExtensionStateTimer.Reset();
            mainExtensionStateTimer.Start();
        }

        if (mainExtensionState == mainExtensionStates.goingForwards)
        {
            double max = 1.5;
            double value = (mainExtensionStateTimer.ElapsedMilliseconds / 1500.0) * max;
            if (value > max)
            {
                value = max;
            }
            mainExtensionJoint.targetPosition = new Vector3((float)value, 0, 0);
        }
        if (mainExtensionState == mainExtensionStates.resetting)
        {
            double max = 1.5;
            double value = (1.0 - (mainExtensionStateTimer.ElapsedMilliseconds / 1500.0)) * max;
            if (value < 0)
            {
                value = 0;
            }
            mainExtensionJoint.targetPosition = new Vector3((float)value, 0, 0);
        }

        if (gamepad1_x)
        {
            dumperState = dumperStates.dumping;
            dumperStateTimer.Reset();
            dumperStateTimer.Start();
        }
        if (gamepad1_b)
        {
            dumperState = dumperStates.resetting;
            dumperStateTimer.Reset();
            dumperStateTimer.Start();
        }

        if (gamepad1_a)
        {
            JointMotor s = GF_collector1.motor;
            s.targetVelocity = 1000;
            GF_collector1.motor = s;

            JointMotor g = GF_collector2.motor;
            g.targetVelocity = -1000;
            GF_collector2.motor = g;

            JointMotor h = GF_collector3.motor;
            h.targetVelocity = -750;
            GF_collector3.motor = h;
        }
        if (gamepad1_y)
        {
            JointMotor s = GF_collector1.motor;
            s.targetVelocity = -1000;
            GF_collector1.motor = s;

            JointMotor g = GF_collector2.motor;
            g.targetVelocity = 1000;
            GF_collector2.motor = g;

            JointMotor h = GF_collector3.motor;
            h.targetVelocity = 750;
            GF_collector3.motor = h;
        }


        if (dumperState == dumperStates.dumping)
        {
            JointSpring s = GF_armJoint.spring;
            double maxVal = 145;
            double value = 145 * dumperStateTimer.ElapsedMilliseconds / 1500f;
            if (value > maxVal)
            {
                value = maxVal;
            }
            s.targetPosition = (int)value;


            GF_armJoint.spring = s;
        }
        if (dumperState == dumperStates.resetting)
        {
            JointSpring s = GF_armJoint.spring;
            double maxVal = 145;
            double value = 145 * (1.0f - (dumperStateTimer.ElapsedMilliseconds / 1500f));
            if (value < 0)
            {
                value = 0;
            }
            s.targetPosition = (int)value;


            GF_armJoint.spring = s;
        }

        //MAIN STORAGE EXTENSION STUFF
        if (gamepad1_dpad_right)
        {
            mainStorageState = mainStorageStates.goingForwards;
            mainStorageStateTimer.Reset();
            mainStorageStateTimer.Start();
        }


        if (gamepad1_dpad_left)
        {
            mainStorageState = mainStorageStates.resetting;
            mainStorageStateTimer.Reset();
            mainStorageStateTimer.Start();
        }

        if (mainStorageState == mainStorageStates.goingForwards)
        {
            double max = 1.5;
            double value = (mainStorageStateTimer.ElapsedMilliseconds / 1500.0) * max;
            if (value > max)
            {
                value = max;
            }
            mainStorageJoint.targetPosition = new Vector3((float)value, 0, 0);
        }
        if (mainStorageState == mainStorageStates.resetting)
        {
            double max = 1.5;
            double value = (1.0 - (mainStorageStateTimer.ElapsedMilliseconds / 1500.0)) * max;
            if (value < 0)
            {
                value = 0;
            }
            mainStorageJoint.targetPosition = new Vector3((float)value, 0, 0);
        }

    }

}