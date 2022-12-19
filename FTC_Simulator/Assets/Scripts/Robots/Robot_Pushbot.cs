using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Pushbot
// *******************************
//



public class Robot_Pushbot : RobotInterface3D {

    public float pushbot_arm_speed = 75f;
	
    //pushbot stuff
    public HingeJoint mainArmJoint;
    public HingeJoint finger_r_joint;
    public HingeJoint finger_l_joint;
    public float arm_max_limit = 180f;
    public float arm_min_limit = -180f;

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }
    // Do Pushbot updates
    public override void Update_Robot()
    {
        HingeJoint tempJoint = mainArmJoint;
        JointSpring temp = tempJoint.spring;


        if (gamepad1_dpad_down)
        {
            temp.targetPosition += pushbot_arm_speed * Time.deltaTime;
            if( temp.targetPosition > arm_max_limit)
            {
                temp.targetPosition = arm_max_limit;
            }

        }
        if (gamepad1_dpad_up)
        {
            temp.targetPosition -= pushbot_arm_speed * Time.deltaTime;
            if (temp.targetPosition < arm_min_limit)
            {
                temp.targetPosition = arm_min_limit;
            }

        }
        tempJoint.spring = temp;
        mainArmJoint = tempJoint;


        if (gamepad1_x)
        {
            JointSpring temp1 = finger_l_joint.spring;
            temp1.targetPosition = 30;
            finger_l_joint.spring = temp1;

            JointSpring temp2 = finger_r_joint.spring;
            temp2.targetPosition = -30;
            finger_r_joint.spring = temp2;
        }

        if (gamepad1_a)
        {
            JointSpring temp1 = finger_l_joint.spring;
            temp1.targetPosition = -15;
            finger_l_joint.spring = temp1;

            JointSpring temp2 = finger_r_joint.spring;
            temp2.targetPosition = 15;
            finger_r_joint.spring = temp2;
        }

    }

    // If robot has a bumper, set the name
    public override void SetName(string name)
    {
        // Set the base items
        base.SetName(name);

        bool name_tag_found = false;
        // Set our own items
        Transform mynametag = transform.Find("Body/NametagB1");
        if (mynametag)
        {
            mynametag.GetComponent<TMPro.TextMeshPro>().text = name;
            name_tag_found = true;
        }
        // Set our own items
        mynametag = transform.Find("Body/NametagB2");
        if (mynametag)
        {
            mynametag.GetComponent<TMPro.TextMeshPro>().text = name;
            name_tag_found = true;
        }

        // Turn off main name-tag
        if (name_tag_found)
        {
            mynametag = transform.Find("Body/Nametag");
            mynametag.gameObject.SetActive(false);
        }
    }

}