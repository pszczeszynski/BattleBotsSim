using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;

// *******************************
// ROBOT interface for Robot_Vulcan
// *******************************
//



public class Robot_Vulcan : RobotInterface3D {

    public ConfigurableJoint limb_arm5; // Actual hand
    public ConfigurableJoint limb_arm4; // Main lifting arm: 4th stage
    public ConfigurableJoint limb_arm3; // dummy arm: 3rd stage
    public ConfigurableJoint limb_arm2; // dummy arm: 2nd stage
    public float limb_arm_start = 0;
    public float limb_arm_speed = 100;

    public HingeJoint limb_arm1;    // z-axis rotaton of hand (tilt of hand)
    public float limb_arm1_angle_start = 0;
    public float limb_arm1_speed = 100;

    public ConfigurableJoint finger_l; // fingers
    public ConfigurableJoint finger_r; // fingers
    public float finger_speed = 100;
    public float finger_start_pos = 0;
    public float finger_end_pos = -0.2f;

    /// <summary>
    /// Only animated, not physics
    /// </summary>
    public Transform servoJoint1_R;
    public Transform servoJoint2_R;
    public float servoJoint1_startAngle_R = 0;
    public float servoJoint1_endAngle_R = 90;

    public float servoJoint2_startAngle_R = 0;
    public float servoJoint2_endAngle_R = 90;

    // the relative position of the gripper
    private float fingerMinPosVisual_R = 1.048f;//0.57f;
    private float fingerMaxPosVisual_R = 0.873f;//0.77f;

    public Transform servoJoint1_L;
    public Transform servoJoint2_L;
    public float servoJoint1_startAngle_L = 0;
    public float servoJoint1_endAngle_L = 90;

    public float servoJoint2_startAngle_L = 0;
    public float servoJoint2_endAngle_L = 90;
    
    // the relative position of the gripper
    private float fingerMinPosVisual_L = 0.576f;
    private float fingerMaxPosVisual_L = 0.771f;


    /*
        public HingeJoint limb_hand;     // y-axis rotation of hand (flip hand around to face outward). Angle is negative of this value
        public float limb_hand_angle_start = 0;
        public float limb_hand_speed = 100;

        public HingeJoint limb_fingers;  // z-axis rotation of fingers (grasp block)
        public float limb_fingers_angle_start = 100;
        public float limb_fingers_speed = 100;
         */

    bool gripped = false;
    bool grip_button_last = false;
    float lastResetTime = 0;
    bool a_button_last = false;
    bool b_button_last = false;
       
    public enum armStates : int
    {
        start = 0,    // All joints in starting position
        ready,
        ready2,
        tilt_down,    // Tilt's arm for intake

        gripping,     // attempting to grip block
        starting_up,  // lift block up out of cradle
        swiveling,    // swivel block to ready poisition
        settling,     // preparing block to correct level
        ungrip,       // ungrips block
        clear,        // Move lift up a bit
        clear2        // Retracks hand  
    }
    public armStates arm_state = armStates.start;
    public int armstate = 0;
    float arm_x_saved = 0;
    float wrist_angle_saved = 0;
    // Initialize joints to minimum state

    public void Awake()
    {
        info =
            "<Missing Robot Specific Function: TBD>" +
            info;
    }

    public override void Start()
    {
        base.Start();
        ResetJoints();
    }

    JointSpring g1;

    // Reset all joints to starting position
    void ResetJoints()
    {
        g1 = limb_arm1.spring;
        g1.targetPosition = limb_arm1_angle_start;
        limb_arm1.spring = g1;

        MoveArm(limb_arm_start, 2f*limb_arm_speed);

    }


    // Update Call
    public override void Update_Robot()
    {
        // ****************************
        // X button: Get arm ready for intake
        if (gamepad1_x && arm_state == armStates.ready2)
        {
            arm_state = armStates.tilt_down;
        }

        // ****************************
        // B button: Returns arm to starting position
        if (gamepad1_b )
        {
            arm_state = armStates.start;
        }
       
        // b_button_last = gamepad1_b;


        // ****************************
        // d_pad_up/down move arm up/down
        if (gamepad1_dpad_down) /*&& arm_state == armStates.ready*/
        {
            MoveArm(0, limb_arm_speed);
        }
        if (gamepad1_dpad_up) /* && arm_state == armStates.ready*/
        {
            MoveArm(1.5f, limb_arm_speed);
        }

        // ****************************
        // d_pad_left/right closes/opens fingers
        if (gamepad1_dpad_right )
        {
            Vector3 temppos;
            temppos = finger_l.targetPosition;

            float newvalue = MoveTowards(temppos.x, finger_end_pos, temppos.x, Time.deltaTime * finger_speed);
            temppos.x = newvalue;
            finger_l.targetPosition = temppos;
            finger_r.targetPosition = temppos;

        }
        if (gamepad1_dpad_left)
        {
            Vector3 temppos;
            temppos = finger_l.targetPosition;

            float newvalue = MoveTowards(temppos.x, finger_start_pos, temppos.x, Time.deltaTime * finger_speed);
            temppos.x = newvalue;
            finger_l.targetPosition = temppos;
            finger_r.targetPosition = temppos;
        }

        // ****************************
        Vector3 gripperRelativePosition_R = limb_arm5.transform.InverseTransformPoint(finger_l.transform.position);
        // Gripping servo visuals
        float currentFingerPos_R = gripperRelativePosition_R.x;
        // 0 when unactivated, 1 when fully activated
        float fingerActivationPercent_R = (currentFingerPos_R - fingerMinPosVisual_L) / (fingerMaxPosVisual_L - fingerMinPosVisual_L);
        // UnityEngine.Debug.Log($"pszczesz: activationPercent_R: {fingerActivationPercent_R}");
        // UnityEngine.Debug.Log($"pszczesz: currPos_R: {currentFingerPos_R}");

        // servoJoint1_R.transform.localRotation = Quaternion.Euler(0, fingerActivationPercent_R * (servoJoint1_endAngle_R - servoJoint1_startAngle_R) + servoJoint1_startAngle_R,0);
        // servoJoint2_R.transform.localRotation = Quaternion.Euler(0, fingerActivationPercent_R * (servoJoint2_endAngle_R - servoJoint2_startAngle_R) + servoJoint2_startAngle_R,0);


        Vector3 gripperRelativePosition_L = limb_arm5.transform.InverseTransformPoint(finger_r.transform.position);
        // Gripping servo visuals
        float currentFingerPos_L = gripperRelativePosition_L.x; 
        // 0 when unactivated, 1 when fully activated
        float fingerActivationPercent_L = (currentFingerPos_L - fingerMinPosVisual_R) / (fingerMaxPosVisual_R - fingerMinPosVisual_R);
        // UnityEngine.Debug.Log($"pszczesz: activationPercent_L: {fingerActivationPercent_L}");
        // UnityEngine.Debug.Log($"pszczesz: currPos_L: {currentFingerPos_L}");

        // servoJoint1_L.transform.localRotation = Quaternion.Euler(0, fingerActivationPercent_L * (servoJoint1_endAngle_L - servoJoint1_startAngle_L) + servoJoint1_startAngle_L,0);
        // servoJoint2_L.transform.localRotation = Quaternion.Euler(0, fingerActivationPercent_L * (servoJoint2_endAngle_L - servoJoint2_startAngle_L) + servoJoint2_startAngle_L,0);



        // ****************************
        // Arm State Machine
        if (arm_state == armStates.start)
        {
            bool alldone = true;

            // Move all joints to home position
            if ( !MoveArm(limb_arm_start, 3f*limb_arm_speed)) { alldone = false;  }
            if( !MyMoveHinge(limb_arm1, limb_arm1_angle_start, limb_arm1_speed)) { alldone = false; }

            if (alldone)
            {
                arm_state = armStates.ready2;
            }
        }
        if (arm_state == armStates.tilt_down)
        {
            bool alldone = true;

            // rotate arm down
            if (!MyMoveHinge(limb_arm1, -90f, limb_arm1_speed)) { alldone = false; }
            if (alldone)
            { arm_state = armStates.ready; }
        }
    }


    // Moves joint by a small delta. If reached destination, return true
    bool MyMoveHinge( HingeJoint myhinge, float target, float speed)
    {
        return MoveHinge(myhinge, target, speed) == target;
    }

    // Moves joint by a small delta. If reached destination, return true
    // target is a positive number, but all positions in the spring are negative.
    // I probably should have left everything negative to avoid confusion...
    bool MoveArm(float target, float speed)
    {
        // Constrain target to limits
        if (limb_arm4.linearLimit.limit < target) { target = limb_arm4.linearLimit.limit; }
        if (0 > target) { target = 0; }

        // Quit if we reached our target
        if (-1f*limb_arm4.targetPosition.x == target) { return true; }

        Vector3 temppos;
        temppos = limb_arm4.targetPosition;

        float newvalue = MoveTowards(temppos.x, -1f*target, temppos.x, Time.deltaTime * speed);
        temppos.x = newvalue;

        limb_arm5.targetPosition = -1f*temppos * 4f / 3f;
        limb_arm4.targetPosition = temppos;
        
        temppos.x = temppos.x * 2f / 3f;
        limb_arm3.targetPosition = temppos;

        temppos.x = temppos.x / 2f;
        limb_arm2.targetPosition = temppos;

        return false;
    }

}