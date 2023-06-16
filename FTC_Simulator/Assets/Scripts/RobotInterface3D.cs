using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using System.Text;
using System.Diagnostics;
using FTC_Simulator.Assets.Scripts;
using Debug = UnityEngine.Debug;


// *******************************
// ROBOT interface and storage of state values
// *******************************
//
// UPDATED: Removed internal stability loop - just using spring angular force mode. 
//          General cleanup due to this.

public class RobotInterface3D : MonoBehaviour {

    private bool DEBUG = false;
    private bool DEBUG_COG = false;

    private bool use_new_algorithm = false;

    public bool deleted = false; // When set true, this item has been marked for deletion

    public GameObject wheelTL;
    public GameObject wheelTR;
    public GameObject wheelBL;
    public GameObject wheelBR;

    // Wheels for 6-wheel drive
    public GameObject wheelML;
    public GameObject wheelMR;

    // Save the joints for easy access
    private ConfigurableJoint wheelTL_joint;
    private Rigidbody wheelTL_body;
    private ConfigurableJoint wheelTR_joint;
    private Rigidbody wheelTR_body;
    private ConfigurableJoint wheelBL_joint;
    private Rigidbody wheelBL_body;
    private ConfigurableJoint wheelBR_joint;
    private Rigidbody wheelBR_body;
    private ConfigurableJoint wheelML_joint;
    private Rigidbody wheelML_body;
    private ConfigurableJoint wheelMR_joint;
    private Rigidbody wheelMR_body;

    public Vector3 centerOfMass; // Sets the center of mass
    public float rot_inertia_scaler = 1f; // scales the rotational inertia 
    public bool rot_inertia_scale_only_body = true;  // if true, only scale body
    private float max_speed_multiplier = 1000f; // This is a clamp that prevents the wheels from reaching past this multiplier, not sure we ever need this (I think it was used to clamp wild oscialltions during stability issues)
    public float turning_overide = 0f; // This variable forces the robot to turn in place. Can be used by derived Robots to help control movement.
    public float friction_extrax = 1f;

    public List<string> valid_DriveTrains = new List<string>() { "Tank", "Mecanum" };


    // *****************************
    // User parameters that can be changed
    public string DriveTrain = "Tank";
    public bool DriveTrain_lock = false;

    public float total_weight = 1f; // Weight of robot in pounds
    public bool total_weight_lock = false;

    public bool fieldcentric = false;
    public bool fieldcentric_lock = false;

    public bool activebreaking = false;
    public bool activebreaking_lock = false;

    public bool tankcontrol = false;
    public bool tankcontrol_lock = false;
    // ***************************************


    public bool lock_all_parameters = false;


    public Quaternion fieldcentric_rotation; // Reference rotation of camera to guide fieldcentric movement

    public RobotID myRobotID = null;

    public bool is_FRC = false;
    protected bool isKinematic = false;  // Marks this robot is set to be kinematic. use SetKinematic to change modes

    // Mark that this is not a real robot. This should not be used to disable a robot.
    public bool isSpectator = false; // Easy check for others that this should be ignored. This variable should only be initialized at the beggining and never changed afterwords since it avoid certain initializations.

    protected GameObject highlitecircle = null;  // We turn it on to indicate robot is under mouse in ClientMode
    protected GameObject selectcircle = null;  // We turn it on to indicate robot is selected in ClientMode

    //States that may need to be syncrhonized to client
    protected ProgressMeter progressmeter = null;  // Progress meter is controlled by the scorekeepers
    public bool disabledTopObjects = false;  // If false, the top elements are disabled and thus not visible

    // Client side relevant variables 
    public bool pauseUpdates = false; // If set to true, client will not update positions on this robot
    public int robot_color = -1; // Color code: 0 = grey, 1 = red, 2 = blue, 3 = .....

    // Client side: the robotskin needs to tie into this. Right now only 1 skin allowed
    public RobotSkin robotskin = null;

    struct ObjPos // class for storing initial object positions
    {
        public Vector3 pos;
        public Quaternion rot;
    };

    // Store the starting position of every child to reset later (indexed by unity's uuid)
    Dictionary<int, ObjPos> starting_positions = new Dictionary<int, ObjPos>();

    // Store the kinematic state of every child for resetting later
    Dictionary<int, bool> kinematic_states = new Dictionary<int, bool>();
    Dictionary<int, CollisionDetectionMode> collision_states = new Dictionary<int, CollisionDetectionMode>();

    public string info =
        "\nLeft/Right Bumper = Precision Mode (while held) " +
        "\nLeft Joystick Y-axis = Forward/Backwards" +
        "\nLeft Joystick X-axis = Strafe" +
        "\nRight Joystick X-axis = Turn/Rotate" +
        "\nPress in left joystick = Switch Camera" +
        "\nPress in right joystick = Restart Position" +
        "\nBack button = Start Timer" +
        "\nStart button = Stop Timer";

    private void OnEnable()
    {
        time_last_button_activitiy = MyUtils.GetTimeMillis();
    }

    // Derived classes sometimes use start but forget this base-class start to be called.
    // be carefull.
    virtual public void Start()
    {
        Initialize();

        // Save all initial LOCAL positions
        Transform[] allchildren = GetComponentsInChildren<Transform>(true);
        foreach (Transform currtrans in allchildren)
        {

            ObjPos currpos = new ObjPos();

            currpos.pos = currtrans.localPosition;
            currpos.rot = currtrans.localRotation;
            starting_positions[currtrans.GetInstanceID()] = currpos;
        }
    }

    // Reset all children positions to that present at Start (plus an offset if applied to top level only
    public void ResetPosition(float x_off = 0f, float y_off = 0, float z_off = 0)
    {
        // Restore all positions
        Transform[] allchildren = GetComponentsInChildren<Transform>(true);
        foreach (Transform currtrans in allchildren)
        {
            int currid = currtrans.GetInstanceID();
            if (!starting_positions.ContainsKey(currid) || (currtrans == transform))
            {
                continue;
            }



            // If this is top level, add offset otherwise do not
            if (currtrans.parent == transform)
            {
                currtrans.localPosition = starting_positions[currid].pos + new Vector3(x_off, y_off, z_off);
            }
            else
            {
                currtrans.localPosition = starting_positions[currid].pos;
            }

            currtrans.localRotation = starting_positions[currid].rot;

        }
    }


    public bool doinit = false;
    public Rigidbody rb_body;

    FileStream mylogfile;
    UnicodeEncoding encoding;
    private void WriteLog(String text)
    {
        if (DEBUG && (mylogfile != null))
        {
            byte[] mybytes = encoding.GetBytes(text);
            mylogfile.Write(mybytes, 0, mybytes.Length);
        }
    }

    private void OnDestroy()
    {
        if (mylogfile != null)
        {
            mylogfile.Close();
        }
    }

    public void SetSpeedParametersToGlobals()
    {
        if (lock_all_parameters) { return; }

        max_speed = GLOBALS.speed;
        max_acceleration = GLOBALS.acceleration;
        total_weight = GLOBALS.weight;
    }

    public void SetUserParameters(float new_max_speed = -1f,
                                       float new_max_acceleration = -1f,
                                       float new_total_weight = -1f,
                                       string new_DriveTrain = "",
                                       float new_turn_scale = -1f,
                                       int new_field_centric = -1,
                                       int new_active_breaking = -1,
                                       int new_tank_control = -1)
    {
        if (!max_speed_lock && !lock_all_parameters)
        {
            max_speed = (new_max_speed < 0f) ? GLOBALS.speed : new_max_speed;
        }

        if (!max_acceleration_lock && !lock_all_parameters)
        {
            max_acceleration = (new_max_acceleration < 0f) ? GLOBALS.acceleration : new_max_acceleration;
        }

        if (!total_weight_lock && !lock_all_parameters)
        {
            total_weight = (new_total_weight < 0f) ? GLOBALS.weight : new_total_weight;
        }

        if (!DriveTrain_lock)
        {
            DriveTrain = (new_DriveTrain.Length < 1) ? GLOBALS.DriveTrain : new_DriveTrain;

            // If this is 6-wheel drive, enable the middle wheels
            if (DriveTrain == "6-Wheel Tank")
            {
                if (wheelML) { wheelML.SetActive(true); }
                if (wheelMR) { wheelMR.SetActive(true); }
            }
            else
            {
                if (wheelML) { wheelML.SetActive(false); }
                if (wheelMR) { wheelMR.SetActive(false); }
            }
        }

        if (!turn_scale_lock)
        {
            turn_scale = (new_turn_scale < 0f) ? GLOBALS.turning_scaler : new_turn_scale;
        }

        if (!fieldcentric_lock)
        {
            fieldcentric = (new_field_centric < 0) ? GLOBALS.fieldcentric : (new_field_centric == 1);
        }

        if (!activebreaking_lock)
        {
            activebreaking = (new_active_breaking < 0) ? GLOBALS.activebreaking : (new_active_breaking == 1);
        }

        if (!tankcontrol_lock)
        {
            tankcontrol = (new_tank_control < 0) ? GLOBALS.tankcontrol : (new_tank_control == 1);
        }

    }

    // Initialize does calculations does need to be done after the component is created, for example, if it was instantiated but it's mass, velocity were changed after
    public void Initialize()
    {
        if (DEBUG && (mylogfile == null))
        {
            mylogfile = new FileStream(Application.persistentDataPath + @"\FTCsim debug.csv", FileMode.Append);

            encoding = new UnicodeEncoding();

            WriteLog("Log file started " + DateTime.Now + "\n");
        }


        // Get the body component for mass calcs
        if (!transform.Find("Body")) { return; }

        rb_body = transform.Find("Body").GetComponent<Rigidbody>();
        if (!rb_body) { return; }
        if (!rb_body.gameObject.activeSelf) { return; }

        // Set center of mass
        rb_body.centerOfMass = centerOfMass;

        // ********* Torque calculation
        // 
        // Field size real-world = 12ft by 12ft
        // Field size unity = 7.09 x 7.09
        // Scaler = 1.69ft/unit = 22inches/unit
        // Wheel diameter = 0.1693 = 3.4334"
        // Max speed = ft/s = x12 inch/s = 1.025 rotations/s = 6.44 rads/s

        // Make the robot match total weight
        SetTotalWeight(total_weight, rot_inertia_scaler);

        // 1.5f for slow

        // Acceleration is in ft/s^2
        // We will increase acceleration by the friction since friction is always applied but measurements already include its effect 
        max_speed_corr = max_speed * 12f * 2f / 3.4334f;
        max_torque = total_weight * (max_acceleration + GLOBALS.friction) / 75f / max_speed_corr; // Experimention found this scaler. Max_pseed_corr is here because the motor "mode" applies force linearly proportional to max_speed
        friction_torque_scaler = total_weight / 75f;

        // Make sure wheels are set for local operations
        // IF there are no wheels, then exit
        if (isSpectator) { return; }



        InitWheel(wheelTL, ref wheelTL_joint, ref wheelTL_body);
        InitWheel(wheelTR, ref wheelTR_joint, ref wheelTR_body);
        InitWheel(wheelBL, ref wheelBL_joint, ref wheelBL_body);
        InitWheel(wheelBR, ref wheelBR_joint, ref wheelBR_body);
        if (wheelML) { InitWheel(wheelML, ref wheelML_joint, ref wheelML_body); }
        if (wheelMR) { InitWheel(wheelMR, ref wheelMR_joint, ref wheelMR_body); }

        // If the movement is restricted to tank drive, than remove sideways component
        // Also, front wheels are omni-wheels: no drag on them
        // For 6-wheel drive the middle wheels have no sideways movement, but the others do.
        //
        // In general 6-wheel drive should always be tank drive, but we wont enforce that here.

        if (DriveTrain == "Tank")
        {
            if (wheelBL && wheelBR)
            {
                // The wheel that is supplied initial power is the Back wheels if 4-wheel drive
                // Will prevent side-way motion in the driven wheel. All others are omni-wheels.

                wheelBL.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Locked;
                wheelBL.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Locked;
                wheelBR.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Locked;
                wheelBR.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Locked;
            }
        }
        else if (DriveTrain == "6-Wheel Tank")
        {
            // The wheel that is supplied initial power is the Back wheels if 4-wheel drive, center wheel if 6-wheel drive.
            // Will prevent side-way motion in the driven wheel. All others are omni-wheels.
            // Center wheels can only rotate forward/backwards
             if (wheelML)
            {
                wheelML.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Locked;
                wheelML.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Locked;
            }

            if (wheelMR)
            {
                wheelMR.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Locked;
                wheelMR.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Locked;
            }
            
            
            wheelTL.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Free;
            wheelTL.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Free;
            wheelTR.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Free;
            wheelTR.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Free;

            wheelBL.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Free;
            wheelBL.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Free;
            wheelBR.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Free;
            wheelBR.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Free;
            

        }
        else
        {
            wheelBL.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Free;
            wheelBL.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Free;
            wheelBR.GetComponent<ConfigurableJoint>().angularYMotion = ConfigurableJointMotion.Free;
            wheelBR.GetComponent<ConfigurableJoint>().angularZMotion = ConfigurableJointMotion.Free;
        }



        // ***************** ROBOT COLLISION HANDLING *******************
        // Things associated with determining robot-to-robot collisions
        // **************************************************************
        // NEW: each model includes a collision boundry that grows body to what it needs to be.
        // thus we only need to attach our script.
        //
        // OLD: Tried creating my ownbbox... didn't work
        // Next, create a box collider that will encapsulate all of the children
        // Need to set scale to 1 first, calculate everything, then set scale back


        /*BoxCollider boxCol = rb_body.gameObject.GetComponent<BoxCollider>();
        if (boxCol == null)
        {
            boxCol = rb_body.gameObject.AddComponent<BoxCollider>();
        }
        boxCol.isTrigger = true;

        Vector3 oldscale = new Vector3(); // Remeber scale
        oldscale = rb_body.transform.localScale;
        rb_body.transform.localScale = new Vector3(1f, 1f, 1f); // Set body to scale of 1

        // Determine it's size usings bounds function
        Bounds bbox = new Bounds(rb_body.transform.position, Vector3.zero);
        foreach (Renderer r in rb_body.gameObject.GetComponentsInChildren<Renderer>())
        {
            // Ignore nametag
            if( r.name.StartsWith("Nametag")) { continue;  }
            bbox.Encapsulate(r.bounds);
        }

        // Add this bounding box for collisions to our robot. 
        // Increase size by 5%
        boxCol.center = bbox.center - rb_body.transform.position;
        boxCol.size = bbox.size * 1.05f;

        // Restore scale (thus properly scaling down the boundingbox)
        rb_body.transform.localScale = oldscale;
        */

        // Add helper class to move collisions to us
        PassCollisionsUp col_helper = rb_body.gameObject.GetComponent<PassCollisionsUp>();
        if (col_helper == null)
        {
            col_helper = rb_body.gameObject.AddComponent<PassCollisionsUp>();
            col_helper.owner = this;
        }

        // ********************************* Progress Meter ***********************
        // Add progress meter
        if (progressmeter == null)
        {
            GameObject prefab = Resources.Load("Prefabs/ProgressMeter") as GameObject;
            GameObject instance = Instantiate(prefab, rb_body.transform) as GameObject;

            // Scale it to the body's scale
            // Most of the time, all body scales are 1... however, the FRC robots are slightly bigger then FTC robots, thus scale it up for FRC
            Vector3 scaling = instance.transform.localScale;
            scaling.x *= 1f / rb_body.transform.localScale.x;
            scaling.y *= 1f / rb_body.transform.localScale.y;
            scaling.z *= 1f / rb_body.transform.localScale.z;

            instance.transform.localScale = scaling * ((is_FRC) ? 1.3f : 1f);

            progressmeter = instance.GetComponent<ProgressMeter>();
        }

        // Add highlite circle
        if (highlitecircle == null)
        {
            GameObject prefab = Resources.Load("Prefabs/HighliteCircle") as GameObject;
            GameObject instance = Instantiate(prefab, rb_body.transform) as GameObject;

            // Scale it to the body's scale
            // Most of the time, all body scales are 1... however, the FRC robots are slightly bigger then FTC robots, thus scale it up for FRC
            Vector3 scaling = instance.transform.localScale;
            scaling.x *= 1f / rb_body.transform.localScale.x;
            scaling.y *= 1f / rb_body.transform.localScale.y;
            scaling.z *= 1f / rb_body.transform.localScale.z;

            instance.transform.localScale = scaling * ((is_FRC) ? 1.3f : 1f);

            highlitecircle = instance;
            highlitecircle.SetActive(false);
        }

        // Add select circle
        if (selectcircle == null)
        {
            GameObject prefab = Resources.Load("Prefabs/SelectCircle") as GameObject;
            GameObject instance = Instantiate(prefab, rb_body.transform) as GameObject;

            // Scale it to the body's scale
            // Most of the time, all body scales are 1... however, the FRC robots are slightly bigger then FTC robots, thus scale it up for FRC
            Vector3 scaling = instance.transform.localScale;
            scaling.x *= 1f / rb_body.transform.localScale.x;
            scaling.y *= 1f / rb_body.transform.localScale.y;
            scaling.z *= 1f / rb_body.transform.localScale.z;

            instance.transform.localScale = scaling * ((is_FRC) ? 1.3f : 1f);

            selectcircle = instance;
            selectcircle.SetActive(false);
        }

        // Do final initializations
        myRobotID = gameObject.GetComponent<RobotID>();

        // instance.transform.parent = rb.transform;
        if (myRobotID != null)
        {
            // Some scorekeepers use this bool
            myRobotID.is_red = (myRobotID.starting_pos.StartsWith("Red")) ? true : false; // This is an over-simplistic bool that may be removed in future

            if (robot_color != -1)
            {
                SetRobotColor(robot_color, true);
            }
            else
            {
                SetColorFromPosition(myRobotID.starting_pos);
            }
        }

        Init_Robot();
    }

    private void InitWheel(GameObject wheel, ref ConfigurableJoint wheel_joint, ref Rigidbody wheel_body)
    {
        if (wheel == null) { return; }
        wheel_joint = wheel.GetComponent<ConfigurableJoint>();
        wheel_body = wheel_joint.GetComponent<Rigidbody>();

        wheel_joint.configuredInWorldSpace = false;
        wheel_body.maxAngularVelocity = max_speed_multiplier * max_speed_corr;

        JointDrive xdriveTL = wheel_joint.angularXDrive;
        xdriveTL.maximumForce = 1e+28f;
        xdriveTL.positionSpring = 0;
        xdriveTL.positionDamper = max_torque;
        wheel_joint.angularXDrive = xdriveTL;

        JointDrive yzdriveTL = wheel_joint.angularYZDrive;
        yzdriveTL.maximumForce = 1e+28f;
        yzdriveTL.positionSpring = 0;
        yzdriveTL.positionDamper = max_torque;
        wheel_joint.angularYZDrive = yzdriveTL;

        wheel_joint.targetAngularVelocity = new Vector3(0,0,0);
    }

    [HideInInspector()] public bool gamepad1_a;
    [HideInInspector()] public bool gamepad1_x;
    [HideInInspector()] public bool gamepad1_y;
    [HideInInspector()] public bool gamepad1_b;
    public float gamepad1_right_stick_x;
    public float gamepad1_right_stick_y;
    public float gamepad1_left_stick_x;
    public float gamepad1_left_stick_y;
    [HideInInspector()] public bool gamepad1_dpad_down;
    [HideInInspector()] public bool gamepad1_dpad_up;
    [HideInInspector()] public bool gamepad1_dpad_right;
    [HideInInspector()] public bool gamepad1_dpad_left;
    [HideInInspector()] public bool gamepad1_right_bumper;
    [HideInInspector()] public bool gamepad1_left_bumper;
    [HideInInspector()] public float gamepad1_left_trigger;
    [HideInInspector()] public float gamepad1_right_trigger;

    [HideInInspector()] public bool gamepad1_stop = false;
    [HideInInspector()] public bool gamepad1_restart = false;
    [HideInInspector()] public bool gamepad1_reset = false;

    [HideInInspector()] public bool gamepad1_a_changed = false;
    [HideInInspector()] public bool gamepad1_b_changed = false;
    [HideInInspector()] public bool gamepad1_x_changed = false;
    [HideInInspector()] public bool gamepad1_y_changed = false;
    [HideInInspector()] public bool gamepad1_right_bumper_changed = false;
    [HideInInspector()] public bool gamepad1_left_bumper_changed = false;

    [HideInInspector()] public bool gamepad1_a_previous = false;
    [HideInInspector()] public bool gamepad1_b_previous = false;
    [HideInInspector()] public bool gamepad1_x_previous = false;
    [HideInInspector()] public bool gamepad1_y_previous = false;
    [HideInInspector()] public bool gamepad1_right_bumper_previous = false;
    [HideInInspector()] public bool gamepad1_left_bumper_previous = false;

    [HideInInspector()] public bool gamepad1_dpad_down_old = false;
    [HideInInspector()] public bool gamepad1_dpad_up_old = false;
    [HideInInspector()] public bool gamepad1_dpad_left_old = false;
    [HideInInspector()] public bool gamepad1_dpad_right_old = false;
    [HideInInspector()] public bool gamepad1_stop_old = false;
    [HideInInspector()] public bool gamepad1_restart_old = false;
    [HideInInspector()] public bool gamepad1_reset_old = false;

    [HideInInspector()] public bool gamepad1_dpad_down_changed = false;
    [HideInInspector()] public bool gamepad1_dpad_up_changed = false;
    [HideInInspector()] public bool gamepad1_dpad_left_changed = false;
    [HideInInspector()] public bool gamepad1_dpad_right_changed = false;
    [HideInInspector()] public bool gamepad1_stop_changed = false;
    [HideInInspector()] public bool gamepad1_restart_changed = false;
    [HideInInspector()] public bool gamepad1_reset_changed = false;
    [HideInInspector()] public float gamepad1_right_stick_y_old = 0;
    [HideInInspector()] public float gamepad1_right_stick_x_old = 0;
    [HideInInspector()] public float gamepad1_left_stick_x_old = 0;
    [HideInInspector()] public float gamepad1_left_stick_y_old = 0;


    public long time_last_button_activitiy = 0;

    public float turning_scaler = 1f;
    public float precision_scaler = 0.3f;


    // ********* updateGamepadVars ***********
    // Updates the state of all the controller buttons and provides the key mappings here.
    // While these game variables are labeled after the intended default mapping, the end user may
    // remap these to be tottaly different.
    // 
    // In the options menu, "movement" and "turning" buttons are presented. These indicate final function,
    // not just buttons-to-controller mappings. To support this, please have the following convention:
    //  var gamepad1_left_stick = this will be responsible for MOVEMENT including straffing. +1 means bottom-right
    //  var gamepad1_right_stick_x = this will be responsible for turning. +1 means clockwise.
    // 
    public void updateControlsFromHash(Dictionary<string,string> inputs)
    {
        foreach( string currbutton in inputs.Keys)
        {
            switch(currbutton)
            {
                case "a":  
                    gamepad1_a = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "b":
                    gamepad1_b = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "x":
                    gamepad1_x = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "y":
                    gamepad1_y = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "dpad_down":
                    gamepad1_dpad_down = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "dpad_up":
                    gamepad1_dpad_up = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "dpad_left":
                    gamepad1_dpad_left = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "dpad_right":
                    gamepad1_dpad_right = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "bumper_l":
                    gamepad1_left_bumper = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "bumper_r":
                    gamepad1_right_bumper = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "stop":
                    gamepad1_stop = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "restart":
                    gamepad1_restart = MyUtils.GetBoolFromString(inputs[currbutton]);
                    break;

                case "right_y":
                    gamepad1_right_stick_y = MyUtils.GetFloatFromString(inputs[currbutton]);
                    break;

                case "right_x":
                    gamepad1_right_stick_x = MyUtils.GetFloatFromString(inputs[currbutton]);
                    break;

                case "left_y":
                    gamepad1_left_stick_y = MyUtils.GetFloatFromString(inputs[currbutton]);
                    break;

                case "left_x":
                    gamepad1_left_stick_x = MyUtils.GetFloatFromString(inputs[currbutton]);
                    break;

                case "trigger_l":
                    gamepad1_left_trigger = MyUtils.GetFloatFromString(inputs[currbutton]);
                    break;

                case "trigger_r":
                    gamepad1_right_trigger = MyUtils.GetFloatFromString(inputs[currbutton]);
                    break;

                case "precision":
                    precision_scaler = MyUtils.GetFloatFromString(inputs[currbutton], 0f, 4f);
                    break;
            }


        }
    }

    public void updateGamepadVars()
    {

        // First update all the variables
        // Basic Buttons
        gamepad1_a = GLOBALS.JoystickMap["Jcontrolls_A"].GetButton();
        gamepad1_b = GLOBALS.JoystickMap["Jcontrolls_B"].GetButton();
        gamepad1_x = GLOBALS.JoystickMap["Jcontrolls_X"].GetButton();
        gamepad1_y = GLOBALS.JoystickMap["Jcontrolls_Y"].GetButton();

        // Basic Movement
        // Right stick y is not really used
        // ASSIGNMENTS: left_x is strafe, left_y is forward/backward, right_x is turn
        gamepad1_right_stick_y = GLOBALS.JoystickMap["Jcontrolls_right_y"].GetAxis();
        gamepad1_right_stick_x = GLOBALS.JoystickMap["Jcontrolls_turn"].GetAxis() * turning_scaler;
        gamepad1_left_stick_x = GLOBALS.JoystickMap["Jcontrolls_move_lr"].GetAxis() * turning_scaler;
        gamepad1_left_stick_y = GLOBALS.JoystickMap["Jcontrolls_move_ud"].GetAxis();

        // DPAD
        gamepad1_dpad_down = GLOBALS.JoystickMap["Jcontrolls_dpad_ud"].GetAxis() > 0.5f;
        gamepad1_dpad_up = GLOBALS.JoystickMap["Jcontrolls_dpad_ud"].GetAxis() < -0.5f;
        gamepad1_dpad_left = GLOBALS.JoystickMap["Jcontrolls_dpad_lr"].GetAxis() < -0.5f;
        gamepad1_dpad_right = GLOBALS.JoystickMap["Jcontrolls_dpad_lr"].GetAxis() > 0.5f;

        // Others
        gamepad1_right_bumper = GLOBALS.JoystickMap["Jcontrolls_RB"].GetButton();
        gamepad1_left_bumper = GLOBALS.JoystickMap["Jcontrolls_LB"].GetButton();
        gamepad1_left_trigger = GLOBALS.JoystickMap["Jcontrolls_LTR"].GetAxis();
        gamepad1_right_trigger = GLOBALS.JoystickMap["Jcontrolls_RTR"].GetAxis();


        gamepad1_stop = GLOBALS.JoystickMap["Jcontrolls_stop"].GetButton();
        gamepad1_restart = GLOBALS.JoystickMap["Jcontrolls_restart"].GetButton();

        // Now add the keyboard settings (which override the controller)
        // Shift will decrease turning to 1/nth
        if (!GLOBALS.keyboard_inuse)
        {
            if (Input.GetKey(KeyCode.LeftShift)) {
                gamepad1_left_bumper = Input.GetKey(KeyCode.LeftShift);
            }
            if (Input.GetKey(KeyCode.RightShift)) {

                gamepad1_right_bumper = Input.GetKey(KeyCode.RightShift);
            }

            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_A"].key)) { gamepad1_a = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_B"].key)) { gamepad1_b = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_X"].key)) { gamepad1_x = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_Y"].key)) { gamepad1_y = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_LD"].key)) 
                { gamepad1_left_trigger = 1f; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_RD"].key)) { gamepad1_right_trigger = 1f; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_dpad_u"].key)) { gamepad1_dpad_up = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_dpad_d"].key)) { gamepad1_dpad_down = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_dpad_l"].key)) { gamepad1_dpad_left = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_dpad_r"].key)) { gamepad1_dpad_right = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_move_u"].key)) { gamepad1_left_stick_y = -1f; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_move_d"].key)) { gamepad1_left_stick_y = 1f; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_move_l"].key)) { gamepad1_left_stick_x = -1f * turning_scaler; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_move_r"].key)) { gamepad1_left_stick_x = 1f * turning_scaler; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_turn_l"].key)) { gamepad1_right_stick_x = -1f * turning_scaler; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_turn_r"].key)) { gamepad1_right_stick_x = 1f * turning_scaler; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_stop"].key)) { gamepad1_stop = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_restart"].key)) { gamepad1_restart = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_reset"].key)) { gamepad1_reset = true; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_rightstick_up"].key)) { gamepad1_right_stick_y = -1f; }
            }
            catch (Exception e) { }
            try
            {
                if (Input.GetKey(GLOBALS.KeyboardMap["controlls_rightstick_down"].key)) { gamepad1_right_stick_y = 1f; }
            }
            catch (Exception e) { }
        }
    }


    // Update internal variables to make life easier
    // Fixed: now runs right before Update_Robot
    public void InputsChanges()
    {
        bool something_changed = false;

        // Finally, calculate events to make Robot programming easier
        gamepad1_a_changed = gamepad1_a != gamepad1_a_previous;
        something_changed |= gamepad1_a_changed;
        gamepad1_b_changed = gamepad1_b != gamepad1_b_previous;
        something_changed |= gamepad1_b_changed;
        gamepad1_x_changed = gamepad1_x != gamepad1_x_previous;
        something_changed |= gamepad1_x_changed;
        gamepad1_y_changed = gamepad1_y != gamepad1_y_previous;
        something_changed |= gamepad1_y_changed;

        something_changed |= Math.Abs(gamepad1_right_stick_y - gamepad1_right_stick_y_old) > 0.01f;
        something_changed |= Math.Abs(gamepad1_right_stick_x - gamepad1_right_stick_x_old) > 0.01f;
        something_changed |= Math.Abs(gamepad1_left_stick_x - gamepad1_left_stick_x_old) > 0.01f;
        something_changed |= Math.Abs(gamepad1_left_stick_y - gamepad1_left_stick_y_old) > 0.01f;

        gamepad1_dpad_down_changed = gamepad1_dpad_down != gamepad1_dpad_down_old;
        something_changed |= gamepad1_dpad_down_changed;
        gamepad1_dpad_up_changed = gamepad1_dpad_up != gamepad1_dpad_up_old;
        something_changed |= gamepad1_dpad_up_changed;
        gamepad1_dpad_left_changed = gamepad1_dpad_left != gamepad1_dpad_left_old;
        something_changed |= gamepad1_dpad_left_changed;
        gamepad1_dpad_right_changed = gamepad1_dpad_right != gamepad1_dpad_right_old;
        something_changed |= gamepad1_dpad_right_changed;
        gamepad1_stop_changed = gamepad1_stop != gamepad1_stop_old;
        something_changed |= gamepad1_stop_changed;
        gamepad1_restart_changed = gamepad1_restart != gamepad1_restart_old;
        something_changed |= gamepad1_restart_changed;
        gamepad1_reset_changed = gamepad1_reset != gamepad1_reset_old;
        something_changed |= gamepad1_reset_changed;

        gamepad1_right_bumper_changed = gamepad1_right_bumper != gamepad1_right_bumper_previous;
        something_changed |= gamepad1_right_bumper_changed;
        gamepad1_left_bumper_changed = gamepad1_left_bumper != gamepad1_left_bumper_previous;
        something_changed |= gamepad1_left_bumper_changed;

        // Save current states
        gamepad1_a_previous = gamepad1_a;
        gamepad1_b_previous = gamepad1_b;
        gamepad1_x_previous = gamepad1_x;
        gamepad1_y_previous = gamepad1_y;
        gamepad1_right_bumper_previous = gamepad1_right_bumper;
        gamepad1_left_bumper_previous = gamepad1_left_bumper;
        gamepad1_dpad_down_old = gamepad1_dpad_down;
        gamepad1_dpad_up_old = gamepad1_dpad_up;
        gamepad1_dpad_left_old = gamepad1_dpad_left;
        gamepad1_dpad_right_old = gamepad1_dpad_right;
        gamepad1_stop_old = gamepad1_stop;
        gamepad1_restart_old = gamepad1_restart;
        gamepad1_reset_old = gamepad1_reset;

        // Adjust turning scaler if so required
        turning_scaler = 1f;
        if (gamepad1_left_bumper || gamepad1_right_bumper) { turning_scaler = precision_scaler; }

        if (something_changed || (time_last_button_activitiy == 0))
        {
            time_last_button_activitiy = MyUtils.GetTimeMillis();
        }

    }


    private double Deg2Rad = Math.PI / 180f;

    [Tooltip("Maximum speed in ft/s")]
    public float max_speed = 20f;               // Sets the no-force max-speed of wheel - in ft/s
    public bool max_speed_lock = false;
    public float max_speed_corr = 20f * 6.44f; // Internal max_speed that is unit conversation from real-world to unity

    [Tooltip("Maximum acceleration in ft/s2")]
    public float max_acceleration = 5f;         // Sets the maximum acceleration in ft/s^2
    public bool max_acceleration_lock = false;
    public float max_torque = 1000f;           // Sets the stall torque of each wheel

    [Tooltip("Allows turnign to be faster/slower")]
    public float turn_scale = 1f;               // Allow turning to be at different speed (mecanum or joytstick comfort)
    public bool turn_scale_lock = false;

    // Spring dampening is actually the force of the motor drive (inversily proportional to the max speed)
    private float friction_torque_scaler = 1;       // The torque required to get the desired friciton forces. We calculate it based on weight in code below.
    public float straffing_friction_scaler = 1.5f;  // The friction applied in straffing direction: usually higher for mecanum wheels, set to 1.5f by default
    public float default_regen_breaking = 0.25f;    // regenerative breaking scaling factor for when activebreaking is turned off. Treated as friction (fixed force) as opposed to motor power (lienarly decreasing force with speed).


    // Basically motor force goes away linearly as you approach the maximum velocity, while position force tries to move to wheel to the target position
    // From physx: force = spring* (targetPosition - position) + damping* (targetVelocity - velocity)
    // where: spring = position_force_scalar * calculated_torque
    //        damping = motor_force_scalar * calculated_torque
    // Since we are not using position mode, than spring = 0;

    private Vector3 forceTL, forceBL, forceTR, forceBR; // Final nromalized forced, put here so they can be observed for debugging if put public

    // Cache of key parameters
    private JointDrive xdriveTL, yzdriveTL, xdriveTR, yzdriveTR, xdriveBL, yzdriveBL, xdriveBR, yzdriveBR, xdriveML, yzdriveML, xdriveMR, yzdriveMR;
    private Vector3 straffing_vec = new Vector3(0, 0, 1f);

    public Vector3 TL_current;
    public Vector3 TL_goaly;

    // Used to transform angular velocity from the real-world space to one that would be applied 
    // to the wheel springs.
    private Vector3 TransformWorldVelToWheelVel(Vector3 worldvector)
    {
        // Unfortunetaly the wheels have been grandfathered into starting off with 90-degree counter-clockwise orientation to the body.
        // This means body's x is the wheels'z z, and the body's z is the wheel's -x.
        // All goals are specified in the wheel's initial frame-of-reference. Since those wheels are rotating, we can't use them to reverse-transform.
        // we will therefore use the "Body" frame of reference for reverse transform, but need to switch the values around as described above.

        Vector3 body_reference = rb_body.transform.InverseTransformDirection(worldvector);

        // Now translate it to wheel reference
        return new Vector3(-1f * body_reference.z, body_reference.y, body_reference.x);
    }

    // Assumes that if wheel is a real object, it then has to have a Rigidbodyu
    private Vector3 TransformWorldVelToWheelVel(GameObject wheel )
    {
        // Make sure there is something
        if(wheel == null) { return Vector3.zero; }

        // Return value
        return TransformWorldVelToWheelVel(wheel.GetComponent<Rigidbody>().angularVelocity);
    }

    private Vector3 diagonal_rotation = new Vector3(1f, 0, 1f);

    public float ML_torque_multiplier = 1f;
    public float MR_torque_multiplier = 1f;
    private void ApplyWheelSpringForces(Vector3 TL_goal, Vector3 TR_goal, Vector3 BL_goal, Vector3 BR_goal)
    {
        if (!rb_body || !rb_body.gameObject.activeSelf) { return; }

        // Clear old sound data
        averageMovementForceMagnitude = 0f;
        averageVelocityMagnitude = 0f;

        // If robot is in a holding position, don't do anything
        // Holding is used when the robot is in a penalty timeout
        if (hold_position) { return; }

        // ***********************************************
        // Calculations are done on 4 wheels
        //  Top + Bottom Left+Right.
        // If it's tank drive, than Top forces are zeroed out, bottom forces are doubled.


        // Get current angular velocities of the spring
        Vector3 TL_curr = TransformWorldVelToWheelVel(wheelTL.GetComponent<Rigidbody>().angularVelocity);
        Vector3 TR_curr = TransformWorldVelToWheelVel(wheelTR.GetComponent<Rigidbody>().angularVelocity);
        Vector3 BL_curr = TransformWorldVelToWheelVel(wheelBL.GetComponent<Rigidbody>().angularVelocity);
        Vector3 BR_curr = TransformWorldVelToWheelVel(wheelBR.GetComponent<Rigidbody>().angularVelocity);
        Vector3 ML_curr = new Vector3();
        Vector3 MR_curr = new Vector3();

        if (wheelML && wheelMR && DriveTrain == "6-Wheel Tank")
        {
            ML_curr = TransformWorldVelToWheelVel(wheelML.GetComponent<Rigidbody>().angularVelocity);
            MR_curr = TransformWorldVelToWheelVel(wheelMR.GetComponent<Rigidbody>().angularVelocity);
        }

        TL_current = TL_curr;
        TL_goaly = TL_goal;

        // Set the max torques
        float TL_torque = max_torque;
        float TR_torque = max_torque;
        float BL_torque = max_torque;
        float BR_torque = max_torque;
        float ML_torque = max_torque;
        float MR_torque = max_torque;

        // Determine if we should be adding regenerative breaking 
        // ******** Measurement of robot behavior on field shows this breaking looks like a fixed force, thus it's less likelly caused
        // ******** by regeneration, but more by the powerloss in the gears??? Not sure at this point, but will follow lab measurement
        //
        // Some more explanation: The rev hub controls the motors as a half-bridge - what this means is that it can easily add power to the
        // wheel, but reducing power is poor UNLESS it switches into "reverse direction" mode. Thus if you just reduce forward power it will
        // coast into the new speed at a ~25% regenerative braking, but if you apply even a little bit of reverse direction power, it now switches
        // mode and will be using full motor power to reduce speed.
        //
        // This is too complicated given our wheels are modeled as a sphere rotating in 2 dimensions. Will simplify this to as follows:
        // 1) If magnitude is reduced, we will coast
        // 2) If the difference in Vectors is more than 10% higher than original (e.g. we reversed direction), then apply full motor power
        // Do keep in mind that the "goal" versus actual speed attained are different because of friction forces
        float TL_regen = 0;
        if (!activebreaking && (TL_goal.magnitude < 0.9f * TL_curr.magnitude) && ((TL_goal - TL_curr).magnitude < 1.1f * TL_curr.magnitude))  // If decelerating, rely on friction forces only (no motor spring)
        {
            TL_regen = default_regen_breaking;
            TL_torque = 0f;
        }

        float TR_regen = 0;
        if (!activebreaking && (TR_goal.magnitude < 0.9f * TR_curr.magnitude) && ((TR_goal - TR_curr).magnitude < 1.1f * TR_curr.magnitude))
        {
            TR_regen = default_regen_breaking;
            TR_torque = 0f;
        }

        float BL_regen = 0;
        if (!activebreaking && (BL_goal.magnitude < 0.9f * BL_curr.magnitude) && ((BL_goal - BL_curr).magnitude < 1.1f * BL_curr.magnitude))
        {
            BL_regen = default_regen_breaking;
            BL_torque = 0;
        }

        float BR_regen = 0;
        if (!activebreaking && (BR_goal.magnitude < 0.9f * BR_curr.magnitude) && ((BR_goal - BR_curr).magnitude < 1.1f * BR_curr.magnitude))
        {
            BR_regen = default_regen_breaking;
            BR_torque = 0;
        }


        if (DriveTrain == "Tank")
        {
            // Make sure final goals are off
            TL_goal.x = 0; TL_goal.y = 0; TL_goal.z = 0;
            TR_goal.x = 0; TR_goal.y = 0; TR_goal.z = 0;

            TL_torque = 0f;
            TR_torque = 0f;
            TL_regen = 0f;
            TR_regen = 0f;

            // Double the BL/BR torques since all 4 motors are on those wheels
            BL_torque *= 2f;
            BR_torque *= 2f;
        }

        Vector3 ML_goal = new Vector3();
        Vector3 MR_goal = new Vector3();
        float ML_regen = 0;
        float MR_regen = 0;
        ML_torque_multiplier = 1f;
        MR_torque_multiplier = 1f;


        if (DriveTrain == "6-Wheel Tank")
        {
            TL_goal.y = 0; TL_goal.z = 0;
            TR_goal.y = 0; TR_goal.z = 0;
            BL_goal.y = 0; BL_goal.z = 0;
            BR_goal.y = 0; BR_goal.z = 0;

            TL_goal.x = BL_goal.x;
            TR_goal.x = BR_goal.x;


            // Assign ML goals to be average top top/bottom
            ML_goal = (TL_goal + BL_goal) / 2f;
            MR_goal = (TR_goal + BR_goal) / 2f;

            // Use the average for T/B torques for middle
            TL_torque *= 1f; // Used to be 0.666f for all)
            BL_torque *= 1f;
            TR_torque *= 1f;
            BR_torque *= 1f;

            ML_torque = (TL_torque + BL_torque) / 2f;
            MR_torque = (TR_torque + BR_torque) / 2f;

            // Apply breaking if necessary
            if (!activebreaking && (ML_goal.magnitude < 0.9f * ML_curr.magnitude) && ((ML_goal - ML_curr).magnitude < 1.1f * ML_curr.magnitude))
            {
                ML_regen = default_regen_breaking;
                ML_torque = 0;
            }

            if (!activebreaking && (MR_goal.magnitude < 0.9f * MR_curr.magnitude) && ((MR_goal - MR_curr).magnitude < 1.1f * MR_curr.magnitude))
            {
                MR_regen = default_regen_breaking;
                MR_torque = 0;
            }



            // Finally if the speed of the wheel is small and we are trying to make it go, increase the torques
            if ((TL_goal.magnitude > 0.2f * max_speed_corr) && (TL_curr.magnitude < 0.05f * max_speed_corr))
            {
                TL_torque *= 1f + (1 - TL_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((TR_goal.magnitude > 0.2f * max_speed_corr) && (TR_curr.magnitude < 0.05f * max_speed_corr))
            {
                TR_torque *= 1f + (1 - TR_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((ML_goal.magnitude > 0.2f * max_speed_corr) && (ML_curr.magnitude < 0.05f * max_speed_corr))
            {
                ML_torque *= 1f + (1 - ML_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((MR_goal.magnitude > 0.2f * max_speed_corr) && (MR_curr.magnitude < 0.05f * max_speed_corr))
            {
                MR_torque *= 1f + (1 - MR_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((BL_goal.magnitude > 0.2f * max_speed_corr) && (BL_curr.magnitude < 0.05f * max_speed_corr))
            {
                BL_torque *= 1f + (1 - BL_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((BR_goal.magnitude > 0.2f * max_speed_corr) && (BR_curr.magnitude < 0.05f * max_speed_corr))
            {
                BR_torque *= 1f + (1 - BR_curr.magnitude / (0.05f * max_speed_corr));
            }
        }

        if (DriveTrain == "Mecanum")
        {
            // With mecanum wheels we need to reduce the speed applied at 45 degrees
            // May want to increase the torque (since mecanums have higher torque at 45 degrees)
            // Vector3.Angle is a positiove only value between 0 and 180 returning the smallest of the number;
            // Since we want the function to repeat every 90 degrees, we need to multiply angle by 4 when taking the Sin of it
            float TL_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(TL_goal, diagonal_rotation) * 3.1415926f / 180f));
            float TR_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(TR_goal, diagonal_rotation) * 3.1415926f / 180f));
            float BL_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(BL_goal, diagonal_rotation) * 3.1415926f / 180f));
            float BR_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(BR_goal, diagonal_rotation) * 3.1415926f / 180f));

            // reduce top speeds by how close we are to the diagonal
            TL_goal *= 1f / (1f + 0.7f * TL_diag_scalar);
            TR_goal *= 1f / (1f + 0.7f * TR_diag_scalar);
            BL_goal *= 1f / (1f + 0.7f * BL_diag_scalar);
            BR_goal *= 1f / (1f + 0.7f * BR_diag_scalar);

            // Next increase torque based on how much we reduce the speed by
            // Since our target velocity was reduced, increasing the torque only offsets the acc/decc to be back to normal,
            // but our applied torque at the lower speed needs to be higher than the previous full speed. Thus we need to square the torque increase.
            TL_torque *= (float)Math.Pow(1f + 0.7f * TL_diag_scalar, 2f);
            TR_torque *= (float)Math.Pow(1f + 0.7f * TR_diag_scalar, 2f);
            BL_torque *= (float)Math.Pow(1f + 0.7f * BL_diag_scalar, 2f);
            BR_torque *= (float)Math.Pow(1f + 0.7f * BR_diag_scalar, 2f);
        }

        // NOTE: the positionDamper is a force that is proportional to speed, which is exactly what motors do,
        // Thus we are using the positionDamper as the motor driver.
        // From unity manual: force = PositionSpring * (target position - position) + PositionDamper * (targetVelocity - velocity)
        // So we want PositionDamper to be the torque
        xdriveTL = wheelTL_joint.angularXDrive;
        xdriveTL.positionDamper = TL_torque;
        wheelTL_joint.angularXDrive = xdriveTL;

        yzdriveTL = wheelTL_joint.angularYZDrive;
        yzdriveTL.positionDamper = TL_torque;
        wheelTL_joint.angularYZDrive = yzdriveTL;

        xdriveTR = wheelTR_joint.angularXDrive;
        xdriveTR.positionDamper = TR_torque;
        wheelTR_joint.angularXDrive = xdriveTR;

        yzdriveTR = wheelTR_joint.angularYZDrive;
        yzdriveTR.positionDamper = TR_torque;
        wheelTR_joint.angularYZDrive = yzdriveTR;

        xdriveBL = wheelBL_joint.angularXDrive;
        xdriveBL.positionDamper = BL_torque;
        wheelBL_joint.angularXDrive = xdriveBL;

        yzdriveBL = wheelBL_joint.angularYZDrive;
        yzdriveBL.positionDamper = BL_torque;
        wheelBL_joint.angularYZDrive = yzdriveBL;

        xdriveBR = wheelBR_joint.angularXDrive;
        xdriveBR.positionDamper = BR_torque;
        wheelBR_joint.angularXDrive = xdriveBR;

        yzdriveBR = wheelBR_joint.angularYZDrive;
        yzdriveBR.positionDamper = BR_torque;
        wheelBR_joint.angularYZDrive = yzdriveBR;

        wheelTL_joint.targetAngularVelocity = TL_goal;
        wheelTR_joint.targetAngularVelocity = TR_goal;
        wheelBL_joint.targetAngularVelocity = BL_goal;
        wheelBR_joint.targetAngularVelocity = BR_goal;

        if (DriveTrain == "6-Wheel Tank")
        {
            xdriveML = wheelML_joint.angularXDrive;
            xdriveML.positionDamper = ML_torque;
            wheelML_joint.angularXDrive = xdriveML;

            yzdriveML = wheelML_joint.angularYZDrive;
            yzdriveML.positionDamper = ML_torque;
            wheelML_joint.angularYZDrive = yzdriveML;

            xdriveMR = wheelMR_joint.angularXDrive;
            xdriveMR.positionDamper = MR_torque;
            wheelMR_joint.angularXDrive = xdriveMR;

            yzdriveMR = wheelMR_joint.angularYZDrive;
            yzdriveMR.positionDamper = MR_torque;
            wheelMR_joint.angularYZDrive = yzdriveMR;

            wheelML_joint.targetAngularVelocity = ML_goal;
            wheelMR_joint.targetAngularVelocity = MR_goal;
        }



        //** SOUND CALCULATIONS *
        averageMovementForceMagnitude = (TL_goal.magnitude + TR_goal.magnitude +
                BL_goal.magnitude + BR_goal.magnitude) / 4f / max_speed_corr;
        averageVelocityMagnitude = (TL_curr.magnitude + TR_curr.magnitude +
                BL_curr.magnitude + BR_curr.magnitude) / 4f / max_speed_corr;
        // **********************

        // Now apply friction forces
        // friction_torque tries to apply a force that reduces speed of the robot by a fixed ft/s^2 regardless of weight
        // In general, straffing exhibits a much higher friction. 
        // First just add general friction against all dimensions
        wheelTL_body.AddTorque(-wheelTL_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + TL_regen * max_acceleration));
        wheelBL_body.AddTorque(-wheelBL_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + BL_regen * max_acceleration));
        wheelTR_body.AddTorque(-wheelTR_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + TR_regen * max_acceleration));
        wheelBR_body.AddTorque(-wheelBR_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + BR_regen * max_acceleration));

        if (wheelML_body && wheelMR_body && DriveTrain == "6-Wheel Tank")
        {
            wheelML_body.AddTorque(-wheelML_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + ML_regen * max_acceleration));
            wheelMR_body.AddTorque(-wheelMR_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + MR_regen * max_acceleration));
        }

        // Next add friction against straffing 
        // z-axis is going forward, thus rotation around x-axis is the direction of rotation
        // Also need to consider rb_body may be rotated..
        if (DriveTrain == "Mecanum")
        {
            Vector3 friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelTL_body.angularVelocity).x, 0, 0);
            wheelTL_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);

            friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelTR_body.angularVelocity).x, 0, 0);
            wheelTR_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);

            friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelBR_body.angularVelocity).x, 0, 0);
            wheelBR_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);

            friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelBL_body.angularVelocity).x, 0, 0);
            wheelBL_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);
        }
        return;
    }


    // New version of ApplyWheelForces
    // Tries to fix friction issues from before, and apply correct orthogonal resistance when pushed

    // Variables the remember the direction the wheels points, important for Swerve drive
    // A value of 0 indicates omni-wheel
    public Vector3 wheel_dir_TL = new Vector3(1f, 0, 0);
    public Vector3 wheel_dir_TR = new Vector3(1f, 0, 0);
    public Vector3 wheel_dir_BL = new Vector3(1f, 0, 0);
    public Vector3 wheel_dir_BR = new Vector3(1f, 0, 0);
    public Vector3 wheel_dir_ML = new Vector3(0, 0, 0);
    public Vector3 wheel_dir_MR = new Vector3(0, 0, 0);
    // NOTE: Middle wheels always point forward 1f so no need for variables to store their direciton, but here for consitancy.
    // Set to 0f (omniwheels) by default


    public Vector3 TL_curr;
    public Vector3 targetv_tl;
    public Vector3 TR_curr;
    public Vector3 targetv_tr;
    public Vector3 BL_curr;
    public Vector3 targetv_bl;
    public Vector3 BR_curr;
    public Vector3 targetv_br;
    public Vector3 ML_curr;
    public Vector3 targetv_ml;
    public Vector3 MR_curr;
    public Vector3 targetv_mr;



    public Vector3 ortho3_vel_tl;
    public Vector3 ortho3_vel_tr;
    public Vector3 ortho3_vel_bl;
    public Vector3 ortho3_vel_br;
    public Vector3 ortho3_vel_ml;
    public Vector3 ortho3_vel_mr;

    //private void ApplyWheelSpringForces_new(Vector3 TL_goal, Vector3 TR_goal, Vector3 BL_goal, Vector3 BR_goal)
    private void ApplyWheelSpringForces_new(Vector3 dir_target)
    {
        // ******************************************
        // Work in progress: not ready to be used
        // looking at improving/cleaning up spring forces. However, fighting with unity apply-force issues is making this into a project
        // For now, this is put on hold until later.

        if (!rb_body || !rb_body.gameObject.activeSelf) { return; }

        // Clear old sound data
        averageMovementForceMagnitude = 0f;
        averageVelocityMagnitude = 0f;

        // If robot is in a holding position, don't do anything
        // Holding is used when the robot is in a penalty timeout
        if (hold_position) { return; }

        // *************************************************
        // Determine final target velocities of each wheel.
        // This is drive train dependent
        Vector3 target_tl = new Vector3(0, 0, 0);
        Vector3 target_tr = new Vector3(0, 0, 0);
        Vector3 target_bl = new Vector3(0, 0, 0);
        Vector3 target_br = new Vector3(0, 0, 0);
        Vector3 target_ml = new Vector3(0, 0, 0);
        Vector3 target_mr = new Vector3(0, 0, 0);

        // Translate desired direction to individual wheels
        // Aslo while we're at it, calculate the maximum torques
        float TL_torque = 0;
        float TR_torque = 0;
        float BL_torque = 0;
        float BR_torque = 0;
        float ML_torque = 0;
        float MR_torque = 0;
        float rootscaler = 1 / Mathf.Sqrt(2);

        bool ortho_locked_top = false;
        bool ortho_locked_mid = false;
        bool ortho_locked_bot = false;


        switch (DriveTrain )
        {
            case "Tank":
                // This has driven back wheels, omni fron wheels
                // The back wheels has angular motion constrained around only x-axis, thus don't try to cancel orthogonal forces
                target_bl.x = dir_target.z * turn_scale + dir_target.y;
                target_br.x = 1f * dir_target.z * turn_scale + dir_target.y;
                wheel_dir_TL = Vector3.zero;
                wheel_dir_TR = Vector3.zero;

                // Double toruqe on 2 wheels since max_torque assumes 4 wheels
                BL_torque = 2f * max_torque;
                BR_torque = 2f * max_torque;

                // Mark bottom are locked
                ortho_locked_bot = true;
                break;

            case "6-Wheel Tank":
                target_ml.x = dir_target.z * turn_scale + dir_target.y;
                target_mr.x = -1f * dir_target.z * turn_scale + dir_target.y;
                // Front and back wheels become omni wheels
                //wheel_dir_TL = Vector3.zero;
                //wheel_dir_TR = Vector3.zero;
                //wheel_dir_BL = Vector3.zero;
                //wheel_dir_BR = Vector3.zero;
                wheel_dir_ML.x = 1f;
                wheel_dir_MR.x = 1f;

                // Double toruqe on 2 wheels since max_torque assumes 4 wheels
                ML_torque = 2f * max_torque;
                MR_torque = 2f * max_torque;

                ortho_locked_mid = true;
                break;

            default: // "Mecanum", "Swerve", etc..
                     // First apply forward 
                target_tl.x = dir_target.y;
                target_tr.x = dir_target.y;
                target_bl.x = dir_target.y;
                target_br.x = dir_target.y;

                // Apply Straffing (if there is any)
                target_tl.z = dir_target.x;
                target_tr.z = dir_target.x;
                target_bl.z = dir_target.x;
                target_br.z = dir_target.x;

                // Apply rotation
                // For this rotation in place would be all wheres going 45Degrees around center. Thus
                // Apply rotation force as a 45 Degree rotational force who's magnitude is 1/sqrt(2) in the individual domains (Net magnitude of 1)
                
                target_tl.x += rootscaler * dir_target.z;
                target_tl.z += -1f * rootscaler * dir_target.z;

                target_tr.x += -1f * rootscaler * dir_target.z;
                target_tr.z += -1f * rootscaler * dir_target.z;

                target_br.x += -1f * rootscaler * dir_target.z;
                target_br.z += rootscaler * dir_target.z;

                target_bl.x += rootscaler * dir_target.z;
                target_bl.z += rootscaler * dir_target.z;

                // Targets point in direction of wheels.
                // If 0 magnitude, remember old position
                if( target_tl.magnitude != 0f) { wheel_dir_TL = target_tl.normalized; }
                if (target_tr.magnitude != 0f) { wheel_dir_TR = target_tr.normalized; }
                if (target_bl.magnitude != 0f) { wheel_dir_BL = target_bl.normalized; }
                if (target_br.magnitude != 0f) { wheel_dir_BR = target_br.normalized; }

                TL_torque = max_torque;
                TR_torque = max_torque;
                BL_torque = max_torque;
                BR_torque = max_torque;

                break;
        }

        // ****************************************
        // Limit max powers

        // Now limit the wheels power: we can limit each individual, or scale all together.
        bool limit_power_individually = true;

        if (limit_power_individually)
        {
            if (target_tl.magnitude > 1f) { target_tl.Normalize(); }
            if (target_tr.magnitude > 1f) { target_tr.Normalize(); }
            if (target_bl.magnitude > 1f) { target_bl.Normalize(); }
            if (target_br.magnitude > 1f) { target_br.Normalize(); }
            if (target_ml.magnitude > 1f) { target_ml.Normalize(); }
            if (target_mr.magnitude > 1f) { target_mr.Normalize(); }
        }
        else
        {
            // Find the maximum magnitude
            // Scale down if it will exceed 1
            float correct_for_excessive_speed = 1.0f / Math.Max(Math.Max(Math.Max(Math.Max(Math.Max(Math.Max(
                target_tl.magnitude,
                target_tr.magnitude),
                target_bl.magnitude),
                target_br.magnitude),
                target_ml.magnitude),
                target_mr.magnitude),
                1f
                );

            target_tl *= correct_for_excessive_speed;
            target_tr *= correct_for_excessive_speed;
            target_bl *= correct_for_excessive_speed;
            target_br *= correct_for_excessive_speed;
            target_ml *= correct_for_excessive_speed;
            target_mr *= correct_for_excessive_speed;
        }

        // ************************************
        // Get final target velocity
         targetv_tl = max_speed_corr * target_tl;
         targetv_tr = max_speed_corr * target_tr;
         targetv_bl = max_speed_corr * target_bl;
         targetv_br = max_speed_corr * target_br;
         targetv_ml = max_speed_corr * target_ml;
         targetv_mr = max_speed_corr * target_mr;

       
        // Make X the drive, all others hugelly resistive to change
        // TBD: Apply non regenerative option
        
        xdriveTL = wheelTL_joint.angularXDrive;
        xdriveTL.positionDamper = TL_torque;
        wheelTL_joint.angularXDrive = xdriveTL;

        yzdriveTL = wheelTL_joint.angularYZDrive;
        yzdriveTL.positionDamper = (ortho_locked_top) ? 0 : max_torque;
        yzdriveTL.positionSpring = (ortho_locked_top) ? 9000 : 0;
        wheelTL_joint.angularYZDrive = yzdriveTL;

        xdriveTR = wheelTR_joint.angularXDrive;
        xdriveTR.positionDamper = TR_torque;
        wheelTR_joint.angularXDrive = xdriveTR;

        yzdriveTR = wheelTR_joint.angularYZDrive;
        yzdriveTR.positionDamper = (ortho_locked_top) ? 0 : max_torque;
        yzdriveTR.positionSpring = (ortho_locked_top) ? 9000 : 0;
        wheelTR_joint.angularYZDrive = yzdriveTR;

        xdriveBL = wheelBL_joint.angularXDrive;
        xdriveBL.positionDamper = BL_torque;
        wheelBL_joint.angularXDrive = xdriveBL;

        yzdriveBL = wheelBL_joint.angularYZDrive;
        yzdriveBL.positionDamper = (ortho_locked_bot) ? 0 : max_torque;
        yzdriveBL.positionSpring = (ortho_locked_bot) ? 9000 : 0;
        wheelBL_joint.angularYZDrive = yzdriveBL;

        xdriveBR = wheelBR_joint.angularXDrive;
        xdriveBR.positionDamper = BR_torque;
        wheelBR_joint.angularXDrive = xdriveBR;

        yzdriveBR = wheelBR_joint.angularYZDrive;
        yzdriveBR.positionDamper = (ortho_locked_bot) ? 0 : max_torque;
        yzdriveBR.positionSpring = (ortho_locked_bot) ? 9000 : 0;
        wheelBR_joint.angularYZDrive = yzdriveBR;

        wheelTL_joint.targetAngularVelocity = targetv_tl;
        wheelTR_joint.targetAngularVelocity = targetv_tr;
        wheelBL_joint.targetAngularVelocity = targetv_bl;
        wheelBR_joint.targetAngularVelocity = targetv_br;

        if (DriveTrain == "6-Wheel Tank")
        {
            xdriveML = wheelML_joint.angularXDrive;
            xdriveML.positionDamper = ML_torque;
            wheelML_joint.angularXDrive = xdriveML;

            yzdriveML = wheelML_joint.angularYZDrive;
            yzdriveML.positionDamper = (ortho_locked_bot) ? 0 : max_torque;
            yzdriveML.positionSpring = (ortho_locked_bot) ? 9000 : 0;
            wheelML_joint.angularYZDrive = yzdriveML;

            xdriveMR = wheelMR_joint.angularXDrive;
            xdriveMR.positionDamper = MR_torque;
            wheelMR_joint.angularXDrive = xdriveMR;

            yzdriveMR = wheelMR_joint.angularYZDrive;
            yzdriveMR.positionDamper = (ortho_locked_bot) ? 0 : max_torque;
            yzdriveMR.positionSpring = (ortho_locked_bot) ? 9000 : 0;
            wheelMR_joint.angularYZDrive = yzdriveMR;

            wheelML_joint.targetAngularVelocity = targetv_ml;
            wheelMR_joint.targetAngularVelocity = targetv_mr;
        }

        // **********************************************************
        // For orthogonal direction, apply breaking force if not omni-wheel
        // Compare to actual velocities
        TL_curr = TransformWorldVelToWheelVel(wheelTL);
        TR_curr = TransformWorldVelToWheelVel(wheelTR);
        BL_curr = TransformWorldVelToWheelVel(wheelBL);
        BR_curr = TransformWorldVelToWheelVel(wheelBR);
        ML_curr = TransformWorldVelToWheelVel(wheelML);
        MR_curr = TransformWorldVelToWheelVel(wheelMR);

        // Get orthagonal movement to desired direction
        // Note that we only use x and z direction, no Y direction, thus for this calculation go to Vector2 where y=z
        Vector2 ortho2_dir_tl = Vector2.Perpendicular((new Vector2(wheel_dir_TL.x, wheel_dir_TL.z)));
        Vector2 ortho2_dir_tr = Vector2.Perpendicular((new Vector2(wheel_dir_TR.x, wheel_dir_TR.z)));
        Vector2 ortho2_dir_bl = Vector2.Perpendicular((new Vector2(wheel_dir_BL.x, wheel_dir_BL.z)));
        Vector2 ortho2_dir_br = Vector2.Perpendicular((new Vector2(wheel_dir_BR.x, wheel_dir_BR.z)));
        Vector2 ortho2_dir_ml = Vector2.Perpendicular((new Vector2(wheel_dir_ML.x, wheel_dir_ML.z)));
        Vector2 ortho2_dir_mr = Vector2.Perpendicular((new Vector2(wheel_dir_MR.x, wheel_dir_MR.z)));

        float ortho_vel_tl = Vector2.Dot(ortho2_dir_tl, new Vector2(TL_curr.x, TL_curr.z));
        float ortho_vel_tr = Vector2.Dot(ortho2_dir_tr, new Vector2(TR_curr.x, TR_curr.z));
        float ortho_vel_bl = Vector2.Dot(ortho2_dir_bl, new Vector2(BL_curr.x, BL_curr.z));
        float ortho_vel_br = Vector2.Dot(ortho2_dir_br, new Vector2(BR_curr.x, BR_curr.z));
        float ortho_vel_ml = Vector2.Dot(ortho2_dir_ml, new Vector2(ML_curr.x, ML_curr.z));
        float ortho_vel_mr = Vector2.Dot(ortho2_dir_mr, new Vector2(MR_curr.x, MR_curr.z));


         ortho3_vel_tl =  (new Vector3(ortho_vel_tl * ortho2_dir_tl.x, 0*TL_curr.y, ortho_vel_tl * ortho2_dir_tl.y));
         ortho3_vel_tr =  (new Vector3(ortho_vel_tr * ortho2_dir_tr.x, 0*TR_curr.y, ortho_vel_tr * ortho2_dir_tr.y));
         ortho3_vel_bl =  (new Vector3(ortho_vel_bl * ortho2_dir_bl.x, 0*BL_curr.y, ortho_vel_bl * ortho2_dir_bl.y));
         ortho3_vel_br =  (new Vector3(ortho_vel_br * ortho2_dir_br.x, 0*BR_curr.y, ortho_vel_br * ortho2_dir_br.y));
         ortho3_vel_ml =  (new Vector3(ortho_vel_ml * ortho2_dir_ml.x, 0*ML_curr.y, ortho_vel_ml * ortho2_dir_ml.y));
         ortho3_vel_mr =  (new Vector3(ortho_vel_mr * ortho2_dir_mr.x, 0*MR_curr.y, ortho_vel_mr * ortho2_dir_mr.y));


        // Apply the orthogonal torque forces 
        // However, will apply to some regenerative forces just to help
        
        if (!ortho_locked_top && wheel_dir_TL.magnitude > 0f) { wheelTL_body.AddTorque(rb_body.transform.TransformDirection(ortho3_vel_tl) * max_torque * -1f, ForceMode.Force); }
        if (!ortho_locked_top && wheel_dir_TR.magnitude > 0f) { wheelTR_body.AddTorque(rb_body.transform.TransformDirection(ortho3_vel_tr) * max_torque * -1f, ForceMode.Force); }
        if (!ortho_locked_bot && wheel_dir_BL.magnitude > 0f) { wheelBL_body.AddTorque(rb_body.transform.TransformDirection(ortho3_vel_bl) * max_torque * -1f, ForceMode.Force); }
        if (!ortho_locked_bot && wheel_dir_BR.magnitude > 0f) { wheelBR_body.AddTorque(rb_body.transform.TransformDirection(ortho3_vel_br) * max_torque * -1f, ForceMode.Force); }
        if (!ortho_locked_mid && wheel_dir_ML.magnitude > 0f) { wheelML_body.AddTorque(rb_body.transform.TransformDirection(ortho3_vel_ml) * max_torque * -1f, ForceMode.Force); }
        if (!ortho_locked_mid && wheel_dir_MR.magnitude > 0f) { wheelMR_body.AddTorque(rb_body.transform.TransformDirection(ortho3_vel_mr) * max_torque * -1f, ForceMode.Force); }
        



        // **********************************************************
        // Next need to apply friction forces:
        // TBD
        // TBD
        // TBD













        return;

        /*



        // Apply torques







        // Determine if we should be adding regenerative breaking 
        // ******** Measurement of robot behavior on field shows this breaking looks like a fixed force, thus it's less likelly caused
        // ******** by regeneration, but more by the powerloss in the gears??? Not sure at this point, but will follow lab measurement
        //
        // Some more explanation: The rev hub controls the motors as a half-bridge - what this means is that it can easily add power to the
        // wheel, but reducing power is poor UNLESS it switches into "reverse direction" mode. Thus if you just reduce forward power it will
        // coast into the new speed at a ~25% regenerative braking, but if you apply even a little bit of reverse direction power, it now switches
        // mode and will be using full motor power to reduce speed.
        //
        // This is too complicated given our wheels are modeled as a sphere rotating in 2 dimensions. Will simplify this to as follows:
        // 1) If magnitude is reduced, we will coast
        // 2) If the difference in Vectors is more than 10% higher than original (e.g. we reversed direction), then apply full motor power
        // Do keep in mind that the "goal" versus actual speed attained are different because of friction forces
        float TL_regen = 0;
        if (!activebreaking && (TL_goal.magnitude < 0.9f * TL_curr.magnitude) && ((TL_goal - TL_curr).magnitude < 1.1f * TL_curr.magnitude))  // If decelerating, rely on friction forces only (no motor spring)
        {
            TL_regen = default_regen_breaking;
            TL_torque = 0f;
        }

        float TR_regen = 0;
        if (!activebreaking && (TR_goal.magnitude < 0.9f * TR_curr.magnitude) && ((TR_goal - TR_curr).magnitude < 1.1f * TR_curr.magnitude))
        {
            TR_regen = default_regen_breaking;
            TR_torque = 0f;
        }

        float BL_regen = 0;
        if (!activebreaking && (BL_goal.magnitude < 0.9f * BL_curr.magnitude) && ((BL_goal - BL_curr).magnitude < 1.1f * BL_curr.magnitude))
        {
            BL_regen = default_regen_breaking;
            BL_torque = 0;
        }

        float BR_regen = 0;
        if (!activebreaking && (BR_goal.magnitude < 0.9f * BR_curr.magnitude) && ((BR_goal - BR_curr).magnitude < 1.1f * BR_curr.magnitude))
        {
            BR_regen = default_regen_breaking;
            BR_torque = 0;
        }


        if (DriveTrain == "Tank")
        {
            // Make sure final goals are off
            TL_goal.x = 0; TL_goal.y = 0; TL_goal.z = 0;
            TR_goal.x = 0; TR_goal.y = 0; TR_goal.z = 0;

            TL_torque = 0f;
            TR_torque = 0f;
            TL_regen = 0f;
            TR_regen = 0f;

            // Double the BL/BR torques since all 4 motors are on those wheels
            BL_torque *= 2f;
            BR_torque *= 2f;
        }

        Vector3 ML_goal = new Vector3();
        Vector3 MR_goal = new Vector3();
        float ML_regen = 0;
        float MR_regen = 0;
        ML_torque_multiplier = 1f;
        MR_torque_multiplier = 1f;


        if (DriveTrain == "6-Wheel Tank")
        {
            TL_goal.y = 0; TL_goal.z = 0;
            TR_goal.y = 0; TR_goal.z = 0;
            BL_goal.y = 0; BL_goal.z = 0;
            BR_goal.y = 0; BR_goal.z = 0;

            TL_goal.x = BL_goal.x;
            TR_goal.x = BR_goal.x;


            // Assign ML goals to be average top top/bottom
            ML_goal = (TL_goal + BL_goal) / 2f;
            MR_goal = (TR_goal + BR_goal) / 2f;

            // Use the average for T/B torques for middle
            TL_torque *= 1f; // Used to be 0.666f for all)
            BL_torque *= 1f;
            TR_torque *= 1f;
            BR_torque *= 1f;

            ML_torque = (TL_torque + BL_torque) / 2f;
            MR_torque = (TR_torque + BR_torque) / 2f;

            // Apply breaking if necessary
            if (!activebreaking && (ML_goal.magnitude < 0.9f * ML_curr.magnitude) && ((ML_goal - ML_curr).magnitude < 1.1f * ML_curr.magnitude))
            {
                ML_regen = default_regen_breaking;
                ML_torque = 0;
            }

            if (!activebreaking && (MR_goal.magnitude < 0.9f * MR_curr.magnitude) && ((MR_goal - MR_curr).magnitude < 1.1f * MR_curr.magnitude))
            {
                MR_regen = default_regen_breaking;
                MR_torque = 0;
            }



            // Finally if the speed of the wheel is small and we are trying to make it go, increase the torques
            if ((TL_goal.magnitude > 0.2f * max_speed_corr) && (TL_curr.magnitude < 0.05f * max_speed_corr))
            {
                TL_torque *= 1f + (1 - TL_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((TR_goal.magnitude > 0.2f * max_speed_corr) && (TR_curr.magnitude < 0.05f * max_speed_corr))
            {
                TR_torque *= 1f + (1 - TR_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((ML_goal.magnitude > 0.2f * max_speed_corr) && (ML_curr.magnitude < 0.05f * max_speed_corr))
            {
                ML_torque *= 1f + (1 - ML_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((MR_goal.magnitude > 0.2f * max_speed_corr) && (MR_curr.magnitude < 0.05f * max_speed_corr))
            {
                MR_torque *= 1f + (1 - MR_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((BL_goal.magnitude > 0.2f * max_speed_corr) && (BL_curr.magnitude < 0.05f * max_speed_corr))
            {
                BL_torque *= 1f + (1 - BL_curr.magnitude / (0.05f * max_speed_corr));
            }

            if ((BR_goal.magnitude > 0.2f * max_speed_corr) && (BR_curr.magnitude < 0.05f * max_speed_corr))
            {
                BR_torque *= 1f + (1 - BR_curr.magnitude / (0.05f * max_speed_corr));
            }
        }

        if (DriveTrain == "Mecanum")
        {
            // With mecanum wheels we need to reduce the speed applied at 45 degrees
            // May want to increase the torque (since mecanums have higher torque at 45 degrees)
            // Vector3.Angle is a positiove only value between 0 and 180 returning the smallest of the number;
            // Since we want the function to repeat every 90 degrees, we need to multiply angle by 4 when taking the Sin of it
            float TL_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(TL_goal, diagonal_rotation) * 3.1415926f / 180f));
            float TR_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(TR_goal, diagonal_rotation) * 3.1415926f / 180f));
            float BL_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(BL_goal, diagonal_rotation) * 3.1415926f / 180f));
            float BR_diag_scalar = (float)Math.Abs(Math.Cos(2f * Vector3.Angle(BR_goal, diagonal_rotation) * 3.1415926f / 180f));

            // reduce top speeds by how close we are to the diagonal
            TL_goal *= 1f / (1f + 0.7f * TL_diag_scalar);
            TR_goal *= 1f / (1f + 0.7f * TR_diag_scalar);
            BL_goal *= 1f / (1f + 0.7f * BL_diag_scalar);
            BR_goal *= 1f / (1f + 0.7f * BR_diag_scalar);

            // Next increase torque based on how much we reduce the speed by
            // Since our target velocity was reduced, increasing the torque only offsets the acc/decc to be back to normal,
            // but our applied torque at the lower speed needs to be higher than the previous full speed. Thus we need to square the torque increase.
            TL_torque *= (float)Math.Pow(1f + 0.7f * TL_diag_scalar, 2f);
            TR_torque *= (float)Math.Pow(1f + 0.7f * TR_diag_scalar, 2f);
            BL_torque *= (float)Math.Pow(1f + 0.7f * BL_diag_scalar, 2f);
            BR_torque *= (float)Math.Pow(1f + 0.7f * BR_diag_scalar, 2f);
        }

        // NOTE: the positionDamper is a force that is proportional to speed, which is exactly what motors do,
        // Thus we are using the positionDamper as the motor driver.
        // From unity manual: force = PositionSpring * (target position - position) + PositionDamper * (targetVelocity - velocity)
        // So we want PositionDamper to be the torque
        xdriveTL = wheelTL_joint.angularXDrive;
        xdriveTL.positionDamper = TL_torque;
        wheelTL_joint.angularXDrive = xdriveTL;

        yzdriveTL = wheelTL_joint.angularYZDrive;
        yzdriveTL.positionDamper = TL_torque;
        wheelTL_joint.angularYZDrive = yzdriveTL;

        xdriveTR = wheelTR_joint.angularXDrive;
        xdriveTR.positionDamper = TR_torque;
        wheelTR_joint.angularXDrive = xdriveTR;

        yzdriveTR = wheelTR_joint.angularYZDrive;
        yzdriveTR.positionDamper = TR_torque;
        wheelTR_joint.angularYZDrive = yzdriveTR;

        xdriveBL = wheelBL_joint.angularXDrive;
        xdriveBL.positionDamper = BL_torque;
        wheelBL_joint.angularXDrive = xdriveBL;

        yzdriveBL = wheelBL_joint.angularYZDrive;
        yzdriveBL.positionDamper = BL_torque;
        wheelBL_joint.angularYZDrive = yzdriveBL;

        xdriveBR = wheelBR_joint.angularXDrive;
        xdriveBR.positionDamper = BR_torque;
        wheelBR_joint.angularXDrive = xdriveBR;

        yzdriveBR = wheelBR_joint.angularYZDrive;
        yzdriveBR.positionDamper = BR_torque;
        wheelBR_joint.angularYZDrive = yzdriveBR;

        wheelTL_joint.targetAngularVelocity = TL_goal;
        wheelTR_joint.targetAngularVelocity = TR_goal;
        wheelBL_joint.targetAngularVelocity = BL_goal;
        wheelBR_joint.targetAngularVelocity = BR_goal;

        if (DriveTrain == "6-Wheel Tank")
        {
            xdriveML = wheelML_joint.angularXDrive;
            xdriveML.positionDamper = ML_torque;
            wheelML_joint.angularXDrive = xdriveML;

            yzdriveML = wheelML_joint.angularYZDrive;
            yzdriveML.positionDamper = ML_torque;
            wheelML_joint.angularYZDrive = yzdriveML;

            xdriveMR = wheelMR_joint.angularXDrive;
            xdriveMR.positionDamper = MR_torque;
            wheelMR_joint.angularXDrive = xdriveMR;

            yzdriveMR = wheelMR_joint.angularYZDrive;
            yzdriveMR.positionDamper = MR_torque;
            wheelMR_joint.angularYZDrive = yzdriveMR;

            wheelML_joint.targetAngularVelocity = ML_goal;
            wheelMR_joint.targetAngularVelocity = MR_goal;
        }



        //** SOUND CALCULATIONS *
        averageMovementForceMagnitude = (TL_goal.magnitude + TR_goal.magnitude +
                BL_goal.magnitude + BR_goal.magnitude) / 4f / max_speed_corr;
        averageVelocityMagnitude = (TL_curr.magnitude + TR_curr.magnitude +
                BL_curr.magnitude + BR_curr.magnitude) / 4f / max_speed_corr;
        // **********************

        // Now apply friction forces
        // friction_torque tries to apply a force that reduces speed of the robot by a fixed ft/s^2 regardless of weight
        // In general, straffing exhibits a much higher friction. 
        // First just add general friction against all dimensions
        wheelTL_body.AddTorque(-wheelTL_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + TL_regen * max_acceleration));
        wheelBL_body.AddTorque(-wheelBL_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + BL_regen * max_acceleration));
        wheelTR_body.AddTorque(-wheelTR_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + TR_regen * max_acceleration));
        wheelBR_body.AddTorque(-wheelBR_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + BR_regen * max_acceleration));

        if (wheelML_body && wheelMR_body && DriveTrain == "6-Wheel Tank")
        {
            wheelML_body.AddTorque(-wheelML_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + ML_regen * max_acceleration));
            wheelMR_body.AddTorque(-wheelMR_body.angularVelocity.normalized * friction_torque_scaler * friction_extrax * (GLOBALS.friction + MR_regen * max_acceleration));
        }

        // Next add friction against straffing 
        // z-axis is going forward, thus rotation around x-axis is the direction of rotation
        // Also need to consider rb_body may be rotated..
        if (DriveTrain == "Mecanum")
        {
            Vector3 friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelTL_body.angularVelocity).x, 0, 0);
            wheelTL_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);

            friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelTR_body.angularVelocity).x, 0, 0);
            wheelTR_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);

            friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelBR_body.angularVelocity).x, 0, 0);
            wheelBR_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);

            friction_strafe = new Vector3(-1f * rb_body.transform.InverseTransformDirection(wheelBL_body.angularVelocity).x, 0, 0);
            wheelBL_body.AddTorque(rb_body.transform.TransformDirection(friction_strafe.normalized) * straffing_friction_scaler * GLOBALS.friction * friction_torque_scaler * friction_extrax);
        }
        return;

        */
    }














    private float averageMovementForceMagnitude = 0;
    private float averageVelocityMagnitude = 0;

    //used for sound purposes
    public float getAverageMovementForceMagnitude() {
        return averageMovementForceMagnitude;
    }

    public float getAveragVelocityMagnitude()
    {
        return averageVelocityMagnitude;
    }


    // Tweak Performance
    // You can specify a speed or torque multiplier
    // It can make people faster, slower, or even invert value
    // Enter value of 0 to restore back to normal

    private float max_torque_saved = 0f;
    private float max_speed_saved = 0f;
    public void TweakPerformance(float speed_multiplier_new, float torque_multiplier_new)
    {
        float speed_multiplier = speed_multiplier_new;
        float torque_multiplier = torque_multiplier_new;

        // An increase in Speed also increases torque, thus we need to inverse affect torque to compensate
        if (speed_multiplier_new != 0)
        {
            torque_multiplier /= speed_multiplier_new;
        }

        // On the other hand, torque itself doesn't affect speed directly, but since friction forces reduce max-speed, we do want to nudge max speed down
        // to keep top speed approximately the same
        if (torque_multiplier_new != 0)
        {
            speed_multiplier /= (torque_multiplier - 1f) * 0.05f + 1f;
        }

        // If no torque is specified, return back to normal (if there was a saved value)
        if (torque_multiplier == 0f)
        {
            if (max_torque_saved != 0f)
            {
                max_torque = max_torque_saved;
                max_torque_saved = 0f;
            }
        }

        // If maxtorque was specified, then increment it appropriately
        else
        {
            if (max_torque_saved == 0f)
            {
                max_torque_saved = max_torque;
            }

            max_torque *= torque_multiplier;
        }

        // If no speed is specified, return back to normal (if there was a saved value)
        if (speed_multiplier == 0f)
        {
            if (max_speed_saved != 0f)
            {
                max_speed_corr = max_speed_saved;
                max_speed_saved = 0f;
            }

        }

        // If maxspeed was specified, then increment it appropriately
        else
        {
            if (max_speed_saved == 0f)
            {
                max_speed_saved = max_speed_corr;
            }

            max_speed_corr *= speed_multiplier;
        }

    }

    public float stick_multiplier = 1f;
    public void ScaleStickControls(float scaler)
    {
        stick_multiplier = scaler;
    }

    // Turns off renderers OR makes them transluscent
    private List<Renderer> saved_renderers = new List<Renderer>();
    private List<Material> saved_materials = new List<Material>();
    private Material material_translucent;
    private bool invisible = false;

    // Turns off all renderers to make it invisible (or translucent)
    public void TurnOffRenderers(bool translucent)
    {
        // If we haven't identified all renderers, then find them
        if (saved_renderers.Count <= 0)
        {
            Renderer[] all_renderers = transform.GetComponentsInChildren<Renderer>();
            foreach (Renderer currRenderer in all_renderers)
            {
                if (currRenderer.enabled)
                {
                    saved_renderers.Add(currRenderer);
                    saved_materials.Add(currRenderer.material);
                }
            }

            material_translucent = (Material)Resources.Load("Gray_Translucent", typeof(Material));

        }

        // Apply the change in visibility
        foreach (Renderer currRenderer in saved_renderers)
        {
            if (!currRenderer) { continue; }
            if(currRenderer.material == material_translucent) { continue; }
            if (!translucent) { currRenderer.enabled = false; }
            else
            {
                currRenderer.enabled = true;

                // Don't change material for "Protractor" or "Indicator"
                if (currRenderer.name == "Protractor" || currRenderer.name == "Indicator") { continue; }
                
                // Destroy(currRenderer.material);
                currRenderer.material = material_translucent;
            }
        }

        invisible = true;
    }

    // Turns the renderers back on
    public void TurnOnRenderers()
    {
        // If we haven't identified all renderers, then find them
        if (saved_renderers.Count <= 0)
        {
            return;
        }

        // Apply the change in visibility
        for (int i = 0; i < saved_renderers.Count; i++)
        {
            if (!saved_renderers[i]) { continue; }
            saved_renderers[i].enabled = true;

            if(saved_renderers[i].material == saved_materials[i]) { continue; }

            // Don't want to destroy these renderers??
            Destroy(saved_renderers[i].material); 
            saved_renderers[i].material = saved_materials[i];
        }

        invisible = false;
    }


    // UPDATE MOVEMENT
    // Calculates and applies the movement rotational speeds
    // gamepad1_left_stick_y == forward/backwards (-1 = forwards)
    // gamepad1_left_stick_x == sideways strafe (-1 = left)
    // gamepad1_right_stick_x == Rotation
    //
    // If tank control is enabled, then
    // gamepad1_left_stick_y = left wheel
    // gamepad1_right_stick_y = right wheel


    // Update movement determines desired direction of movement relative to body position wanted by used, corrected for any system settings
    // It then passes it to the ApplySpringForces function that applies the correct forces given the drive system used
    virtual public void UpdateMovement_new()
    {
        // Do not update movement if we dont have a body
        if (!rb_body) { return; }
        if (!rb_body.gameObject.activeSelf) { return; }
        if (!wheelTL || !wheelTR || !wheelBL || !wheelBR) { return; }


        // *************************************************
        // Get the vector for movement + strafing
        // Will define the following: angle = 0 is moving forward.
        // angle = 90 is moving sideways using mechanum drive (to the left of object facing forwards)

        // Spectator movement is not handled here
        if (isSpectator) { return; }

        // If we are kinematic, skip
        if (isKinematic) { return; }

        // Scale movement if applicable
        // Scaled mainly for power-ups (e.g. inverting control), this should rarely be used and is not user settable, etc...
        float igamepad1_left_stick_y = stick_multiplier * gamepad1_left_stick_y;  // Forward/Backwards or Left wheel for tank control
        float igamepad1_left_stick_x = stick_multiplier * gamepad1_left_stick_x;  // Strafe Left/Right
        float igamepad1_right_stick_y = stick_multiplier * gamepad1_right_stick_y; // Right wheel for tank control
        float igamepad1_right_stick_x = stick_multiplier * gamepad1_right_stick_x; // Turn left/right


        // ***************************************
        // using Vector3 to specify all movement
        // (x,y) = x=strafe, y=forward movement
        // z = rotation (clockwise I believe)
        // Actualy joystick has left be -1f, right be 1f, up is -1f, down is 1f. These two are in opposite direction
        // of how we are representing it, thus multiply these be -1f;
        Vector3 target_movement = new Vector3(0, 0, 0);

        target_movement.x = -1f*igamepad1_left_stick_x;
        if (tankcontrol)
        {
            target_movement.y = -1f*(igamepad1_left_stick_y + igamepad1_right_stick_y) / 2f;
            target_movement.z = (igamepad1_right_stick_y - igamepad1_left_stick_y) / 2f;
        }
        else
        {
            target_movement.y = -1f*igamepad1_left_stick_y;
            target_movement.z = igamepad1_right_stick_x;
        }

        // ********************************************
        // Field Centric Calculations
        // ********************************************
        //
        // Rotation: this is clockwise or counterclockwise and has nothing to do with field centric, thus this doesn't get affect
        // Forward/strafe: this is affected by field centric, and thus has to be corrected.
        // Tank drive is a bit nonsensical to use field centric: when do you turn versus just go backwards? Thus the reuslts may be unpredictable
     
        // If we have field centric control enabled and this is mecanum drive, then make it so
        if (fieldcentric)
        {
            // Get the difference between body and fieldcentric rotation
            Quaternion delta = rb_body.transform.rotation * Quaternion.Inverse(fieldcentric_rotation);

            Vector2 movement_2D;
            movement_2D.x = target_movement.x;
            movement_2D.y = target_movement.y;

            // Correct the angle of movement based on body's  delta movement on field
            double movement_speed = movement_2D.magnitude;
            double movement_angle = Math.Atan2(movement_2D.x, movement_2D.y);
            movement_angle += delta.eulerAngles.y * Math.PI / 180f - Math.PI / 2f;

            // Put back into target_movement
            target_movement.x = (float) (movement_speed * Math.Cos(movement_angle));
            target_movement.y = (float) (movement_speed * Math.Sin(movement_angle));
        }

        // ********************************************
        // Clean up target directions for limitations/ overrides, etc...
        // ********************************************
        //

        // Strip straffing information if we have tank drivetrain
        if (DriveTrain == "Tank" || DriveTrain == "6-Wheel Tank")
        {
            target_movement.x = 0f;
        }

        // Add turning override function
        if (Math.Abs(turning_overide) > 0.01f)
        {
            target_movement.z = turning_overide;
        }

        // Apply turn-scale for turning function
        target_movement.z *= turn_scale;

        // Get the actual wheel forces
        ApplyWheelSpringForces_new(target_movement);

        return;

    }



    virtual public void UpdateMovement()
    {
        // If we want to use new algorithm, use it
        if( use_new_algorithm)
        {
            UpdateMovement_new();
            return;
        }

        // Do not update movement if we dont have a body
        if (!rb_body) { return; }
        if (!rb_body.gameObject.activeSelf) { return; }
        if (!wheelTL || !wheelTR || !wheelBL || !wheelBR) { return; }


        // *************************************************
        // Get the vector for movement + strafing
        // Will define the following: angle = 0 is moving forward.
        // angle = 90 is moving sideways using mechanum drive (to the left of object facing forwards)

        // First, if both x and y are 0, make the angle 0
        double movement_angle = 0;
        double movement_speed = 0;

        // ***********************
        // First clean up and correct joystick readings
        // ***********************
        // CHANGED: Joystick control now handles dead-zone, therefore this is not really required except to make sure numerical noise can
        // be zeroed out if a user doesn't set a dead zone
        if (Math.Abs(gamepad1_left_stick_y) < 0.001f)
        {
            gamepad1_left_stick_y = 0;
        }
        if (Math.Abs(gamepad1_left_stick_x) < 0.001f)
        {
            gamepad1_left_stick_x = 0;
        }
        if (Math.Abs(gamepad1_right_stick_y) < 0.001f)
        {
            gamepad1_right_stick_y = 0;
        }
        if (Math.Abs(gamepad1_right_stick_x) < 0.001f)
        {
            gamepad1_right_stick_x = 0;
        }

        // Spectator movement is not handled here
        if (isSpectator) { return; }

        // If we are kinematic, skip
        if (isKinematic) { return; }

        // Scale movement if applicable
        // Scaled mainly for power-ups (e.g. inverting control), this should rarely be used and is not user settable, etc...
        float igamepad1_left_stick_y = stick_multiplier * gamepad1_left_stick_y;  // Forward/Backwards or Left wheel for tank control
        float igamepad1_left_stick_x = stick_multiplier * gamepad1_left_stick_x;  // Strafe Left/Right
        float igamepad1_right_stick_y = stick_multiplier * gamepad1_right_stick_y; // Right wheel for tank control
        float igamepad1_right_stick_x = stick_multiplier * gamepad1_right_stick_x; // Turn left/right

        // ***************************************
        // Trying new algorithm: using Vector3 to specify all movement
        // (x,y) = x=turning, y=forward movement
        // z = straffing
        Vector3 target_movement;
        target_movement.z = igamepad1_left_stick_x;
        if (tankcontrol)
        {
            target_movement.y = (igamepad1_left_stick_y + igamepad1_right_stick_y) / 2f;
            target_movement.x = (igamepad1_right_stick_y - igamepad1_left_stick_y) / 2f;
        }
        else
        {
            target_movement.y = igamepad1_left_stick_y;
            target_movement.x = igamepad1_right_stick_x;
        }


        // ****************************************
        // Calculate desired rotation (clockwise, etc.. relative to robot body)
        // ****************************************
        // Normally turning is only right_stick_x. 1=counterclockwise, -1=clockwise
        float rotation_target = igamepad1_right_stick_x;

        // Tank control means 1 joystick is left side, the other is right side of the robot. This is different from Tank drive which restricts straffing.
        if (tankcontrol)
        {
            // For tank movement, it's just the difference between wheel forces
            rotation_target = (igamepad1_right_stick_y - igamepad1_left_stick_y) / 2f;
        }

        // ********************************************
        // Calculate desired robot movement (forward/backwards,strafe - relative to robot body)
        // *********************************************
        // Calculate movement speed + angle

        if (!tankcontrol)
        {
            // Regular control
            // Applies to both Mecanum and Tank drive
            // For mecanum, to extract magnitude + angle we use atan2.
            // Since both x and y movement can be 1, the magnitude of the vector can be as high as 1.414 (will need to clamp speed to 1f later on)
            // Notice that tank drives should have no "x" component. For simplicity, we will calculate full mecanum here and remove the "x" component later
            if ((igamepad1_left_stick_x != 0f) || (igamepad1_left_stick_y != 0f))
            {
                movement_angle = Math.Atan2(-1f * igamepad1_left_stick_x, -1f * igamepad1_left_stick_y);
            }
            movement_speed = Math.Sqrt(Math.Pow(igamepad1_left_stick_y, 2f) + Math.Pow(igamepad1_left_stick_x, 2f));
        }
        else
        {
            // Tank control 
            // In this case the movement is either forwards or backwards
            // Movement speed is just the average of the left + right wheels
            movement_speed = (igamepad1_left_stick_y + igamepad1_right_stick_y) / 2f;

            // The angle is also simply the forward/backwards direction
            if (movement_speed != 0f)
            {
                movement_angle = Math.Atan2(0, -1f * movement_speed); // Using Atan2 to keep units and offset consistent
            }
            movement_speed = Math.Abs(movement_speed); // Take out negative movement speed since angle takes care of direction
        }

        // ********************************************
        // Field Centric Calculations
        // ********************************************
        // Translating body-centric to field centric controls:
        //  For Mecanum it's simply modifying the angle of movement to be field centric rather than body centric
        // Tank drive would require rotating the robot towards the target angle, and it then would prevent reversing so this doesn't make sense.
        //  Will thus ignore fieldcentric drive for non-mecanum drive

        // If we have field centric control enabled and this is mecanum drive, then make it so
        if (fieldcentric && DriveTrain != "Tank" && DriveTrain != "6-Wheel Tank")
        {
            // Get the difference between body and fieldcentric rotation
            Quaternion delta = rb_body.transform.rotation * Quaternion.Inverse(fieldcentric_rotation);

            // Add it to the current movement angle.. However camera forward direction is different from robot forward, correct it here with the 90Deg shift
            movement_angle += delta.eulerAngles.y * Math.PI / 180f + Math.PI / 2f;
        }


        // If the movement is restricted to tank drive, than remove sideways (y) component
        if (DriveTrain == "Tank" || DriveTrain == "6-Wheel Tank")
        {
            movement_speed = Math.Abs(Math.Cos(movement_angle) * movement_speed);

            if (movement_angle > Deg2Rad * 90f ||
                movement_angle < -1f * Deg2Rad * 90f)
            {
                movement_angle = Math.PI;
            }
            else
            {
                movement_angle = 0;
            }
        }

        // Make sure movement speed is clamped to 100%
        if (movement_speed > 1f)
        { movement_speed = 1f; }



        // Now change the angle/magnitude into Z and X force such that:
        // Z-Force = Sideways movemenet
        // X force = Forward movement
        Vector3 forceWorld = new Vector3((float)(movement_speed * Math.Cos(movement_angle)), 0, (float)(movement_speed * Math.Sin(movement_angle)));

        // *************************************************
        // Get the vector for turning

        // Turning speed scaling. 1 = same max speed/force as movement but 1/root(2) in x and z (diagonal)

        float rotational_speed = (float)1f;

        if (Math.Abs(turning_overide) > 0.01f)
        {
            rotation_target = turning_overide;
        }

        if (Math.Abs(rotation_target) < 0.01)
        {
            rotational_speed = 0;
        }

        Vector3 rotationWorld_TL = new Vector3(0, 0, 0);
        Vector3 rotationWorld_TR = new Vector3(0, 0, 0);
        Vector3 rotationWorld_BR = new Vector3(0, 0, 0);
        Vector3 rotationWorld_BL = new Vector3(0, 0, 0);

        // Add the rotational force to the vecotrs
        // So how should the wheels turn? Ideally each wheel goes 45 degrees: 1/root(2) in x, 1/root(2) in z
        // Unless it's tank drive, then it just goes forward/backwards
        if (DriveTrain == "Tank" || DriveTrain == "6-Wheel Tank")
        {
            rotationWorld_BR = new Vector3(-1f * rotation_target * turn_scale, 0, 0);
            rotationWorld_BL = new Vector3(rotation_target * turn_scale, 0, 0);
        }
        else
        {
            // In mecanum drive, going diagonally is 2x slower and less total power but torque is higher
            // used to scale down the speed here. No longer, will now process this at the application of spring force level
            rotational_speed *= 1f; // No longer reducing rotational speed here
            rotationWorld_TL = new Vector3(rotational_speed * rotation_target * turn_scale, 0, -1f * rotational_speed * rotation_target * turn_scale);
            rotationWorld_TR = new Vector3(-1f * rotational_speed * rotation_target * turn_scale, 0, -1f * rotational_speed * rotation_target * turn_scale);
            rotationWorld_BR = new Vector3(-1f * rotational_speed * rotation_target * turn_scale, 0, rotational_speed * rotation_target * turn_scale);
            rotationWorld_BL = new Vector3(rotational_speed * rotation_target * turn_scale, 0, rotational_speed * rotation_target * turn_scale);
        }


        // Scale down if it will exceed 1
        float correct_for_excessive_speed = 1.0f / Math.Max(Math.Max(Math.Max(Math.Max(
            (forceWorld + rotationWorld_TL).magnitude,
            (forceWorld + rotationWorld_TR).magnitude),
            (forceWorld + rotationWorld_BR).magnitude),
            (forceWorld + rotationWorld_BL).magnitude),
            1f
            );

        // Combine movement and turning vectors
        Vector3 forceTLWorld = correct_for_excessive_speed * (forceWorld + rotationWorld_TL);
        Vector3 forceBLWorld = correct_for_excessive_speed * (forceWorld + rotationWorld_BL);
        Vector3 forceTRWorld = correct_for_excessive_speed * (forceWorld + rotationWorld_TR);
        Vector3 forceBRWorld = correct_for_excessive_speed * (forceWorld + rotationWorld_BR);

        // Convert vectors from object centered to wheel space (whose axis are rotating/changing)
        // Calculate the rotational force/velocity vector for each wheel


        // Only need to do this if I'm applying my own forces. The spring will do that automatically if using their velocities
        forceTL = forceTLWorld;
        forceBL = forceBLWorld;
        forceTR = forceTRWorld;
        forceBR = forceBRWorld;

        // *************************************************
        // Calculate Actual Force and Apply them
        //
        // Since we are now using the spring joints to do this, need only to calculate final speed goal.

        // Define min speed to make sure object doesn't go to sleep
        Vector3 minspeed = new Vector3(1.1E-20f, 1.2E-20f, 1.3E-20f);

        // Target speed
        // Field size real-world = 12ft by 12ft
        // Field size unity = 7.09 x 7.09
        // Scaler = 1.69ft/unit = 22inches/unit
        // Wheel diameter = 0.1693 = 3.725"
        // Max speed = ft/s = x12 inch/s = 1.025 rotations/s = 6.44 rads/s
        Vector3 TL_goal = forceTL * max_speed_corr + minspeed;
        Vector3 TR_goal = forceTR * max_speed_corr + minspeed;
        Vector3 BL_goal = forceBL * max_speed_corr + minspeed;
        Vector3 BR_goal = forceBR * max_speed_corr + minspeed;


        ApplyWheelSpringForces(TL_goal, TR_goal, BL_goal, BR_goal);
        return;
    }

    Vector3 point1;
    public float time1 = 0;
    Vector3 point2;
    float time2 = 0.05f;
    Vector3 point3;
    float time3 = 0.1f;

    public float[] lastacc = { 0f, 0f, 0f, 0f, 0f };
    int countdown = 0;
    public float velocity1 = 0;
    public float velocity2 = 0;
    public float accel = 0;

    public float maxvel = 0;
    public float timemaxvel = 0;
    public float maxacc = 0;
    public float timemaxacc = 0;
    bool accreset = true;

    protected virtual void FixedUpdate()
    {
        // Movement wants to be updated at the highest frequency we can to keep it stable
        MoveAllHinges();
        MoveAllSlides();
        UpdateMovement();
        RobotFixedUpdate();
    }

    public virtual void RobotFixedUpdate()
    {

    }

    public float old_rot_intertia_scaler;
    bool logging = false;
    float logtime = 0f;
    private void Update()
    {
        if (doinit)
        {
            myRobotID = gameObject.GetComponent<RobotID>();
            Init_Robot();
            doinit = false;
        }

        // Clear hinges
        ClearHingeList();
        ClearSlideList();

        // Mark what control bits changed (for easy access in Update_Robot()
        InputsChanges();

        // Update our robot
        Update_Robot();

        if( DEBUG_COG)
        { 
            // Draw center of gravity
            // Get total weight
            Rigidbody[] allbodies = transform.GetComponentsInChildren<Rigidbody>();

            float total_weight = 0f;
            // Get total weight
            foreach (Rigidbody currbody in allbodies)
            {
                total_weight += currbody.mass;
            }

            // Now get the CoG
            Vector3 CoG = new Vector3(0, 0, 0);
            foreach (Rigidbody currbody in allbodies)
            {
                CoG += (currbody.centerOfMass + currbody.transform.position) * currbody.mass / total_weight;
            }


            Debug.DrawLine(CoG, CoG + new Vector3(0.1f, 0.1f, 0.1f));
            Debug.DrawLine(CoG, CoG - new Vector3(0.1f, 0.1f, 0.1f));

            Debug.DrawLine(CoG, CoG + new Vector3(0.1f, -0.1f, 0.1f));
            Debug.DrawLine(CoG, CoG - new Vector3(0.1f, -0.1f, 0.1f));
            Debug.DrawLine(CoG, CoG + new Vector3(0.1f, 0.1f, -0.1f));
            Debug.DrawLine(CoG, CoG - new Vector3(0.1f, 0.1f, -0.1f));


        }
        if (DEBUG)
        {

  

   

            // Update variables for tuning 
            if (old_rot_intertia_scaler != rot_inertia_scaler)
            {
                old_rot_intertia_scaler = rot_inertia_scaler;

                SetTotalWeight(total_weight, rot_inertia_scaler);

            }

            if (Input.GetKey(KeyCode.LeftBracket)) { logging = true; }
            if (Input.GetKey(KeyCode.RightBracket)) { logging = false; }

            if (logging && (Time.time - logtime > 0.05f))
            {
                logtime = Time.time;

                point1 = point2;
                point2 = point3;
                point3 = wheelTL_body.transform.position;

                time1 = time2;
                time2 = time3;
                time3 = Time.fixedTime;

                if (time3 - timemaxvel > 5f) { maxvel = 0; }

                if (time3 - timemaxacc > 5f)
                {
                    maxacc = 0;
                    accreset = true;
                }

                velocity1 = velocity2;
                velocity2 = (point3 - point2).magnitude / (time3 - time2);
                accel = (velocity2 - velocity1) / (time3 - time2);

                if (maxvel < velocity2)
                {
                    maxvel = velocity2;
                    timemaxvel = time3;
                }

                if (countdown > 0)
                {
                    lastacc[countdown - 1] = accel;
                    countdown--;
                }

                if (maxacc < accel)
                {
                    if (!accreset)
                    {
                        maxacc = accel;
                        lastacc[4] = accel;
                        countdown = 4;
                    }
                    timemaxacc = time3;
                    accreset = false;

                }

                WriteLog("\n" + time3 + "," + point3.x + "," + point3.z + "," + rb_body.transform.rotation.eulerAngles.y);

            }
        }

    }

    public virtual void Init_Robot()
    {
        // Default robot has nothing to be initialized
        // You need to overide this function for your robot specific initialization
    }

    public virtual void Update_Robot()
    {
        // Default robot has nothing to be done
        // You need to overide this function for your robot specific stuff
    }

    public virtual void SetName(string name)
    {
        // Set the name-tag
        Transform mynametag = transform.Find("Body/Nametag");
        if (mynametag)
        {
            if (mynametag.GetComponent<TMPro.TextMeshPro>())
            {
                mynametag.GetComponent<TextMesh>().text = name;
            }
        }

        // Set body name tags, if some are found, then disable to top one
        // Set our own items
        bool body_tag_found = false;
        List<Transform> nametags = new List<Transform>();

        foreach( Transform currchild in transform)
        {
            List<Transform> more_tags = FindAll(currchild, "NametagB");

            if(more_tags == null) { continue; }

            foreach ( Transform currnametag in more_tags)
            {
                nametags.Add(currnametag);
            }            
        }

        // List<Transform> nametags = FindAll(transform.Find("Body"), "NametagB");
        
        if (nametags != null)
        {
            foreach (Transform currtag in nametags)
            {
                if (currtag.gameObject.activeSelf && currtag.GetComponent<TMPro.TextMeshPro>() != null)
                {
                    currtag.GetComponent<TMPro.TextMeshPro>().text = name;
                    body_tag_found = true;
                }
            }
        }

        mynametag = transform.Find("Body/Nametag");
        if (!body_tag_found)
        {
            // Turn off main name-tag
            if (mynametag.GetComponent<TMPro.TextMeshPro>())
            {
                mynametag.GetComponent<TextMesh>().text = name;
                mynametag.gameObject.SetActive(true);
            }
        }
        else
        {
            mynametag.gameObject.SetActive(false);
        }
    }

    // Returns children in the parent transform whos name starts with startinName
    private List<Transform> FindAll(Transform parent, string startingName)
    {
        if (parent == null) { return null; }

        List<Transform> found_items = new List<Transform>();

        for (int i = 0; i < parent.childCount; i++)
        {
            if (parent.GetChild(i).name.StartsWith(startingName))
            {
                found_items.Add(parent.GetChild(i));
            }
        }

        // Return null if no items found
        if (found_items.Count <= 0) { return null; }
        return found_items;
    }

    // Returns the current value incremented by delta (or decremented) depending on what start/stop are.
    public float MoveTowards(float start, float stop, float current, float delta)
    {
        float outvalue = current;

        if (start < stop)
        {
            if (current < stop)
            { outvalue = current + delta; }

            if (outvalue > stop)
            { return stop; }

            return outvalue;
        }

        if (current > stop)
        { outvalue = current - delta; }

        if (outvalue < stop)
        { return stop; }

        return outvalue;
    }


    // Moves joint by a small delta. Returns target if destination is reached.
    // Respects hinge limits, thus if target is beyond limits it will never reach target.
    // Now implements move in the physics engine
    public struct HingeData
    {
        public bool enable;
        public HingeJoint myhinge;
        public ConfigurableJoint myhingeCJ;
        public float mytarget;
        public float myspeed;
    };

    private List<HingeData> hinge_list = new List<HingeData>();
    private int hinge_index = 0; // Points to the next free item

    // Add a new item to the hinge list
    // Maintains pool data to prevent memory releasing
    private void AddHingeToList(float target, float speed, HingeJoint hinge, ConfigurableJoint hingeCJ = null)
    {
        if (hinge_index == hinge_list.Count)
        {
            HingeData hingedata = new HingeData
            {
                enable = false,
                myhinge = null,
                myhingeCJ = null,
                mytarget = 0f,
                myspeed = 0f
            };

            hinge_list.Add(hingedata);
        }

        HingeData newdata = hinge_list[hinge_index];

        newdata.enable = true;
        newdata.myhinge = hinge;
        newdata.myhingeCJ = hingeCJ;
        newdata.mytarget = target;
        newdata.myspeed = speed;
        hinge_list[hinge_index] = newdata;

        hinge_index += 1;
    }


    private void ClearHingeList()
    {
        hinge_index = 0;
    }


    public float MoveHinge(HingeJoint hinge, float target, float speed)
    {
        if (!hinge) { return 0f; }

        // Constrain target to limits
        if (hinge.limits.max != hinge.limits.min)
        {
            if (hinge.limits.max < target) { target = hinge.limits.max; }
            if (hinge.limits.min > target) { target = hinge.limits.min; }
        }

        // Quit if we reached our target
        if (hinge.spring.targetPosition == target) { return target; }

        // Add it to our hinge_data list
        AddHingeToList(target, speed, hinge);

        // Return current position
        return hinge.spring.targetPosition;
    }

    // Hinge as aconfigurable joint - x-axis being the one to rotate
    public float MoveHinge(ConfigurableJoint hinge, float target, float speed)
    {
        if (!hinge) { return 0f; }

        // Constrain target to limits
        if (hinge.lowAngularXLimit.limit != hinge.highAngularXLimit.limit)
        {
            if (hinge.highAngularXLimit.limit < target) { target = hinge.highAngularXLimit.limit; }
            if (hinge.lowAngularXLimit.limit > target) { target = hinge.lowAngularXLimit.limit; }
        }


        // Quit if we reached our target
        if (MyUtils.AngleWrap(hinge.targetRotation.eulerAngles.x) == target) { return target; }

        // Add it to our hinge_data list
        AddHingeToList( target, speed, null, hinge);

        // Return current position
        return (float)MyUtils.AngleWrap(hinge.targetRotation.eulerAngles.x);
    }

    private void MoveAllHinges()
    {
        for( int i = 0; i < hinge_index; i++)
        {
            HingeData currdata = hinge_list[i];
            float temppos;

            if (currdata.myhinge)
            {
                temppos = currdata.myhinge.spring.targetPosition;
            }
            else if( currdata.myhingeCJ)
            {
                temppos = (float)MyUtils.AngleWrap(currdata.myhingeCJ.targetRotation.eulerAngles.x);
            }
            else
            {
                return;
            }

            temppos = MoveTowards(temppos, currdata.mytarget, temppos, Time.fixedDeltaTime * currdata.myspeed);

            if (currdata.myhinge)
            {
                JointSpring currspring = currdata.myhinge.spring;
                currspring.targetPosition = temppos;
                currdata.myhinge.spring = currspring;
            }
            else if (currdata.myhingeCJ)
            {
                Vector3 rotation_euler = currdata.myhingeCJ.targetRotation.eulerAngles;
                rotation_euler.x = temppos;
                currdata.myhingeCJ.targetRotation = Quaternion.Euler(rotation_euler);
            }


        }
    }


    // Linear slide articulation
    public enum Axis
    {
        x=0,
        y,
        z
    };

    public struct SlideData
    {
        public bool enable;
        public ConfigurableJoint myjoint;
        public Axis myaxis;
        public float mytarget;
        public float myspeed;
    };

    private List<SlideData> slide_list = new List<SlideData>();
    private int slide_index = 0; // Points to the next free item

    public float MoveSlide(ConfigurableJoint joint, Axis axis , float target, float speed)
    {
        if (!joint) { return 0f; }

        // Quit if we reached our target
        float curr_target = 0f;

        switch (axis)
        {
            case Axis.x:
                curr_target = joint.targetPosition.x;
                break;

            case Axis.y:
                curr_target = joint.targetPosition.y;
                break;

            case Axis.z:
                curr_target = joint.targetPosition.z;
                break;

        }

        if (curr_target == target) { return target; }

        // Add it to our hinge_data list
        AddSlideToList(joint, axis, target, speed);

        // Return current position
        return curr_target;
    }

    // Add a slide setting to our list
    private void AddSlideToList(ConfigurableJoint joint, Axis axis, float target, float speed)
    {
        if (slide_index == slide_list.Count)
        {
            SlideData slidedata = new SlideData
            {
                enable = false,
                myjoint = null,
                myaxis = axis,
                mytarget = 0f,
                myspeed = 0f
            };

            slide_list.Add(slidedata);
        }

        SlideData newdata = slide_list[slide_index];

        newdata.enable = true;
        newdata.myjoint = joint;
        newdata.mytarget = target;
        newdata.myspeed = speed;
        slide_list[slide_index] = newdata;

        slide_index += 1;
    }

    private void MoveAllSlides()
    {
        for (int i = 0; i < slide_index; i++)
        {
            SlideData currdata = slide_list[i];
            float temppos;

            switch( currdata.myaxis)
            {
                case Axis.x:
                    temppos = currdata.myjoint.targetPosition.x;
                    break;

                case Axis.y:
                    temppos = currdata.myjoint.targetPosition.y;
                    break;

                default:
                    temppos = currdata.myjoint.targetPosition.z;
                    break;

            }

            temppos = MoveTowards(temppos, currdata.mytarget, temppos, Time.fixedDeltaTime * currdata.myspeed);

            Vector3 target = currdata.myjoint.targetPosition;

            switch (currdata.myaxis)
            {
                case Axis.x:
                    target.x = temppos;
                    break;

                case Axis.y:
                    target.y = temppos;
                    break;

                default:
                    target.z = temppos;
                    break;

            }

            currdata.myjoint.targetPosition = target;
        }
    }

    private void ClearSlideList()
    {
        slide_index = 0;
    }

    // Scales everything to have the correct weight AND scales moment of intertia
    private void SetTotalWeight(float weight, float inertia_scale = 1f)
    {
        Rigidbody[] rigidbodies;
        float currweight = 0;

        rigidbodies = this.gameObject.GetComponentsInChildren<Rigidbody>();

        // First get the total weight in model
        foreach (Rigidbody currbody in rigidbodies)
        {
            currweight += currbody.mass;
        }

        // Next find my scaling factor
        float scaling = weight / currweight;

        Vector3 moment_of_inertia;
        // Now scale everything
        foreach (Rigidbody currbody in rigidbodies)
        {
            currbody.mass *= scaling;

            // Change the "Body's" moment of intertia
            if ( !rot_inertia_scale_only_body || (currbody.name == "Body"))
            {
                currbody.ResetInertiaTensor();
                moment_of_inertia = currbody.inertiaTensor;
                moment_of_inertia.Scale(new Vector3(inertia_scale, inertia_scale, inertia_scale));
                currbody.inertiaTensor = moment_of_inertia;
            }
        }
    }

    // *******************************************************
    // Mark and/or service state(s) + general functions
    // *******************************************************

    public float reset_duration = -1f;

    // MarkForReset
    // Marks if robot needs to be reset and if there is any holding time required
    public void MarkForReset(float duration = 0f)
    {
        reset_duration = duration;
    }

    // GetNeedsResets
    // Returns true if a reset is required
    public bool GetNeedsReset()
    {
        return reset_duration >= 0f;
    }

    // GetRestDuration
    // Returns the duration of time for reset
    public float GetResetDuration()
    {
        return (reset_duration >= 0f) ? reset_duration : 0f;
    }

    public void SetColorFromPosition(string position)
    {
        int new_color = -1;

        // Do nothing if spectator
        if (position.StartsWith("Spectator"))
        {
            return;
        }

        if (position.StartsWith("Red"))
        {
            new_color = 1;

            if (myRobotID) 
            {
                myRobotID.is_holding = false; 
            }
        }
        else if(position.StartsWith("Blue"))
        {
            new_color = 2;

            if (myRobotID)
            {
                myRobotID.is_holding = false;
            }
        }
        // Otherwise this is a holding position
        else 
        {
            new_color = 0;

            if (myRobotID)
            {
                myRobotID.is_holding = true;
            }
        }

        SetRobotColor(new_color);
    }

    private Dictionary<string, Material> material_cache = new Dictionary<string, Material>();

    public void SetRobotColor(int color, bool force = false)
    {
        // If we are already at the correct color, then do nothing
        if( !force && ( robot_color == color) ) { return; }
        Material robot_material;
        Material bumper_material;
        Material clearbumper_material=null;

        robot_color = color;
        string robotcolor;
        string bumpercolor;

        switch (color)
        {
            case 0:
                robotcolor = "Robot_holding";
                bumpercolor = "BumperHold";
                if (progressmeter)
                {
                    progressmeter.SetColor(new Color(0.1f,0.1f,0.1f));
                }
                break;
            case 1:
                robotcolor = "Robot_red";
                bumpercolor = "BumperRed";
                if (progressmeter)
                {
                    progressmeter.SetColor(new Color(1f, 0f, 0f));
                }
                break;
            case 2:
                robotcolor = "Robot_blue";
                bumpercolor = "BumperBlue";
                if (progressmeter)
                {
                    progressmeter.SetColor(new Color(0f, 0f, 1f));
                }
                break;
            case 3:
                robotcolor = "Robot_other";
                bumpercolor = "BumperOther";
                if (progressmeter)
                {
                    progressmeter.SetColor(new Color(0.7f, 0.35f, 0.7f));
                }
                break;
            default:
                robotcolor = "Robot_holding";
                bumpercolor = "BumperHold";
                if (progressmeter)
                {
                    progressmeter.SetColor(new Color(0.1f, 0.1f, 0.1f));
                }
                break;
        }

        // Cache materials
        if( material_cache.ContainsKey(robotcolor) )
        {
            robot_material = material_cache[robotcolor];
        }
        else
        {
            robot_material = Resources.Load(robotcolor, typeof(Material)) as Material;
            material_cache[robotcolor] = robot_material;
        }

        if (material_cache.ContainsKey(bumpercolor))
        {
            bumper_material = material_cache[bumpercolor];
        }
        else
        {
            bumper_material = Resources.Load(bumpercolor, typeof(Material)) as Material;
            material_cache[bumpercolor] = bumper_material;
        }

        // Add Clear versions
        switch( bumper_material.name)
        {
            case "BumperRed":
            case "BumperBlue":
                clearbumper_material = Resources.Load("Clear" + bumpercolor , typeof(Material)) as Material;
                material_cache["Clear" + bumpercolor] = bumper_material;
                break;
        }

        // Set all objects that have a "Robot" material to appropriate color
        foreach (Renderer curr_rendered in transform.root.GetComponentsInChildren<Renderer>(true))
        {
            if (curr_rendered.material.name.StartsWith("Robot") && (curr_rendered.material != robot_material))
            {
                Destroy(curr_rendered.material);
                curr_rendered.material = robot_material;
            }
        }

        // Set all objects that have a "Bumper" material to appropriate color
        foreach (Renderer curr_rendered in transform.root.GetComponentsInChildren<Renderer>(true))
        {
            int i = 0;
            for (i = 0; i < curr_rendered.materials.Length; i++)
            {

                if (curr_rendered.materials[i].name.StartsWith("Bumper"))
                {
                    Material[] all_materials = curr_rendered.materials;
                    all_materials[i] = bumper_material;
                    curr_rendered.materials = all_materials;
                }
            }
        }

        // Repeat for Clear versions
        if (clearbumper_material)
        {
            foreach (Renderer curr_rendered in transform.root.GetComponentsInChildren<Renderer>(true))
            {
                int i = 0;
                for (i = 0; i < curr_rendered.materials.Length; i++)
                {

                    if (curr_rendered.materials[i].name.StartsWith("ClearBumper"))
                    {
                        Material[] all_materials = curr_rendered.materials;
                        all_materials[i] = clearbumper_material;
                        curr_rendered.materials = all_materials;
                    }
                }
            }
        }


        // Free up memory that may be using old materials
        Resources.UnloadUnusedAssets();
    }

    // Sets the progress bar
    private float progress_value = 0f;
    public void SetProgressBar(float value)
    {
        if( !rb_body) { return; }
        if (isSpectator) { return; }

        progress_value = value;
        progressmeter.SetProgress(value);
    }

    public float GetProgressBar()
    {
        return progress_value;
    }

    public bool hold_position = false;

    // Holds the robots in place... if passed false, will release robot
    public void HoldRobot(bool state = true)
    {
        if (!rb_body) { return; }
        if (isSpectator) { return; }

        // If this is a holding, then mark robot as so
        if (!hold_position && state) {
            SetColorFromPosition("Holding");
        }
        else if( !state && hold_position )
        {
            SetColorFromPosition(myRobotID.starting_pos);
        }

        hold_position = state;
    }

    // Overrides the color with the color #
    // Set color to <0 to return to normal
    public void OverrideColor(int color)
    {
        if( color < 0 )
        {
            SetColorFromPosition(myRobotID.starting_pos);
        }
        else
        {
            SetRobotColor(color);
        }
    }

    // Disable robot into effectivelly a spectator
    public void DisableTopObjects()
    {
        disabledTopObjects = true;
        // Go through all top children and disable
        for ( int i =0; i < transform.childCount; i++)
        {
            transform.GetChild(i).gameObject.SetActive(false);
        }
    }

    virtual public void EnableTopObjects()
    {
        disabledTopObjects = false;

        // Go through all top children and enable
        for (int i = 0; i < transform.childCount; i++)
        {
            transform.GetChild(i).gameObject.SetActive(true);
        }
    }


    // *******************************************************
    // Collision Recording
    // Collision is based on the "Body" structure. 
    // A bounding box sized for the Body + margin 
    // *******************************************************

    // List of enemies touching us
    public List<RobotInterface3D> enemies = new List<RobotInterface3D>();
    public List<int> enemies_collisions = new List<int>();

    public void OnTriggerEnter(Collider collision)
    {
        // No collisions if this is spectator
        if( !rb_body ) { return;  }
        if (!rb_body.gameObject.activeSelf) { return; }
        if (isSpectator) { return; }

        // Clean up list
        RemoveInvalidItems();

        // If this is an enemy, record our collision
        AddEnemy(collision);

    }

    public void OnTriggerExit(Collider collision)
    {
        // No collisions if this is spectator
        if (!rb_body ) { return; }
        if (isSpectator) { return; }

        // Clean up list
        RemoveInvalidItems();

        // If this is an enemy, record our collision
        RemoveEnemy(collision);
    }

    // Returns true if the collision is an enemy robot
    private bool IsEnemy(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }
        if (!myRobotID) { return false;  }

        if (robotElement.is_holding || myRobotID.is_holding || 
            robotElement.is_red &&  myRobotID.is_red ||
            !robotElement.is_red && !myRobotID.is_red)
        {
            return false;
        }

        // This is an enemy
        return true;
    }

    // Adds a friend robot to the list and/or increment its collision counter
    private bool AddEnemy(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }
        if (!myRobotID) { return false; }

        if (robotElement.is_holding || myRobotID.is_holding || 
            (robotElement.is_red && myRobotID.is_red) ||
            (!robotElement.is_red && !myRobotID.is_red))
        {
            return false;
        }


        // Increment the collision count
        RobotInterface3D newrobot = topparent.GetComponent<RobotInterface3D>();
        int index = enemies.IndexOf(newrobot);

        // Increment collision counter
        if (index >= 0)
        {
            enemies_collisions[index] += 1;
        }
        else // First time this enemies is added
        {
            enemies.Add(newrobot);
            enemies_collisions.Add(1);
        }

        return true;
    }

    // Remove a friends from the list and/or remove its collision counter
    private bool RemoveEnemy(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }
        if (!myRobotID) { return false; }

        if (robotElement.is_holding || myRobotID.is_holding || 
            (robotElement.is_red && myRobotID.is_red) ||
            (!robotElement.is_red && !myRobotID.is_red))
        {
            return false;
        }


        // Decerement the collision count
        RobotInterface3D newrobot = topparent.GetComponent<RobotInterface3D>();
        int index = enemies.IndexOf(newrobot);

        // This is a friend
        if (index >= 0)
        {
            enemies_collisions[index] -= 1;
            if (enemies_collisions[index] <= 0)
            {
                enemies.RemoveAt(index);
                enemies_collisions.RemoveAt(index);
            }
        }

        return true;
    }

    private void RemoveInvalidItems()
    {
        // Remove items without a transform
        int item = enemies.Count - 1;
        while (item >= 0)
        {
            if (enemies[item] == null)
            {
                enemies.RemoveAt(item);
                enemies_collisions.RemoveAt(item);
            }

            item -= 1;
        }
    }

    // Returns true if there are enemy collisions
    public bool GetEnemiesColliding()
    {
        RemoveInvalidItems();
        return enemies.Count > 0;
    }

    public List<RobotInterface3D> GetAllEnemies()
    {
        RemoveInvalidItems();
        return enemies;
    }

    // ********* State Sync for server/client
    // Make sure the string is a small, mainly numerical value (e.g. 1:2:0.11...) so that it doesn't increase packet length much, and won't restrict compression.
    virtual public string GetStates()
    {
        // Save states that we want to synchronize over internet

        // ** Add robot color
        string outstring = robot_color.ToString();

        outstring += ":";

        // Add progress bar
        outstring += progress_value.ToString("F2");

        outstring += ":";

        // Add "disabled" state
        outstring += ((disabledTopObjects) ? "1" : "0");

        outstring += ":";

        // Add robot skin
        if ( robotskin )
        {
            outstring += robotskin.GetState();
        }

        outstring += ":";

        // Add invisibility
        outstring += ((invisible) ? "1" : "0");

        outstring += ":";

        outstring += getAverageMovementForceMagnitude().ToString("F2");

        outstring += ":";

        outstring += getAveragVelocityMagnitude().ToString("F2");

        outstring += ":";

        // Nothing else to add now
        return outstring;
    }

    // Read in the states that were turned into a string representation from GetStates()
    private bool disabledTopObjects_old = false;

    virtual public void SetStates(string instring)
    {
        string[] allstates = instring.Split(':');

        // Do holding position
        if (allstates.Length > 0)
        {
            // Save states that we want to synchronize over internet
            if (allstates[0].Length > 0)
            {
                int new_robot_color = int.Parse(allstates[0]);
                SetRobotColor(new_robot_color);
            }
        }

        // Do progress value
        if (allstates.Length > 1)
        {
            float progress_bar = float.Parse(allstates[1]);
            SetProgressBar(progress_bar);
        }

        // Disabled state
        if (allstates.Length > 2)
        {
            disabledTopObjects = (allstates[2][0] == '1');

            if(disabledTopObjects_old != disabledTopObjects)
            {
                if (disabledTopObjects) { DisableTopObjects(); }
                else                    { EnableTopObjects();  }
            }

            disabledTopObjects_old = disabledTopObjects;
        }

        // Add robot skin
        if ((allstates.Length > 3) && robotskin)
        {
            robotskin.SetState(allstates[3]);
        }

        // Get invisibility
        if ((allstates.Length > 4) && (myRobotID != null))
        {
            bool new_invisible = (allstates[4][0] == '1');

            if( new_invisible!= invisible)
            {
                if( new_invisible ) { TurnOffRenderers((GLOBALS.I_AM_RED == myRobotID.is_red) || GLOBALS.I_AM_SPECTATOR); }
                else { TurnOnRenderers(); }
            }
        }

        // Do movement and force numbers (used by movement sound generation)
        if ( (allstates.Length > 5) && (allstates[5].Length > 0))
        {
            averageMovementForceMagnitude = float.Parse(allstates[5]);        
        }

        if ((allstates.Length > 6) && (allstates[6].Length > 0))
        {
            averageVelocityMagnitude = float.Parse(allstates[6]);
        }
    }

    public void SetKinematic(bool turnon = true)
    {
        if(turnon)
        {
            TurnOffPhysics();
        }
        else
        {
            TurnOnPhysics();
        }
    }

    // Draw extras around robots
    // Highlite circle and select circle
    public void Highlite(bool state)
    {
        if( highlitecircle)
        {
            highlitecircle.SetActive(state);
        }
    }

    public void Select(bool state)
    {
        if (selectcircle)
        {
            selectcircle.SetActive(state);
        }
    }

    // Turn off all physics rigidbodies
    private void TurnOffPhysics()
    {
        isKinematic = true;

        // Remember our states if we haven't already
        if (kinematic_states.Count < 1) { RememberPhysics(); }


        Rigidbody[] rigidbodies;

        rigidbodies = gameObject.GetComponentsInChildren<Rigidbody>(true);

        // Make them all kinematic
        foreach (Rigidbody currbody in rigidbodies)
        {
            currbody.detectCollisions = false;
            currbody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            currbody.isKinematic = true;
        }
    }

    // Turn on all physics rigidbodies
    // Restores it to original state (or turns it on if new one)
    private void TurnOnPhysics()
    {
        // Only turn on physics if this is not a client
        if( GLOBALS.CLIENT_MODE) { return; }

        // If we never turned off the physics to begin with, then don't need to do anything
        if(kinematic_states.Count < 1) { return; }

        Rigidbody[] rigidbodies;

        rigidbodies = gameObject.GetComponentsInChildren<Rigidbody>(true);

        // Make them all kinematic
        foreach (Rigidbody currbody in rigidbodies)
        {
            if (kinematic_states.ContainsKey(currbody.GetInstanceID()))
            {
                currbody.isKinematic = kinematic_states[currbody.GetInstanceID()];
                currbody.collisionDetectionMode = collision_states[currbody.GetInstanceID()];
            }
            else
            {
                currbody.isKinematic = false;
            }

            currbody.detectCollisions = true;
        }

        isKinematic = false;
    }

    // Saves the current kinematic state of all rigidbodies
    private void RememberPhysics()
    {
        kinematic_states.Clear();
        collision_states.Clear();

        Rigidbody[] rigidbodies;

        rigidbodies = gameObject.GetComponentsInChildren<Rigidbody>(true);

        foreach (Rigidbody currbody in rigidbodies)
        {
            kinematic_states[currbody.GetInstanceID()] = currbody.isKinematic;
            collision_states[currbody.GetInstanceID()] = currbody.collisionDetectionMode;
        }
    }


    // Generic set hinge speed function
    // Only changes hinge speed if different. Returns true if an update occured, false if it already was set.
    public bool SetHingeSpeed(HingeJoint myhinge, float speed)
    {
        JointMotor mymotor = myhinge.motor;
        if( mymotor.targetVelocity == speed )
        {
            return false;
        }

        mymotor.targetVelocity = speed;
        myhinge.motor = mymotor;
        return true;
    }
}

// Proxy script to move collisions up to RobotInterface3D
public class PassCollisionsUp : MonoBehaviour
{
    public RobotInterface3D owner = null;

    void OnTriggerEnter(Collider other)
    {
        owner.OnTriggerEnter(other);
    }

    void OnTriggerExit(Collider other)
    {
        owner.OnTriggerExit(other);
    }
}

