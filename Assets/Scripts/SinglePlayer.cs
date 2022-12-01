using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;
using System.IO;
using UnityEngine.SceneManagement;
using UnityEngine.XR;
using UnityEngine.EventSystems;


public class SinglePlayer : MonoBehaviour
{
    private bool DEBUG = false;
    public Scorekeeper scorer;
    public string GAME = "Infinite Recharge";

    private SortedDictionary<int, GameObject> allFieldElements = new SortedDictionary<int, GameObject>();
    static bool field_load = false;
    static bool elements_load = false;
    static bool scorer_load = false;
    static bool gui_load = false;
    static bool configuration_done = false;

    // OnEnable: Open log files
    private void OnEnable()
    {
        field_load = false;
        gui_load = false;
        elements_load = false;
        scorer_load = false;
        configuration_done = false;
        GAME = GLOBALS.GAME;
        GLOBALS.SINGLEPLAYER_MODE = true;
        GLOBALS.topsingleplayer = this;

        Physics.autoSimulation = true;
        GLOBALS.game_option = 1;

        // Load field elements
        // OnLevelFinished will trigger when this has finished loading, then we can initialize more data types
        SceneManager.sceneLoaded += OnLevelFinishedLoading;
        // Load GUI Scene
        SceneManager.LoadScene("Scenes/SinglePlayer_gui", LoadSceneMode.Additive);
        // Load field
        SceneManager.LoadScene("Scenes/" + GAME + "/field", LoadSceneMode.Additive);

        SceneManager.LoadScene("Scenes/" + GAME + "/fieldElements", LoadSceneMode.Additive);


        // Load scoring elements
        // SceneManager.LoadScene("Scenes/" + GAME + "/Scoring", LoadSceneMode.Additive);

    }


    // Resets all elements to their starting position
    private void RestartLevel()
    {
        // Restart scorer
        scorer.Restart();

        // Reset our position
        if (robotRI3D) { robotRI3D.deleted = true; }
        Destroy(robotobject);

        foreach (GameObject curr in enemyrobots)
        {
            Destroy(curr);
        }
        enemyrobots.Clear();

        // Set the correct camera location
        SetCamera();

        SpawnPlayer(" ");



        // DEBUG ONLY:
        if (DEBUG) {
            // SpawnEnemy("Player 2", "Blue Left", "Tank");
            SpawnEnemy("Player 3", "Blue Right", "Mecanum");
        }

        scorer.ScorerInit();

        // Go through all game objects and reset their position
        foreach (int currkey in allFieldElements.Keys)
        {
            gameElement currelement = allFieldElements[currkey].GetComponent<gameElement>();
            if (currelement != null)
            {
                currelement.ResetPosition(GLOBALS.game_option);
            }

        }

        // Reset Timer
        scorer.OnTimerReset();

        // Start Timer
        scorer.clean_run = true;
        scorer.OnTimerClick();

        if (clean_run)
        {
            clean_run.GetComponent<Text>().text = "CLEAN";
            clean_run_details.GetComponent<Text>().text = "";
            scorer.cleancode = "";
            copy_to_clip.SetActive(false);
        }

        // Reset auto if enabled
        if( GLOBALS.AUTOMATION_FILES)
        {
            InitAutoFiles();
        }
    }



    public void FlagRequest(Dropdown menu)
    {
        if (menu.value > 0) { GLOBALS.game_option = menu.value; }
        RestartLevel();

        // Reset menu
        menu.value = 0;
        menu.itemText.text = menu.options[0].text;
    }

    public Transform scorer_overlays;
    private GameSettings ourgamesettings;

    void OnLevelFinishedLoading(Scene scene, LoadSceneMode mode)
    {
        if (scene.name == "fieldElements")
        { elements_load = true; }

        if (scene.name == "field")
        {
            field_load = true;

            // The field scene wil ldefine the ambient lighting
            SceneManager.SetActiveScene(scene);
        }

        if (scene.name == "Scoring")
        { scorer_load = true; }

        if (scene.name == "SinglePlayer_gui")
        { gui_load = true; }

        if (elements_load && field_load && scorer_load && gui_load && !configuration_done)
        {
            debugLog = GameObject.Find("DebugLogText");

            // Find all field elements and put the in our dictionary using their index number
            allFieldElements.Clear();
            GameObject[] allelements;
            allelements = GameObject.FindGameObjectsWithTag("GameElement");
            foreach (GameObject currobj in allelements)
            {
                gameElement currelement = currobj.GetComponent<gameElement>();
                if (allFieldElements.ContainsKey(currelement.id)) { Debug.Log("Field element " + currelement.id + " is not unique id."); }
                else { allFieldElements.Add(currelement.id, currobj); }

                // If it has a rigid body, set sleep to 0
                if (currobj.GetComponent<Rigidbody>())
                {
                    currobj.GetComponent<Rigidbody>().sleepThreshold = 0;
                }
            }

            // Turn off all iterpolation in case it was turned on
            ConfigureElements();

            // Find the scorekeeper
            scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper>();
            scorer_overlays = GameObject.Find("ScorerOverlays").transform;

            // Instantiate player
            SpawnPlayer(" ");

            // Set the correct camera location
            SetCamera();

            configuration_done = true;

            // New change: all camera quality levels are set that are anywhere
            // Set the camera quality level
            Camera[] allcameras = Resources.FindObjectsOfTypeAll<Camera>();

            foreach (Camera currcamera in allcameras)
            {
                MyUtils.SetCameraQualityLevel(currcamera.gameObject);
            }

            MyUtils.QualityLevel_AdjustObjects();

            ourgamesettings = FindObjectOfType<GameSettings>();

            if (ourgamesettings) { ourgamesettings.Init(); }

            // Now reset field now that game settings are good
            RestartLevel();

            scorer.clean_run = false;

            // This is redundant but kept it here 
            // MyUtils.SetCameraQualityLevel(main_camera);
        }
    }

    private void ConfigureElements()
    {
        // Turn off interpolation if accidentally turned on
        SortedDictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;

        foreach (GameObject currobj in allelements)
        {
            interpolation currinterpolation = currobj.GetComponent<interpolation>();

            if (currinterpolation != null) { currinterpolation.enabled = false; }
        }

    }


    // Looks at field elements and sees if there's an issue (falling off the board, etc..)
    private void correctFieldElements()
    {
        // Check for having flown off the grid
        SortedDictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;

        foreach (GameObject currobj in allelements)
        {
            scorer.CorrectFieldElement(currobj);
        }

    }

    private float reset_release = 0f;

    // move players from bad positions and/or reset if a reset is required
    private void CorrectPlayers()
    {
        // if object flew off board, correct it
        if (scorer.IsTransformOffField(robotRI3D.rb_body.transform))
        {
            if (robotRI3D) { robotRI3D.deleted = true; }
            Destroy(robotobject);
            SpawnPlayer(" ");
        }

        // If a reset is required, correct it
        if (robotRI3D.GetNeedsReset())
        {
            reset_release = Time.time + robotRI3D.GetResetDuration();
            if (robotRI3D) { robotRI3D.deleted = true; }
            Destroy(robotobject);
            SpawnPlayer(" ");
        }

        // If holding is required, mark it
        if (reset_release > Time.time)
        {
            robotRI3D.HoldRobot(true);
        }
        else
        {
            if (reset_release > 0f)
            {
                robotRI3D.HoldRobot(false);
                reset_release = 0f;
            }
        }

    }


    // OnDisable: close log files
    private void OnDisable()
    {

        SceneManager.sceneLoaded -= OnLevelFinishedLoading;
        GLOBALS.SINGLEPLAYER_MODE = false;
        GLOBALS.topsingleplayer = null;

    }



    // ************** User Error Message Line *********************
    private GameObject debugLog;
    private int error_count = 0;
    public void ShowError(string message)
    {
        Text textobj = debugLog.GetComponent<Text>();

        if (textobj == null)
        { return; }

        // truncate lines
        if( textobj.text.Contains("\n"))
        {
            string[] lines = textobj.text.Split('\n');

            if( lines.Length > 6 )
            {
                textobj.text = "";
                for( int i = lines.Length - 7; i < lines.Length; i++)
                {
                    textobj.text = textobj.text + lines[i] + "\n";
                }
            }
        }
      
        textobj.text = textobj.text + error_count + ":" + message + "\n";
        textobj.enabled = true;
        error_count += 1;
    }

    public void ShowMessage(string message)
    {
        ShowError(message);
    }

    public void ClearError()
    {
        Text textobj = debugLog.GetComponent<Text>();

        if (textobj == null)
        { return; }

        textobj.text = "";
        textobj.enabled = false;
    }

    // ************** Game Object Code *********************
    void Start()
    {

    }

    bool second_load = false;
    void Update()
    {
        if (!second_load && gui_load)
        {
            second_load = true;
            // Load scoring elements
            SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/Scoring", LoadSceneMode.Additive);
        }


        // Don't do anything until the scene is loaded
        if (!configuration_done)
        { return; }


        // Update robot control inputs
        InputControlUpdate();

        // Check for bad field elements
        correctFieldElements();
        CorrectPlayers();

        // Get latest score
        UpdateScore();

        //string securecode = GetMD5();
        // bool verified = VerifyMD5(scorer.GetRedScore(), scorer.GetBlueScore(), RobotPosition, securecode);

        // Deal with keyboard actions
        DoKeyboardStuff();

        // Do Auto files
        if( GLOBALS.AUTOMATION_FILES)
        {
            DoAutoFiles();
        }

        if (DEBUG)
        {
            ClearError();
            ShowError("Vel = " + robotRI3D.maxvel.ToString("#.00") + " Accel = " + robotRI3D.lastacc[4].ToString("#.00") + "," + robotRI3D.lastacc[3].ToString("#.00") + "," + robotRI3D.lastacc[2].ToString("#.00"));
        }

        // Do overlays
        ApplyOverlays();
    }

    // Generate the MD5 string for score
    private string GetMD5()
    {
        int red_score = scorer.GetRedScore();
        int blue_score = scorer.GetBlueScore();

        return MyUtils.EncryptMD5("RED=" + red_score + "BLUE=" + blue_score + "POS=" + RobotPosition + "SET=" + ((ourgamesettings) ? ourgamesettings.GetString() : "None"));
    }

    private bool VerifyMD5( int red_score, int blue_score, string position, string MD5string)
    {
        return MyUtils.IsMD5Match("RED=" + red_score + "BLUE=" + blue_score + "POS=" + position + "SET=" + ourgamesettings.GetString(), MD5string);
    }

    // Update camera to new positions
    private void LateUpdate()
    {
        UpdateTrackingCamera();
    }

    // Process key events that single player deals with
    bool key_space_last = false;
    private void DoKeyboardStuff()
    {
        bool toggle_camera = (Input.GetKey(KeyCode.Space) || GLOBALS.JoystickMap["Jcontrolls_camera"].GetButton()) && !key_space_last;
        key_space_last = Input.GetKey(KeyCode.Space) || GLOBALS.JoystickMap["Jcontrolls_camera"].GetButton();

        // Check whether to cycle camera
        if (toggle_camera)
        {
            GLOBALS.camera_follows = !GLOBALS.camera_follows;

            if( scorer)
            {
                scorer.OnCameraViewChanged(); // Let scorer know we are attempting a camera view change
            }

            DoCameraViewChanged();
        }
    }


    public void DoCameraViewChanged() // Allow others to update our camera view (based on GLOBALS setting of camera_follows)
    {
        // Reset camera to starting position
        if (!GLOBALS.camera_follows && (main_camera != null) && (player_camera != null))
        {
            main_camera.transform.position = player_camera.transform.position;
            main_camera.transform.rotation = player_camera.transform.rotation;
            if (vr_camera)
            {
                vr_camera.transform.position = vr_starting_pos;
                vr_camera.transform.rotation = vr_starting_rot;
            }

            // Correct for worldscale
            MyUtils.AdjustCameraForScale(main_camera, player_camera.transform);


            // Vector3 newpos = main_camera.transform.position;
            //newpos /= GLOBALS.worldscale / 2f;
            //main_camera.transform.position = newpos;
        }
        else if (robot_ref != null)
        {
            // Make sure it will re-create the tracker 
            robot_ref = null;
            mycameratracker.transform.parent = null;
            Destroy(mycameratracker);
            mycameratracker = null;
        }
    }


    // Updates the robotos control inputs
    private bool gamepad1_restart_old = false;
    private bool gamepad1_stop_old = false;
    public Dictionary<string, string> controls_readin = new Dictionary<string, string>();

    private void InputControlUpdate()
    {
        // Get the interface object from the robot
        if (robotobject == null) { return; }

        RobotInterface3D controller = robotobject.GetComponent<RobotInterface3D>();
        if (controller == null) { return; }

        if (GLOBALS.AUTOMATION_FILES)
        {
            controls_readin = MyUtils.GetAutoControls();
            controller.updateControlsFromHash(controls_readin);
        }
        else
        {
            // Update the controlls
            controller.updateGamepadVars();
        }

        // Restart level if button pressed
        if( controller.gamepad1_restart && !gamepad1_restart_old)
        {
            RestartLevel();
        }
        gamepad1_restart_old = controller.gamepad1_restart;

        if ( controller.gamepad1_stop && !gamepad1_stop_old)
        {
            scorer.SetTimerState(Scorekeeper.TimerState.FINISHED);
        }
        gamepad1_stop_old = controller.gamepad1_stop;

    }

    private string RobotPosition = "Red Left";

    public void SetPosition(string pos)
    {
        RobotPosition = pos;
        RestartLevel();
    }

    private GameObject player_camera;
    /// <summary>
    /// A reference to the current camera being used. Only we can set
    /// </summary>
    public static GameObject main_camera { get; private set; }
    private GameObject vr_camera;
    private Vector3 vr_starting_pos;
    private Quaternion vr_starting_rot;


    // called when player hits connect
    public bool SetCamera()
    {   
        // Set the camera position/rotation
        player_camera = GameObject.Find(RobotPosition + " Cam");
        if( player_camera == null)
        {
            player_camera = GameObject.Find("Spectator Cam");
        }

        if (!main_camera) {
            ApplicationManager mymanager = GameObject.FindObjectOfType<ApplicationManager>();
            if (mymanager)
            {
                main_camera = mymanager.MainCamera;
            }
        }

        if (MyUtils.XR_isPresent() )
        {
            if (!vr_camera)
            {
                ApplicationManager mymanager = GameObject.FindObjectOfType<ApplicationManager>();
                if (mymanager)
                {
                    vr_camera = mymanager.VRCamera;
                }
            }
        }
        else
        {
            vr_camera = null;
        }

        // vr_camera = main_camera;
        GameObject vr_camera_scaling = GameObject.Find("OVRCameraScaling");


        // Set non-VR camera
        if ((main_camera != null) && (player_camera != null))
        {
            // Turn of orbit script
            cameraOrbit orbitscript = main_camera.GetComponent<cameraOrbit>();
            if (orbitscript != null)
            { orbitscript.enabled = false; }

            // Set view point to target
            main_camera.transform.position = player_camera.transform.position;
            main_camera.transform.rotation = player_camera.transform.rotation;

            // Scale positions, fov based on world scaler
            // Some practical curve fitting:
            //    Movement in Z direction = pos.y * scaler * (ln(fov)-ln(60))
            //      where scaler = 1 is reasonable

            // First undo any fov changes
            MyUtils.AdjustCameraForScale(main_camera, player_camera.transform);

           
            /* OLD CODE
            Vector3 newpos = main_camera.transform.position;



            newpos /= GLOBALS.worldscale / 2f;
            main_camera.transform.position = newpos;

            // Field of view is a bit more complicated: need to calculate the angle difference required to maintain the same scope of objects..
            // Did a curve fit to approximate good choices near nominal setting
            float newfov = 60f * Mathf.Pow(2f / GLOBALS.worldscale, -1.15f); // sets fov = 60 for worldscale = 2
            main_camera.GetComponent<Camera>().fieldOfView = newfov;
            */
        }

        // Set VR camera
        if ((vr_camera != null) && (player_camera != null))
        {
            // Set proper scaling
            vr_camera_scaling.transform.localScale = (new Vector3(1f, 1f, 1f)) * 2f / GLOBALS.worldscale;

            // Set view point to target with a y of 0 
            Vector3 vr_cam_pos = player_camera.transform.position;
            vr_cam_pos.y = 0;
            vr_camera.transform.position = vr_cam_pos;


            Quaternion camera_rotation = player_camera.transform.rotation;
            Vector3 euler = camera_rotation.eulerAngles;
            euler.x = 0;
            euler.z = 0;
            camera_rotation.eulerAngles = euler;

            vr_camera.transform.rotation = camera_rotation;

            // Set floor height in the tracking space
            Vector3 vr_yheightpos = vr_camera_scaling.transform.localPosition;
            vr_yheightpos.y = MyUtils.GetFloorYPos();
            vr_camera_scaling.transform.localPosition = vr_yheightpos;

            // vr_camera.transform.position *= GLOBALS.worldscale / 2f;

            vr_starting_pos = vr_camera.transform.position;
            vr_starting_rot = vr_camera.transform.rotation;

        }


        return true;
    }

    // For camera tracking, ref positions for camera and robot to calculate offsets
    private Transform robot_ref = null;
    private Transform camera_ref = null;
    private GameObject mycameratracker = null; // use for calculating new camera offset

    // Updates tracking camera if applicable
    Transform cached_body = null;
    private void UpdateTrackingCamera()
    {
        if( ! GLOBALS.camera_follows || main_camera == null || GLOBALS.CAMERA_COUNTDOWN_CONTROL) { return; }

        // NOTE: while doing a lot of FIND's can be slow, one find per frame update is nothing, and it ensures any caching doesn't get stale when the robot is re-spawned.
        // MOD: now do some checks to determin if find is necessary
        if ((cached_body == null) || (cached_body.parent != robotobject.transform))
        {
            cached_body = robotobject.transform.Find("Body");
        }
        Transform body = cached_body;
        if ( body == null) { return; }

        // Initialize new camera tracker if it doesn't exist     
        if ( robot_ref == null)
        {
            robot_ref = GameObject.Find("robot_ref").transform;
            camera_ref = GameObject.Find("camera_ref").transform;
            mycameratracker = new GameObject();

            // The camera_ref and robot_ref are based on 0 rotation blocks and +z defines the robot forward movement.
            // Actual robot is designed 
            // All worldscale is removed from these calculations and done at the end
            // Vector3 camera_pos_offset = (camera_ref.position - robot_ref.position) * 2f / GLOBALS.worldscale;
            Vector3 camera_pos_offset = (camera_ref.position - robot_ref.position);
            mycameratracker.transform.position = camera_pos_offset;
            Quaternion camera_angle = mycameratracker.transform.rotation;
            camera_angle.eulerAngles = (camera_ref.rotation.eulerAngles - robot_ref.rotation.eulerAngles);

            // Do some tilt correction based on scaling
            // No longer needed!!!
            //Vector3 euler_angles = camera_angle.eulerAngles;
            //euler_angles.x -= (GLOBALS.worldscale - 2f) * 5f; // Fudge correction Tilt correction for world scale
            //camera_angle.eulerAngles = euler_angles;
            mycameratracker.transform.rotation = camera_angle;

            // Now correct it for the z-axis x-axis definition differences
            Vector3 ref_point = new Vector3(0f, 0f, 0f);
            Vector3 axis_ref = new Vector3(0f, 1f, 0f);
            mycameratracker.transform.RotateAround(ref_point, axis_ref, 90f);
            Quaternion final_camera_angle = mycameratracker.transform.rotation;

            // Now parent the tracker to the body, (which will apply body rotations to it)
            // Need to save local positions/orientations and re-apply after parenting (parenting changes it all)
            Vector3 localpos = mycameratracker.transform.localPosition;
            Vector3 localscale = mycameratracker.transform.localScale;
            Quaternion localrot = mycameratracker.transform.localRotation;

            // Now parent the tracker to the body        
            mycameratracker.transform.SetParent(body,false);

            // Ok now reset lcoals
            mycameratracker.transform.localPosition = localpos;
            mycameratracker.transform.localScale = localscale;
            mycameratracker.transform.localRotation = localrot;

            // Correct main camera to the new x,z angles
            main_camera.transform.rotation = localrot; // Setting camera rotation to the tracker's original local (un-rotated) to initialize correct x,z ang;es


            // Problem is if body wasn't level, then we just applied body non-level rotation to the camera, but we only want to apply Y axis rotation... Therefore reset x,z rotations to origina
            // Copy over the orientation (required to set the z,x rotations which won't be updated after this
            // But we do need to rome any robot tilt that we dont want
            /* camera_angle = mycameratracker.transform.rotation; 

             // Reset angles x,z
             Vector3 eulers = camera_angle.eulerAngles;
             eulers.x = final_camera_angle.eulerAngles.x;
             eulers.z = final_camera_angle.eulerAngles.z;
             camera_angle.eulerAngles = eulers;
             main_camera.transform.rotation = camera_angle;
             */

            if ( vr_camera )
            {
                vr_camera.transform.rotation = mycameratracker.transform.rotation;
            }
        }

        // Now copy transform info over to main camera
        Vector3 temp_pos = mycameratracker.transform.position;
        temp_pos.y = camera_ref.position.y;
        // temp_pos.y = camera_ref.position.y*(1f + (2f-GLOBALS.worldscale)*0.22f);  // Fix the y position
        main_camera.transform.position = temp_pos;

        // But rotation we only want to change Y
        Vector3 camera_rotation = mycameratracker.transform.rotation.eulerAngles;
        camera_rotation.x = main_camera.transform.rotation.eulerAngles.x;
        camera_rotation.z = main_camera.transform.rotation.eulerAngles.z;

        Quaternion temp_quat = main_camera.transform.rotation;
        temp_quat.eulerAngles = camera_rotation;
        main_camera.transform.rotation = temp_quat;

        // Adjust for worldscale
        MyUtils.AdjustCameraForScale(main_camera, main_camera.transform, body);


        if (vr_camera)
        {
            temp_pos.y = MyUtils.GetFloorYPos();
            vr_camera.transform.position = temp_pos;
            vr_camera.transform.rotation = temp_quat;
        }

    }

    private GameObject robotobject = null;
    private RobotInterface3D robotRI3D = null;

    private void SpawnPlayer(string playerName)
    {
        // Load the player robot
        //Correct starting position
        List<string> empty_list = new List<string>();

        Transform starting_pos = scorer.CorrectRobotPosition(RobotPosition, empty_list);
        RobotPosition = starting_pos.name;

        // Try to get the starting position
        Transform position = starting_pos;

        try
        { 
            string robotmodel = scorer.CorrectRobotChoice(GLOBALS.RobotModel);
            GLOBALS.RobotModel = robotmodel;

            // In single player, good skins are allowed
            //if ( ! LicenseData.CheckRobotIsUnlocked(robotmodel, GLOBALS.robotskins) )
            //{
            //    GLOBALS.robotskins = "Default";
            //}

            robotobject = MyUtils.InstantiateRobot(robotmodel, position.position, position.rotation, GLOBALS.skins, GLOBALS.robotskins);
        }
        catch (Exception e)
        {
            ShowError("Missing robot model for player " + playerName + ". Can't show player!");
            return;
        }

        // Set the avatar name
        robotRI3D = robotobject.GetComponent<RobotInterface3D>();

        // Set player name
        if (robotRI3D)
        {
            robotRI3D.SetName(playerName);
        }


        // Initialize all parameters of  drivetrian to globals
        robotobject.GetComponent<RobotInterface3D>().SetUserParameters();
        robotobject.GetComponent<RobotInterface3D>().fieldcentric_rotation = (player_camera) ? player_camera.transform.rotation : Quaternion.identity;

        // Assign the RobotID
        RobotID newid = robotobject.AddComponent<RobotID>() as RobotID;
        newid.starting_pos = RobotPosition;
        newid.id = 1;
        robotobject.GetComponent<RobotInterface3D>().Initialize();
        GLOBALS.client_names[newid.id] = playerName;

        // Set it's color
        robotobject.GetComponent<RobotInterface3D>().SetColorFromPosition(RobotPosition);

        // Update scorer so it can keep track of robot-to-robot collisions
        scorer.FieldChanged();

        // Clear tracking camera info so it can re-parent itself
        robot_ref = null;

        // Turn of interpolation
        TurnOffInterpolationInObject(robotobject);
    }

    private void TurnOffInterpolationInObject(GameObject inobject)
    {
        // Iterate through every child of the object
        interpolation[] allinterpolations = inobject.GetComponentsInChildren<interpolation>();

        foreach( interpolation curr in allinterpolations )
        {
            curr.enabled = false;
        }

        /*for (int j = 0; j < inobject.transform.childCount; j++)
        {
            // Get the interpolation
            interpolation currint = inobject.transform.GetChild(j).GetComponent<interpolation>();
            if (currint != null)
            {
                currint.enabled = false;
            }
        }
        */

        // Don't forget the parent
        interpolation currintp = inobject.transform.GetComponent<interpolation>();
        if (currintp != null)
        {
            currintp.enabled = false;
        }
    }

    public  List<GameObject> enemyrobots= new List<GameObject>();
    private GameObject SpawnEnemy(string playerName, string start_postion, string drivetrain)
    {
        // Load the player robot
        // Create the object
        // GameObject prefab = Resources.Load("Robots/" + GLOBALS.RobotModel) as GameObject;

        // Try to get the starting position
        GameObject starting_position = GameObject.Find(start_postion);

        Transform position = transform;

        if (starting_position != null)
        {
            position = starting_position.transform;
        }

        GameObject enemyrobot;

        enemyrobot = MyUtils.InstantiateRobot(GLOBALS.RobotModel,
        //    Vector3.zero, Quaternion.Euler(0, 0, 0), "0", "Real");        
              position.position, position.rotation,
             GLOBALS.skins, GLOBALS.robotskins); ;

        /* try
        {
            enemyrobot = Instantiate(prefab, position.position, position.rotation) as GameObject;
        }
        catch (Exception e)
        {
            ShowError("Missing robot model for player " + playerName + ". Can't show player!");
            return null;
        }
        */

        enemyrobot.GetComponent<RobotInterface3D>().SetKinematic(true);

        enemyrobot.GetComponent<RobotInterface3D>().DriveTrain = drivetrain;
        enemyrobot.GetComponent<RobotInterface3D>().max_speed = GLOBALS.speed;
        enemyrobot.GetComponent<RobotInterface3D>().max_acceleration = GLOBALS.acceleration;
        enemyrobot.GetComponent<RobotInterface3D>().total_weight = GLOBALS.weight;
        enemyrobot.GetComponent<RobotInterface3D>().turn_scale = GLOBALS.turning_scaler;
        enemyrobot.GetComponent<RobotInterface3D>().fieldcentric = GLOBALS.fieldcentric;
        enemyrobot.GetComponent<RobotInterface3D>().activebreaking = GLOBALS.activebreaking;
        enemyrobot.GetComponent<RobotInterface3D>().tankcontrol = GLOBALS.tankcontrol;


        // Set the avatar name
        enemyrobot.GetComponentInChildren<TextMesh>().text = playerName;

        // Assign the RobotID
        RobotID newid = enemyrobot.AddComponent<RobotID>() as RobotID;
        newid.starting_pos = "Blue Left";
   
        // Set it's color
        //    enemyrobot.GetComponent<RobotInterface3D>().SetColorFromPosition("Blue Left");
        // enemyrobot.GetComponent<RobotInterface3D>().Initialize();

        // Update scorer so it can keep track of robot-to-robot collisions
        scorer.FieldChanged();

        enemyrobots.Add(enemyrobot);
        
        return enemyrobot;
    }

    // Update the score field
    private GameObject redtextobj = null;
    private GameObject bluetextobj = null;
    private GameObject field_redscore = null;
    private GameObject field_bluescore = null;
    private GameObject clean_run = null;
    private GameObject clean_run_details = null;
    private GameObject copy_to_clip = null;

    public void UpdateScore()
    {
        // Make sure all scenes are loaded 
        if (scorer == null) { return; }

        // Get the red score
        if (redtextobj == null)
        { 
            redtextobj = GameObject.Find("REDSCORE");
            field_redscore = GameObject.Find("FIELD_RED");
        }


        // Make sure we found the score
        if (redtextobj == null)
        { return; }

        redtextobj.GetComponent<Text>().text = scorer.GetRedScore().ToString();
        if (field_redscore) { field_redscore.GetComponent<TMPro.TextMeshPro>().text = scorer.GetRedScore().ToString(); }

        // Get the blue score
        if (bluetextobj == null)
        { 
            bluetextobj = GameObject.Find("BLUESCORE");
            field_bluescore = GameObject.Find("FIELD_BLUE");
        }      

        // Make sure we found the score
        if (bluetextobj == null)
        { return; }

        bluetextobj.GetComponent<Text>().text = scorer.GetBlueScore().ToString();
        if (field_bluescore) { field_bluescore.GetComponent<TMPro.TextMeshPro>().text = scorer.GetBlueScore().ToString(); }

        // Get the clean_run
        if (clean_run == null)
        {
            clean_run = GameObject.Find("CLEANRUN");
            clean_run_details = GameObject.Find("CLEANRUNDETAILS");
            copy_to_clip = GameObject.Find("CopyToClip");
        }

        if (clean_run == null) { return; }


        if (scorer.clean_run)
        {
            if (scorer.IsTimerFinished()  && scorer.cleancode.Length < 1)
            {
                clean_run.SetActive(true);
                clean_run_details.SetActive(true);
                copy_to_clip.SetActive(true);

                // scorer.md5 = GetMD5();
                // clean_run.GetComponent<Text>().text = GLOBALS.VERSION + ", Date=" + System.DateTime.Now.ToString() + ",CODE=" + scorer.md5;
                // clean_run_details.GetComponent<Text>().text = RobotPosition + "," + GLOBALS.RobotModel  + "," + GLOBALS.game_option + "," + scorer.GetRedScore() + "," + scorer.GetBlueScore() + "," + ((ourgamesettings) ? ourgamesettings.GetCleanString() : "None");

                string data_to_encode = GLOBALS.VERSION + "," + 
                                        GLOBALS.GAME_INDEX + "," +
                                        System.DateTime.Now.ToString() + "," + 
                                        scorer.GetRedScore() + "," + 
                                        scorer.GetBlueScore() + "," + 
                                        RobotPosition + "," + 
                                        GLOBALS.RobotModel + "," + 
                                        ((GLOBALS.AUTOMATION_FILES) ? "AUTO," : "TELE,") +
                                        GLOBALS.game_option + "," + 
                                        ((ourgamesettings) ? ourgamesettings.GetCleanString() : "None") + "," +
                                        scorer.GetTimerText();

                string aes_string = MyUtils.EncryptAES(data_to_encode);
                scorer.cleancode = aes_string;
                scorer.CopyCleanCodeToSystemBuffer();

                clean_run_details.GetComponent<Text>().text = data_to_encode;                
                clean_run.GetComponent<Text>().text = "Code copied to clipboard. Ends with: " + aes_string.Substring(aes_string.Length-23,23);
                string aes_decrypted = MyUtils.DecryptAES(aes_string);              
                clean_run_details.GetComponent<Text>().text = aes_decrypted;

            }
            else if( !clean_run.activeSelf )
            {
                clean_run.SetActive(true);
                clean_run.GetComponent<Text>().text = "CLEAN";
                clean_run_details.GetComponent<Text>().text = "";
                scorer.cleancode = "";
                copy_to_clip.SetActive(false);
            }
        }
        else
        {
            clean_run.SetActive(false);
            copy_to_clip.SetActive(false);
        }
    }

    private string old_overlay_string = "";
    public void ApplyOverlays()
    {
        // Make sure there is a main camera
        if (main_camera == null) { return; }

        // Add any scorekeeper overlays
        // Intended give some robot settings for the current robot
        // These will be referenced to the bottom Left
    
        String new_overlay_string = scorer.GetOverlaysString(1);

        // Update overlays if they need to change
        if (!old_overlay_string.Equals(new_overlay_string))
        {
            // Replace old overlays if a change occured
            foreach( Transform child in scorer_overlays.transform)
            {
                Destroy(child.gameObject);
            }

            Transform scorekeeper_overlays = scorer.GetOverlays(1, scorer_overlays);

            /*
            if (scorekeeper_overlays)
            {
                Vector3 oldpos = scorekeeper_overlays.localPosition;
                Quaternion oldrot = scorekeeper_overlays.localRotation;
                scorekeeper_overlays.SetParent(scorer_overlays);
                scorekeeper_overlays.localPosition = oldpos;
                scorekeeper_overlays.localRotation = oldrot;
            }
            */

            old_overlay_string = new_overlay_string;
        }
        
    }

    public Dictionary<string, string> auto_data = new Dictionary<string, string>();
    public gameElement[] all_game_elements;
    public RobotID[] all_robots;
    public bool auto_initialized = false;

    public void InitAutoFiles()
    {
        auto_data.Clear();
        
        // Get all the game elements
        all_game_elements = GameObject.FindObjectsOfType<gameElement>();

        // Get all the robots
        all_robots = GameObject.FindObjectsOfType<RobotID>();

        auto_initialized = true;
    }

    public void DoAutoFiles()
    {
        // Only do stuff here if it was initialized
        if( !auto_initialized ) { return; }

        // Clear existing data
        auto_data.Clear();

        // Add scorer info
        auto_data["GAME_STATE"] = scorer.firstgamestate.ToString() + "\n" +
                                  "Time Left (ms) =" + scorer.time_total.TotalMilliseconds + "\n" + "$$\n";

        // Add all gameobjects data
        string element_data = "{\n\t\"objects\": [\n";

        foreach( gameElement currelement in all_game_elements)
        {
            // See if bandwidth helper is there 
            element_data += "\t\t{\n" +
                            "\t\t\t\"id\":" + currelement.id + ",\n" +
                            "\t\t\t\"type\":" + ((int)currelement.type) + ",\n" +
                            "\t\t\t\"name\":\"" + currelement.name + "\",\n" +
                             "\t\t\t\"global pos\":" + MyUtils.PrintVector3(currelement.transform.position) + ",\n" +
                             "\t\t\t\"global rot\":" + MyUtils.PrintVector3(currelement.transform.rotation.eulerAngles);

            if (currelement.myrb)
            {
                element_data += ",\n\t\t\t\"velocity\":" + MyUtils.PrintVector3(currelement.myrb.velocity) + ",\n" +
                                "\t\t\t\"rot velocity\":" + MyUtils.PrintVector3(currelement.myrb.angularVelocity);                 
            }

            element_data += "\n\t\t},\n";
        }
        auto_data["GameElements"] = element_data +"{}\n\t]\n}\n";

        // Add myself
        Transform myrobot = robotobject.transform;
        string myrobotdata = "{\n\t\"myrobot\": [\n"; ;

        // Add body first
        Transform body = myrobot.Find("Body");
        Quaternion body_rot = Quaternion.identity;
        Vector3 body_pos = Vector3.zero;

        // Add robot info
        myrobotdata += "\t\t{\n" +
                           "\t\t\t\"name\":\"" + "INFO" + "\"," +
                           "\n\t\t\t\"Position\":\"" + RobotPosition + "\"," +
                           "\n\t\t\t\"Model\":\"" + GLOBALS.RobotModel + "\"";

        if (robotRI3D) { myrobotdata += ",\n\t\t\t\"Counter\":" + robotRI3D.GetProgressBar(); }

        myrobotdata += "\n\t\t},\n";


        if ( body )
        {
            body_rot = body.rotation;
            body_pos = body.position;


            // See if bandwidth helper is there 
            myrobotdata += "\t\t{\n" +
                            "\t\t\t\"name\":\"" + body.name + "\",\n" +
                             "\t\t\t\"global pos\":" + MyUtils.PrintVector3(body.position) + ",\n" +
                             "\t\t\t\"global rot\":" + MyUtils.PrintVector3(body.rotation.eulerAngles);

            if (robotRI3D && robotRI3D.rb_body)
            {
                myrobotdata += ",\n\t\t\t\"velocity\":" + MyUtils.PrintVector3(robotRI3D.rb_body.velocity) + ",\n" +
                                "\t\t\t\"rot velocity\":" + MyUtils.PrintVector3(robotRI3D.rb_body.angularVelocity);
            }

                                   
            myrobotdata += "\n\t\t},\n";
        }

        // Next add everything else as a relative value
        foreach (Transform curr in myrobot)
        {
            // Skip body, already did it
            if (curr == body) { continue; }

            // Get the relative position and inverse rotate it to the body
            Vector3 my_pos_from_ref = curr.position - body_pos; // Our current position relative to ref
            my_pos_from_ref = Quaternion.Inverse(body_rot) * my_pos_from_ref; // Our unrotated position relative to ref (as if ref never moved)

            Quaternion my_angle_from_ref = Quaternion.Inverse(body_rot) * curr.rotation;

            myrobotdata += "\t\t{\n" +
                "\t\t\t\"name\":\"" + curr.name + "\",\n" +
                 "\t\t\t\"local pos\":" + MyUtils.PrintVector3(my_pos_from_ref) + ",\n" +
                 "\t\t\t\"local rot\":" + MyUtils.PrintVector3(my_angle_from_ref.eulerAngles) + "\n\t\t},";

        }

        auto_data["myRobot"] = myrobotdata + "{}\n\t]\n}\n";


        MyUtils.DoAutoFiles(auto_data);
  
    }
}

