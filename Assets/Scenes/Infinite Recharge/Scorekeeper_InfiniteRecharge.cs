using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_InfiniteRecharge : Scorekeeper {

    // Game specific settings
    InfiniteRecharge_Settings ir_settings = null;

    // all scorers
    public IR_scoringBox goal_high_red;
    public IR_scoringBox goal_mid_red;
    public IR_scoringBox goal_low_red;
    public IR_scoringBox goal_high_blue;
    public IR_scoringBox goal_mid_blue;
    public IR_scoringBox goal_low_blue;

    // Score details
    public double penalties_red; // additional points given to ref from opposing teams penalties
    public int powercells_red;
    public double score_auto_red;
    public double score_teleop_red;
    public double score_endgame_red;
    public double score_red;

    public double penalties_blue; // additional points given to ref from opposing teams penalties
    public int powercells_blue;
    int auto_pc_low_r = 0;
    int auto_pc_mid_r = 0;
    int auto_pc_high_r = 0;
    int auto_pc_low_b = 0;
    int auto_pc_mid_b = 0;
    int auto_pc_high_b = 0;
    int tele_pc_low_r = 0;
    int tele_pc_mid_r = 0;
    int tele_pc_high_r = 0;
    int tele_pc_low_b = 0;
    int tele_pc_mid_b = 0;
    int tele_pc_high_b = 0;
    bool cw_rotation_r = false;
    bool cw_position_r = false;
    bool cw_rotation_b = false;
    bool cw_position_b = false;
    int bots_parked_r = 0;
    int bots_parked_b = 0;
    int bots_hanging_r = 0;
    int bots_hanging_b = 0;
    bool switch_level_r = false;
    bool switch_level_b = false;

    public double score_auto_blue;
    public double score_teleop_blue;
    public double score_endgame_blue;
    public double score_cpanel_blue;
    public double score_cpanel_red;
    public double score_blue;

    // Ball return positions
    public Transform ballholdingred;
    public Transform ballholdingblue;
    public List<Transform> ballsheld_red = new List<Transform>();
    public List<Transform> ballsheld_blue = new List<Transform>();

    // Loading Zone Handlers
    public FRC_DepotHandler loading_blue;
    public FRC_DepotHandler loading_red;

    // Safety zones
    public IR_fieldtracker blue_safety;
    public IR_fieldtracker red_safety;

    // Wall Safety zones
    public IR_fieldtracker blue_wallsafety;
    public IR_fieldtracker red_wallsafety;

    // Robot scoring zones
    public FRC_DepotHandler floor;
    public FRC_DepotHandler bluepark;
    public FRC_DepotHandler redpark;
    public FRC_DepotHandler bluebalance;
    public FRC_DepotHandler redbalance;

    [Header("IRL FOULS")]
    public GenericFieldTracker BlueTrenchRun;
    public GenericFieldTracker BlueTrenchRun_clear;
    public GenericFieldTracker BlueTargetZone;
    public GenericFieldTracker BlueTargetZone_clear;
    public GenericFieldTracker BlueLoadingZone;
    public GenericFieldTracker BlueLoadingZone_clear;

    public GenericFieldTracker RedTrenchRun;
    public GenericFieldTracker RedTrenchRun_clear;
    public GenericFieldTracker RedTargetZone;
    public GenericFieldTracker RedTargetZone_clear;
    public GenericFieldTracker RedLoadingZone;
    public GenericFieldTracker RedLoadingZone_clear;

    [Header("Control Panel")]
    public HingeJoint controlwheel_red;
    public HingeJoint controlwheel_blue;

    public MeshRenderer wheellight_red;
    public MeshRenderer wheellight_blue;

    public Transform pp_red_lights;
    public Transform pp_blue_lights;
    public List<MeshRenderer> pp_red_array = new List<MeshRenderer>();
    public List<MeshRenderer> pp_blue_array = new List<MeshRenderer>();
    public Material pp_material_off;
    public Material pp_material_yellow;
    public Material pp_material_red;
    public Material pp_material_blue;
    public MeshRenderer[] red_stage;
    public MeshRenderer[] blue_stage;
    public Transform marker_red;
    public Transform marker_blue;
    public GameObject powerup_overlay;

    [Header("PowerUps")]
    public PowerUpScript powerup_center;
    public List<PowerUpScript> powerup_home = new List<PowerUpScript>();

 
    // My own OPR temporary placeholder
    public Dictionary<string, float> player_opr_endgame = new Dictionary<string, float>();
    public class PowerUpData
    {
        public PowerUpType mypowerup = PowerUpType.NOTUSED;
        public long duration = 0; // in ms
        public long start_time = -1; // Start time since game was started (in ms)

        public long GetRemainingTime() // in ms
        {
            return duration - (MyUtils.GetTimeMillisSinceStart() - start_time);
        }

        public void SetRemainingTime(long timeleft_ms)
        {
            start_time = MyUtils.GetTimeMillisSinceStart();
            duration = timeleft_ms;
        }

        public string GetPowerChar()
        {
            switch(mypowerup)
            {
                case PowerUpType.SPEED:
                    return "S";
                case PowerUpType.TORQUE:
                    return "T";
                case PowerUpType.INVISIBILITY:
                    return "I";
                case PowerUpType.WEAK:
                    return "W";
                case PowerUpType.SLOW:
                    return "s";
                case PowerUpType.INVERTED:
                    return "i";
            }
            return "?";
        }
    }

    class RobotStates
    {
        // Timer variables
        public bool counting_down = false;
        public bool counting_up = false;
        public float start_time = -1f;
        public bool isRed = false;
        public bool isblocking = false;
        public RobotID robotID = null;
        public RobotInterface3D robot = null;

        // IRL Fouls
        public bool immune_to_G10 = false;
        public GenericFieldTracker clear_zone = null;
        public bool immune_to_G11 = false;
        public long G10_G11_starttime = -1;

        // Power-Ups applied to this robot
        public List<PowerUpData> myoffensive_powerups = new List<PowerUpData>();
        public List<PowerUpData> mydeffensive_powerups = new List<PowerUpData>();

        // Reset robot states
        public void Reset()
        {
            counting_down = false;
            counting_up = false;
            start_time = -1f;
            isblocking = false;
            immune_to_G10 = false;
            clear_zone = null;
            G10_G11_starttime = -1;
            myoffensive_powerups.Clear();
            mydeffensive_powerups.Clear();
        }
    }

    private Vector3 old_gravity;


    private void Awake()
    {
        GLOBALS.PlayerCount = 6;
        GLOBALS.TIMER_TOTAL = 150;
        GLOBALS.TIMER_AUTO = 15;
        GLOBALS.TIMER_ENDGAME = 30;

        // Change force of gravity to suit the change-up game
        old_gravity = Physics.gravity;
        Physics.gravity = new Vector3(0, -9.81f, 0);
    }

    private void OnDestroy()
    {
        // Return gravity back to original
        Physics.gravity = old_gravity;
    }



    // Rules variables
    private GameObject fault_prefab;

    public override void ScorerInit()
    {
        ScorerReset();

        ir_settings = GameObject.Find("GameSettings").GetComponent<InfiniteRecharge_Settings>();
        fault_prefab = Resources.Load("Prefabs/FaultAnimation") as GameObject;

        // Populate the light arrays
        Transform groupa_red = pp_red_lights.Find("a");
        Transform groupb_red = pp_red_lights.Find("b");

        Transform groupa_blue = pp_blue_lights.Find("a");
        Transform groupb_blue = pp_blue_lights.Find("b");

        // Initialize light arrays
        pp_red_array.Clear();
        pp_blue_array.Clear();
        for ( int i=15; i>= 1; i--)
        {
            pp_red_array.Add(groupa_red.Find(i.ToString()).GetComponent<MeshRenderer>());
            pp_red_array.Add(groupb_red.Find(i.ToString()).GetComponent<MeshRenderer>());

            pp_blue_array.Add(groupa_blue.Find(i.ToString()).GetComponent<MeshRenderer>());
            pp_blue_array.Add(groupb_blue.Find(i.ToString()).GetComponent<MeshRenderer>());
        }

        // Initialize powerups
        powerup_center.myscorekeeper = this;
        foreach( PowerUpScript currpu in powerup_home)
        {
            currpu.myscorekeeper = this;
        }
    }


    public override void ScorerReset()
    {
        base.ScorerReset();

        penalties_red = 0;// additional points given to ref from opposing teams penalties
        powercells_red = 0;
        score_auto_red = 0;
        score_teleop_red = 0;
        score_endgame_red = 0;
        score_cpanel_red = 0;
        score_red = 0;

        penalties_blue = 0; // additional points given to ref from opposing teams penalties
        powercells_blue = 0;
        auto_pc_low_r = 0;
        auto_pc_mid_r = 0;
        auto_pc_high_r = 0;
        auto_pc_low_b = 0;
        auto_pc_mid_b = 0;
        auto_pc_high_b = 0;
        tele_pc_low_r = 0;
        tele_pc_mid_r = 0;
        tele_pc_high_r = 0;
        tele_pc_low_b = 0;
        tele_pc_mid_b = 0;
        tele_pc_high_b = 0;

        score_auto_blue = 0;
        score_teleop_blue = 0;
        score_endgame_blue = 0;
        score_cpanel_blue = 0;
        score_blue = 0;
        cw_rotation_r = false;
        cw_position_r = false;
        cw_rotation_b = false;
        cw_position_b = false;

        bots_parked_r = 0;
        bots_parked_b = 0;
        bots_hanging_r = 0;
        bots_hanging_b = 0;
        switch_level_r = false;
        switch_level_b = false;


        player_opr_endgame.Clear();
        foreach (string currbot in allrobots_byname.Keys)
        {
            player_opr_endgame[currbot] = 0;
        }

        ResetControlPanel();
        ResetRobotStates();
        ResetPowerUps();
    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        // Add this game specific data
        data["PC_R"] = powercells_red.ToString();
        data["PC_B"] = powercells_blue.ToString();
        data["AutoR"] = ((int)score_auto_red).ToString();
        data["AutoB"] = ((int)score_auto_blue).ToString();
        data["TeleR"] = ((int)score_teleop_red).ToString();
        data["TeleB"] = ((int)score_teleop_blue).ToString();
        data["EndR"] = ((int)score_endgame_red).ToString();
        data["EndB"] = ((int)score_endgame_blue).ToString();
        data["CP_R"] = ((int)score_cpanel_red).ToString();
        data["CP_B"] = ((int)score_cpanel_blue).ToString();
        data["ScoreR"] = ((int)score_red + score_redadj).ToString();
        data["ScoreB"] = ((int)score_blue + score_blueadj).ToString();
        data["PenB"] = ((int)penalties_blue).ToString();
        data["PenR"] = ((int)penalties_red).ToString();

        // Additional Data
        data["AutoPC1_R"] = ((int)auto_pc_low_r).ToString();
        data["AutoPC2_R"] = ((int)auto_pc_mid_r).ToString();
        data["AutoPC3_R"] = ((int)auto_pc_high_r).ToString();
        data["AutoPC1_B"] = ((int)auto_pc_low_b).ToString();
        data["AutoPC2_B"] = ((int)auto_pc_mid_b).ToString();
        data["AutoPC3_B"] = ((int)auto_pc_high_b).ToString();
        data["TelePC1_R"] = ((int)tele_pc_low_r).ToString();
        data["TelePC2_R"] = ((int)tele_pc_mid_r).ToString();
        data["TelePC3_R"] = ((int)tele_pc_high_r).ToString();
        data["TelePC1_B"] = ((int)tele_pc_low_b).ToString();
        data["TelePC2_B"] = ((int)tele_pc_mid_b).ToString();
        data["TelePC3_B"] = ((int)tele_pc_high_b).ToString();

        data["CW_rot_R"] = (cw_rotation_r) ? "Y" : "N";
        data["CW_pos_R"] = (cw_position_r) ? "Y" : "N";
        data["CW_rot_B"] = (cw_rotation_b) ? "Y" : "N";
        data["CW_pos_B"] = (cw_position_b) ? "Y" : "N";

        data["End_park_R"] = ((int)bots_parked_r).ToString();
        data["End_park_B"] = ((int)bots_parked_b).ToString();

        data["End_hang_R"] = ((int)bots_hanging_r).ToString();
        data["End_hang_B"] = ((int)bots_hanging_b).ToString();

        data["End_level_R"] = (switch_level_r) ? "Y" : "N";
        data["End_level_B"] = (switch_level_b) ? "Y" : "N";

    }

    public override string GetDetails(bool red = true)
    {
        // Populate the dictionary with details
        Dictionary<string, string> details = new Dictionary<string, string>();
        GetScoreDetails(details);

        // If this is client, need to merge this with score_details
        if (GLOBALS.CLIENT_MODE)
        {
            foreach (string key in MyUtils.score_details.Keys)
            {
                details[key] = MyUtils.score_details[key];
            }
        }

        string team = (red) ? "R" : "B";

        // Now output the string

        return
            "<B>AUTO Score   = </B> " + details["Auto" + team] + "\n" +
            "    # Power-Cells in Low: " + details["AutoPC1_" + team] + "\n" +
            "    # Power-Cells in Mid: " + details["AutoPC2_" + team] + "\n" +
            "    # Power-Cells in High: " + details["AutoPC3_" + team] + "\n" +
            "\n" +
            "<B>TELEOP Score =</B> " + details["Tele" + team] + "\n" +
            "    # Power-Cells in Low: " + details["TelePC1_" + team] + "\n" +
            "    # Power-Cells in Mid: " + details["TelePC2_" + team] + "\n" +
            "    # Power-Cells in High: " + details["TelePC3_" + team] + "\n" +
            "    Control Panel Rotation: " + details["CW_rot_" + team] + "\n" +
            "    Control Panel Position: " + details["CW_pos_" + team] + "\n" +
            "\n" +
            "<B>ENDGAME Score=</B> " + details["End" + team] + "\n" +
            "    # Parked: " + details["End_park_" + team] + "\n" +
            "    # Hanging: " + details["End_hang_" + team] + "\n" +
            "    Switch Level: " + details["End_level_" + team];

    }

    public override void Restart()
    {
        base.Restart();

        goal_high_red.Reset();
        goal_mid_red.Reset();
        goal_low_red.Reset();
        goal_high_blue.Reset();
        goal_mid_blue.Reset();
        goal_low_blue.Reset();

        foreach (Transform currball in ballsheld_blue)
        {
            Rigidbody mybody = currball.GetComponent<Rigidbody>();
            mybody.isKinematic = false;
            mybody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        }

        ballsheld_blue.Clear();

        foreach (Transform currball in ballsheld_red)
        {
            Rigidbody mybody = currball.GetComponent<Rigidbody>();
            mybody.isKinematic = false;
            mybody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        }

        ballsheld_red.Clear();

        ResetControlPanel();
    }


    public enum WheelStates
    {
        RESET = 0,
        READY,
        STAGE1_ARMED, // 9 Balls scored but teleop not started
        STAGE1_COMPLETE, // 9 balls scored and teleop started
        STAGE2_ARMED, // 20 balls scored
        STAGE2_ROTATED, // 3 rotations occured
        STAGE2_COMPLETE, // Color stayed stable for 2s and stage is complete
        STAGE3_ARMED, // 20 balls scored, random color chosen (one that is not currently selected)
        STAGE3_ALIGNED, // Correct color is present
        STAGE3_COMPLETE // Correct color present for 5s
    };



    public class WheelStruct
    {
        public WheelStates cw_stateMachine = WheelStates.RESET;  // Keeps state machine of control wheel
        public int cw_ballcount = 0; // Keep current ball count
        public int cw_lastcolor = 0; // Last color of the wheel
        public int cw_currcolor = 0; // Current color of the wheel
        public int cw_rotation = 0; // Amount of rotation on the wheel. + direction is clockwise, - is counter clockwise. 8 count per rotation
        public long cw_waitstart = -1; // milissecond time when count was started
        public bool cw_flash_lights = false;
        public bool cw_trench_light = false; // Turns on solid trench light
        public int cw_stage3_color = 0;
        public HingeJoint controlwheel = null;
    };

    public WheelStruct cw_red_wheel = new WheelStruct();
    public WheelStruct cw_blue_wheel = new WheelStruct();

    void ResetControlPanel()
    {
        // Reset all control panel states
        cw_red_wheel.cw_stateMachine = WheelStates.RESET;
        cw_red_wheel.cw_ballcount = 0;
        cw_red_wheel.cw_lastcolor = 0;
        cw_red_wheel.cw_currcolor = 0;
        cw_red_wheel.cw_rotation = 0;
        cw_red_wheel.cw_waitstart = -1;
        cw_red_wheel.cw_flash_lights = false;
        cw_red_wheel.cw_stage3_color = UnityEngine.Random.Range(0, 4);
        cw_red_wheel.controlwheel = controlwheel_red;
        cw_red_wheel.cw_trench_light = false;


        cw_blue_wheel.cw_stateMachine = WheelStates.RESET;
        cw_blue_wheel.cw_ballcount = 0;
        cw_blue_wheel.cw_lastcolor = 0;
        cw_blue_wheel.cw_currcolor = 0;
        cw_blue_wheel.cw_rotation = 0;
        cw_blue_wheel.cw_waitstart = -1;
        cw_blue_wheel.cw_flash_lights = false;
        cw_blue_wheel.cw_stage3_color = UnityEngine.Random.Range(0, 4);
        cw_blue_wheel.controlwheel = controlwheel_blue;
        cw_blue_wheel.cw_trench_light = false;


        // Reset lights
        PP_SetLights(pp_red_array, cw_red_wheel, pp_material_red, wheellight_red, red_stage, marker_red);
        PP_SetLights(pp_blue_array, cw_blue_wheel, pp_material_blue, wheellight_blue, blue_stage, marker_blue);

    }

    void UpdateControlPanel()
    {
        // If timerstate is not running, ignore
        if(base.timerstate != TimerState.RUNNING) { return; }

        // Update lights
        PP_SetLights(pp_red_array, cw_red_wheel, pp_material_red, wheellight_red, red_stage, marker_red);
        PP_SetLights(pp_blue_array, cw_blue_wheel, pp_material_blue, wheellight_blue, blue_stage, marker_blue);

        if (!GLOBALS.CLIENT_MODE)
        {
            // Process the state machines for the wheel
            ProcessWheelStateMachine(cw_red_wheel);
            ProcessWheelStateMachine(cw_blue_wheel);
        }
    }


    void ProcessWheelStateMachine( WheelStruct wheel)
    { 

        // Process State Machine changes
        switch (wheel.cw_stateMachine)
        {
            case WheelStates.RESET:
                wheel.cw_flash_lights = false;
                wheel.cw_trench_light = false;
                wheel.cw_stateMachine = WheelStates.READY;
                wheel.cw_ballcount = 0;
                break;

            case WheelStates.READY:
                if (wheel.cw_ballcount >= 9 + ir_settings.SHIELD_OFFSET) 
                {
                    wheel.cw_stateMachine = WheelStates.STAGE1_ARMED;
                    wheel.cw_ballcount = 0;
                }
                break;

            case WheelStates.STAGE1_ARMED:
                if ((firstgamestate == FirstGameState.TELEOP) || (firstgamestate == FirstGameState.ENDGAME))
                {
                    wheel.cw_stateMachine = WheelStates.STAGE1_COMPLETE;
                    wheel.cw_ballcount = 0;
                }
                break;

            case WheelStates.STAGE1_COMPLETE:
                if (((ir_settings.GAMEVERSION == "2020") && wheel.cw_ballcount >= 20 + ir_settings.SHIELD_OFFSET) ||
                    ((ir_settings.GAMEVERSION == "2021") && wheel.cw_ballcount >= 15 + ir_settings.SHIELD_OFFSET)
                   )
                {
                    wheel.cw_stateMachine = WheelStates.STAGE2_ARMED;
                    wheel.cw_waitstart = MyUtils.GetTimeMillis();
                    wheel.cw_rotation = 0;
                    wheel.cw_trench_light = true;
                }
                break;

            case WheelStates.STAGE2_ARMED:

                // If we reached over 3 rotations, go to next stage
                if (Math.Abs(wheel.cw_rotation) >= 24) 
                {
                    wheel.cw_stateMachine = WheelStates.STAGE2_ROTATED;
                    wheel.cw_waitstart = MyUtils.GetTimeMillis();
                    wheel.cw_flash_lights = true;
                    wheel.cw_ballcount = 0;
                    break;
                }

                // Increment/Decrement rotation
                wheel.cw_currcolor = GetWheelColor(wheel.controlwheel.angle);
                if (wheel.cw_currcolor != wheel.cw_lastcolor)
                {
                    int delta = wheel.cw_currcolor - wheel.cw_lastcolor;

                    // Incremenet rotation counter
                    if ((delta == 1) || (delta <= -2)) { wheel.cw_rotation += 1; }
                    else { wheel.cw_rotation -= 1; }

                    wheel.cw_lastcolor = wheel.cw_currcolor;
                }


                break;

            case WheelStates.STAGE2_ROTATED:
                // If we reached over 5 rotations, then reset rotation count
                if (Math.Abs(wheel.cw_rotation) >= 40)
                {
                    wheel.cw_flash_lights = false;
                    wheel.cw_rotation = 0;
                    wheel.cw_stateMachine = WheelStates.STAGE2_ARMED;
                    break;
                }

                // Detect 2s stable period
                wheel.cw_currcolor = GetWheelColor(wheel.controlwheel.angle);
                if (wheel.cw_currcolor != wheel.cw_lastcolor)
                {
                    // Record time of last color change
                    wheel.cw_waitstart = MyUtils.GetTimeMillis();

                    // Incremenet rotation counter
                    if (wheel.cw_currcolor == 0) { wheel.cw_rotation += 1; }
                    else if (wheel.cw_currcolor == 3) { wheel.cw_rotation -= 1; }
                    else { wheel.cw_rotation += wheel.cw_currcolor - wheel.cw_lastcolor; }

                    wheel.cw_lastcolor = wheel.cw_currcolor;
                }

                // If we reached over 3 rotations and color hasn't changed for 2s, complete stage
                if (MyUtils.GetTimeMillis() - wheel.cw_waitstart > 2000)
                {
                    wheel.cw_flash_lights = false;
                    wheel.cw_trench_light = false;
                    wheel.cw_stateMachine = WheelStates.STAGE2_COMPLETE;
                    wheel.cw_ballcount = 0;
                    break;
                }

                break;

            case WheelStates.STAGE2_COMPLETE:
                if (((ir_settings.GAMEVERSION == "2020") && wheel.cw_ballcount >= 20 + ir_settings.SHIELD_OFFSET) ||
                    ((ir_settings.GAMEVERSION == "2021") && wheel.cw_ballcount >= 15 + ir_settings.SHIELD_OFFSET)
                   )
                {
                    wheel.cw_stateMachine = WheelStates.STAGE3_ARMED;
                    wheel.cw_trench_light = true;

                    // Pick a random color that isn't current color
                    if (wheel.cw_stage3_color == GetWheelColor(wheel.controlwheel.angle))
                    {
                        wheel.cw_stage3_color++;

                        if (wheel.cw_stage3_color > 3)
                        {
                            wheel.cw_stage3_color = 0;
                        }
                    }
                }
                break;

            case WheelStates.STAGE3_ARMED:
                // If we are on our color, advance to next state
                if (wheel.cw_stage3_color == GetWheelColor(wheel.controlwheel.angle))
                {
                    wheel.cw_stateMachine = WheelStates.STAGE3_ALIGNED;
                    wheel.cw_waitstart = MyUtils.GetTimeMillis();
                }
                break;

            case WheelStates.STAGE3_ALIGNED:
                // If we moved off our color, go back to previous state
                if (wheel.cw_stage3_color != GetWheelColor(wheel.controlwheel.angle))
                {
                    wheel.cw_stateMachine = WheelStates.STAGE3_ARMED;
                    wheel.cw_flash_lights = false;
                    break;
                }

                // If 2s expired, flash lights
                if (MyUtils.GetTimeMillis() - wheel.cw_waitstart >= 2000)
                {
                    wheel.cw_flash_lights = true;
                }

                // If 5s expired, give the points
                if (MyUtils.GetTimeMillis() - wheel.cw_waitstart >= 5000)
                {
                    wheel.cw_stateMachine = WheelStates.STAGE3_COMPLETE;
                    wheel.cw_flash_lights = false;
                    wheel.cw_trench_light = false;
                }
                break;

            // All stages covered
            default:
                break;

        }
    }

    // Returns the color of the wheel
    // 0 = yellow
    // 1 = red
    // 2 = green
    // 3 = cyan
    int GetWheelColor(float angle)
    {
        // wheel=0 when in the middle of yellow, thus offset it by 25 in order to make testing easier
        angle += 26f;

        // If less than add 360f;
        if( angle < 0 ) { angle += 360f; }

        // If greater then 180f, then subtract 180f
        if( angle >= 179.99f ) { angle -= 180f; }

        return (int) (angle / 45f);
    }


    void PP_SetLights(List<MeshRenderer> pp_array, WheelStruct cw_wheel, Material pp_material_team, MeshRenderer wheellight, MeshRenderer[] stagelights, Transform marker)
    {
        // Make sure some lights were found
        if( pp_array.Count < 15) { return; }

        // Go through the list of lights and set their color
        for (int i = 0; i < pp_array.Count; i+=2)
        {
            // Destroy old material
            Destroy(pp_array[i].material);
            Destroy(pp_array[i + 1].material);

            switch(cw_wheel.cw_stateMachine)
            {
                // Innactive wheel - turn off lights
                case WheelStates.RESET:
                case WheelStates.STAGE3_COMPLETE:
                    pp_array[i].material = pp_material_off;
                    pp_array[i+1].material = pp_material_off;
                    break;

                case WheelStates.READY:
                    if ( ((i<20) && i < cw_wheel.cw_ballcount*4) ||
                        ((i>=20) && i < 20 + (cw_wheel.cw_ballcount-5)*2 ))
                    {
                        pp_array[i].material = pp_material_team;
                        pp_array[i+1].material = pp_material_team;
                    }
                    else
                    {
                        pp_array[i].material = pp_material_off;
                        pp_array[i + 1].material = pp_material_off;
                    }
                    break;

                case WheelStates.STAGE1_COMPLETE:
                case WheelStates.STAGE2_COMPLETE:
                    if( (ir_settings.GAMEVERSION == "2020") && (((i<10) &&  i < cw_wheel.cw_ballcount)  || ((i>=10) && i < (cw_wheel.cw_ballcount-5)*2)) ||
                        (ir_settings.GAMEVERSION == "2021") && ( i < cw_wheel.cw_ballcount*2) 
                    ) 
                    {
                        pp_array[i].material = pp_material_team;
                        pp_array[i+1].material = pp_material_team;
                    }
                    else
                    {
                        pp_array[i].material = pp_material_off;
                        pp_array[i + 1].material = pp_material_off;
                    }
                    break;

                default:
                    // Otherwise we do light chasing
                    if ( (MyUtils.GetTimeMillis() - i*33) % 500> 250)
                    {
                        pp_array[i].material = pp_material_yellow;
                        pp_array[i + 1].material = pp_material_yellow;
                    }
                    else
                    {
                        pp_array[i].material = pp_material_team;
                        pp_array[i + 1].material = pp_material_team;
                    }
                    break;

            }

        }

        // Do trench run lights
        if( cw_wheel.cw_trench_light )
        {
            // Flash lights if appropriate
            if (cw_wheel.cw_flash_lights)
            {
                wheellight.enabled = ((MyUtils.GetTimeMillis() % 1000) > 500) ? true : false;
            }
            else
            {
                wheellight.enabled = true;
            }
        }
        else
        {
            wheellight.enabled = false;
        }

        // Do stage lights
        stagelights[0].gameObject.SetActive(cw_wheel.cw_stateMachine >= WheelStates.STAGE1_COMPLETE);
        stagelights[1].gameObject.SetActive(cw_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE);
        stagelights[2].gameObject.SetActive(cw_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE);

        // Highlite target color
        if((cw_wheel.cw_stateMachine == WheelStates.STAGE3_ARMED) || (cw_wheel.cw_stateMachine == WheelStates.STAGE3_ALIGNED))
        {
            marker.gameObject.SetActive(true);
            Quaternion localrot = marker.localRotation;
            localrot.eulerAngles = new Vector3(0, -45f * cw_wheel.cw_stage3_color, 0);
            marker.localRotation = localrot;
        }
        else
        {
            marker.gameObject.SetActive(false);
        }
    }

    // Deal with power ups

    void UpdatePowerUps()
    {
        // Don't do anything in client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        // Go through robots and release any power ups
        foreach (RobotStates currbot in robotstates)
        {
            // Go through this robot's offensive power-ups
            for (int index = currbot.myoffensive_powerups.Count - 1; index >= 0; index--)
            {
                PowerUpData powerup = currbot.myoffensive_powerups[index];

                // See if this one expired
                if (powerup.GetRemainingTime() < 0)
                {
                    currbot.myoffensive_powerups.RemoveAt(index);
                }
            }

            // Go through this robot's defensive  power-ups
            for (int index = currbot.mydeffensive_powerups.Count - 1; index >= 0; index--)
            {
                PowerUpData powerup = currbot.mydeffensive_powerups[index];

                // See if this one expired
                if (powerup.GetRemainingTime() < 0)
                {
                    currbot.mydeffensive_powerups.RemoveAt(index);
                }
            }
        }

        // Service home power up
        foreach (PowerUpScript current_pu in powerup_home)
        {
            ServicePowerUp(current_pu, ir_settings.PU_HOME, ir_settings.PU_HOME_TYPE);
        }

        // Service center power-up
        ServicePowerUp(powerup_center, ir_settings.PU_CENTER, ir_settings.PU_CENTER_TYPE);

        // Update robots for their powerups
        // Go through robots and release any power ups
        foreach (RobotStates currbot in robotstates)
        {
            // Reset Robots performance
            currbot.robot.TweakPerformance(0, 0);
            currbot.robot.ScaleStickControls(1f);
            currbot.robot.TurnOnRenderers();

            // Go through this robot's offensive power-ups
            for (int index = currbot.myoffensive_powerups.Count - 1; index >= 0; index--)
            {
                PowerUpData powerup = currbot.myoffensive_powerups[index];

                PU_ApplyPowerup(currbot.robot, powerup.mypowerup);
            }

            // Go through this robot's defensive  power-ups
            for (int index = currbot.mydeffensive_powerups.Count - 1; index >= 0; index--)
            {
                PowerUpData powerup = currbot.mydeffensive_powerups[index];

                PU_ApplyPowerup(currbot.robot, powerup.mypowerup);
            }

        }
    }

    // Adds the powerup to the robots
    void PU_ApplyPowerup(RobotInterface3D robot, PowerUpType powerup)
    {
        clean_run = false;
        switch( powerup)
        {
            case PowerUpType.SPEED:
                robot.TweakPerformance(1f + (ir_settings.PU_STRENGTH/100f), 1f); // 
                break;
            case PowerUpType.TORQUE:
                robot.TweakPerformance(1f, 1f + 3f* (ir_settings.PU_STRENGTH / 100f)); // Need to reduce full speed a bit since friction/breaking will no longer slow us down as much
                break;
            case PowerUpType.INVISIBILITY:
                robot.TurnOffRenderers(true);
                break;
            case PowerUpType.WEAK:
                robot.TweakPerformance(1f, 1f /(1f + (ir_settings.PU_STRENGTH / 100f)));
                break;
            case PowerUpType.SLOW:
                robot.TweakPerformance(1f / (1f + (ir_settings.PU_STRENGTH / 100f)), 1f);
                break;
            case PowerUpType.INVERTED:
                robot.ScaleStickControls(-1f);
                break;
        }

    }

    // If a power up needs servicing (it had a collision with a robot), then
    // collect that infor
    private void ServicePowerUp(PowerUpScript current_pu, bool isenabled, int type_allowed)
    {
        // In client mode we don't do this
        if(GLOBALS.CLIENT_MODE ) { return; }

        // Service powerup
        if (current_pu.NeedsServicing() && current_pu.GetOwner())
        {
            // Mark that we serviced it
            current_pu.Serviced();

            RobotStates myrobot = GetRobotState(current_pu.GetOwner());
            if( myrobot == null) { return; }

            // Add the power-up
            PowerUpData newpu = new PowerUpData();
            newpu.start_time = MyUtils.GetTimeMillisSinceStart();
            newpu.mypowerup = current_pu.myPower;

            switch (newpu.mypowerup)
            {
                case PowerUpType.SPEED:
                case PowerUpType.TORQUE:
                case PowerUpType.INVISIBILITY:
                    // If we already have a PU and only 1 is allowed, then drop it... this should never happen
                    if( ir_settings.PU_ONLYONE && (myrobot.myoffensive_powerups.Count >= 1)) { break; }
                    newpu.duration = ir_settings.PU_OFFENSIVE_DURATION * 1000;
                    bool powerupfound = false;
                    // See if we already have a power up like this
                    foreach( PowerUpData currpowerup in myrobot.myoffensive_powerups)
                    {
                        if( currpowerup.mypowerup == newpu.mypowerup)
                        {
                            currpowerup.SetRemainingTime(newpu.GetRemainingTime());
                            powerupfound = true;
                            break;
                        }
                    }
                    
                    if( !powerupfound )
                    {
                        myrobot.myoffensive_powerups.Add(newpu);
                    }
                    break;
                case PowerUpType.WEAK:
                case PowerUpType.SLOW:
                case PowerUpType.INVERTED:
                    newpu.duration = ir_settings.PU_DEFENSIVE_DURATION * 1000;
                    foreach (RobotStates currbot in robotstates)
                    {
                        if (currbot.isRed != myrobot.isRed)
                        {
                            currbot.mydeffensive_powerups.Add(newpu);
                        }
                    }
                    break;
            }
        }

        // enable powerups that need enabling
        if (  ir_settings.ENABLE_POWERUPS && isenabled)
        {
            if (current_pu.IsDisabled() && (MyUtils.GetTimeMillisSinceStart() - current_pu.GetTimeStarted()) > ir_settings.PU_RESPAWN*1000)
            {

                // Get a list of allowed poewr-ups
                List<PowerUpType> allowed_types = new List<PowerUpType>();

                // type_allowed: 0 = offensive, 1 = defensive, 2 = both
                if ((type_allowed == 0) || (type_allowed == 2))
                {
                    if (ir_settings.PU_SPEED) { allowed_types.Add(PowerUpType.SPEED); }
                    if (ir_settings.PU_TORQUE) { allowed_types.Add(PowerUpType.TORQUE); }
                    if (ir_settings.PU_INVISIBLE) { allowed_types.Add(PowerUpType.INVISIBILITY); }
                }

                if ((type_allowed == 1) || (type_allowed == 2))
                {
                    if (ir_settings.PU_SLOW) { allowed_types.Add(PowerUpType.SLOW); }
                    if (ir_settings.PU_WEAK) { allowed_types.Add(PowerUpType.WEAK); }
                    if (ir_settings.PU_INVERTED) { allowed_types.Add(PowerUpType.INVERTED); }
                }

                // If no types allowed,exit
                if (allowed_types.Count < 1) { return; }

                // Now chose a random number
                int new_pu = UnityEngine.Random.Range(0, allowed_types.Count);

                // Now initialize the PU
                current_pu.PU_Enable(allowed_types[new_pu]);
            }
        }
        else
        {
            current_pu.PU_Disable();
        }
    }

    override public bool PU_CheckIfClearToAssign(PowerUpScript thepu, RobotInterface3D robot)
    {
        // Get the robot state
        RobotStates thisrobot = GetRobotState(robot);
        if( thisrobot == null ) { return false; }

        // Make sure multiple offensive power-ups are allowed
        if( (thisrobot.myoffensive_powerups.Count > 0) && ir_settings.PU_ONLYONE && ((int) thepu.myPower) < ((int) PowerUpType.SLOW ))
        {
            return false;
        }

        return true;
    }

    // Create overlays indicating the power-ups
    public override Transform GetOverlays(int id, Transform parent)
    {
        // Get the RobotStates
        RobotStates mystates = GetRobotStateById(id);
        if( mystates==null ) { return null; }

        // Check to see if there are any powerups
        if( mystates.mydeffensive_powerups.Count==0 && mystates.myoffensive_powerups.Count == 0) { return null; }

        // Go through the powerups and create the overlays
        // GameObject top_overlay = new GameObject();

        int y_pos = 10;
        foreach( PowerUpData currpu in mystates.myoffensive_powerups)
        {
            GameObject newoverlay = Instantiate(powerup_overlay, parent.transform);
            newoverlay.transform.Find("Time").GetComponent<Text>().text = (currpu.GetRemainingTime() / 1000).ToString();
            newoverlay.transform.Find("Type").GetComponent<Text>().text = currpu.GetPowerChar();
            Vector3 pos = newoverlay.transform.localPosition;
            pos.y = y_pos;
            newoverlay.transform.localPosition = pos;
            y_pos += 45;
        }
        foreach (PowerUpData currpu in mystates.mydeffensive_powerups)
        {
            GameObject newoverlay = Instantiate(powerup_overlay, parent.transform);
            newoverlay.transform.Find("Time").GetComponent<Text>().text = (currpu.GetRemainingTime() / 1000).ToString();
            newoverlay.transform.Find("Type").GetComponent<Text>().text = currpu.GetPowerChar();
            Vector3 pos = newoverlay.transform.localPosition;
            pos.y = y_pos;
            newoverlay.transform.localPosition = pos;
            y_pos += 45;
        }

        return parent.transform;
    }


    public override String GetOverlaysString(int id)
    {
        // Get the RobotStates
        RobotStates mystates = GetRobotStateById(id);
        if (mystates == null) { return ""; }

        // Check to see if there are any powerups
        if (mystates.mydeffensive_powerups.Count == 0 && mystates.myoffensive_powerups.Count == 0) { return ""; }

        String outdata = "";

        foreach (PowerUpData currpu in mystates.myoffensive_powerups)
        {
            outdata += ":" + ((int)currpu.mypowerup).ToString() + ":" + currpu.GetRemainingTime()/1000;
        }
        foreach (PowerUpData currpu in mystates.mydeffensive_powerups)
        {
            outdata += ":" + ((int)currpu.mypowerup).ToString() + ":" + currpu.GetRemainingTime()/1000;
        }

        return outdata;
    }


    public override void ScorerUpdate(bool last_frame = false)
    {
        // I think we can ignore everything here in client mode
        bool game_running = true;
        if (base.timerstate != TimerState.PAUSED &&
            base.timerstate != TimerState.RUNNING
            )
        {
            game_running = false;
        }

        // Do control Wheel stuff for a sec
        UpdateControlPanel();
        UpdatePowerUps();
        CalculateScores();

        // Nothign left to do if in client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        // If IRL Blockign enabled, then calculate it here
        if (ir_settings.ENABLE_G10_G11 && game_running)
        {
            Do_IRL_Blocking();
        }

        // First reset all robot blocking variable
        int index = robotstates.Count;
        while (index >= 1)
        {
            index--;
            robotstates[index].isblocking = false;
        }

        // Go through every bot and its enemies and see if a valid blocking or interference situation occured
        index = robotstates.Count;
        float time = Time.time;
        index = robotstates.Count;

        while ( index >=1 )
        {
            // Decrement index
            index--;

            RobotStates currbot = robotstates[index];

            // Skip this bot if it's been destroyed
            if( !currbot.robot ) { continue; }


            // ********* Update Progress Bars **********
            // Deal with counting up/down
            if (currbot.counting_down)
            {
                //  Check if timer expired
                if( time - currbot.start_time > ir_settings.BLOCKING_DURATION)
                {
                    currbot.counting_down = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((ir_settings.BLOCKING_DURATION - time + currbot.start_time) / ir_settings.BLOCKING_DURATION);
                }
            }

            if (currbot.counting_up)
            {
                // If blocking is disabled, then count down
                if( !ir_settings.ENABLE_BLOCKING && !ir_settings.ENABLE_WALLBLOCKING)
                {
                    // Start counting down
                    currbot.counting_up = false;
                    currbot.counting_down = true;
                    currbot.start_time = time - (ir_settings.BLOCKING_DURATION - (time - currbot.start_time));
                }
                //  Check if timer expired
                else if (time - currbot.start_time > ir_settings.BLOCKING_DURATION)
                {
                    currbot.counting_up = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                    currbot.robot.MarkForReset(ir_settings.BLOCKING_RESET_HOLDING_TIME);
                    if (game_running)
                    {
                        // Increment the OPR of the robots colliding
                        List<RobotInterface3D> enemies = currbot.robot.GetAllEnemies();
                        foreach (RobotInterface3D enemybot in enemies)
                        {
                            if (player_opr.ContainsKey(GLOBALS.client_names[enemybot.myRobotID.id]))
                            {
                                player_opr[GLOBALS.client_names[enemybot.myRobotID.id]] += ir_settings.PENALTY_BLOCKING / enemies.Count;
                            }
                            else
                            {
                                player_opr[GLOBALS.client_names[enemybot.myRobotID.id]] = ir_settings.PENALTY_BLOCKING / enemies.Count;
                            }

                        }
                            
                        if (currbot.isRed) { penalties_blue += ir_settings.PENALTY_BLOCKING; }
                        else { penalties_red += ir_settings.PENALTY_BLOCKING; }
                    }
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((time - currbot.start_time) / ir_settings.BLOCKING_DURATION);
                }

            }

            // Check for a valid blocking situation
            foreach( RobotInterface3D enemybot in currbot.robot.GetAllEnemies() )
            {
                // Get enemies state
                int enemyindex = GetRobotStateIndex(enemybot);
                if( enemyindex < 0) { continue; }
                RobotStates enemystate = robotstates[enemyindex];

                // If we are inside our safety zone more then the enemy, then count enemy up

                if (ir_settings.ENABLE_BLOCKING && currbot.isRed && red_safety.IsFriendInside(currbot.robot.transform) && (red_safety.GetClosestDistance(currbot.robot.rb_body.transform.position)+0.01f < blue_safety.GetClosestDistance(enemybot.rb_body.transform.position) )||
                    ir_settings.ENABLE_BLOCKING && !currbot.isRed && blue_safety.IsFriendInside(currbot.robot.transform) && (blue_safety.GetClosestDistance(currbot.robot.rb_body.transform.position)+0.01f < red_safety.GetClosestDistance(enemybot.rb_body.transform.position)) ||
                    ir_settings.ENABLE_WALLBLOCKING && currbot.isRed && red_wallsafety.IsFriendInside(currbot.robot.transform) && (red_wallsafety.GetClosestDistance(currbot.robot.rb_body.transform.position) + 0.01f < blue_wallsafety.GetClosestDistance(enemybot.rb_body.transform.position)) ||
                    ir_settings.ENABLE_WALLBLOCKING && !currbot.isRed && blue_wallsafety.IsFriendInside(currbot.robot.transform) && (blue_wallsafety.GetClosestDistance(currbot.robot.rb_body.transform.position) + 0.01f < red_wallsafety.GetClosestDistance(enemybot.rb_body.transform.position))
                    )
                {
                    enemystate.isblocking = true;

                    if( enemystate.counting_down)
                    {
                        enemystate.counting_down = false;
                        enemystate.counting_up = true;
                        enemystate.start_time = time - (ir_settings.BLOCKING_DURATION - (time - enemystate.start_time));
                    }
                    else if(!enemystate.counting_up)
                    {
                        enemystate.counting_up = true;
                        enemystate.start_time = time;
                    }
                }
            }

        }

        // Now go through bots again and see if a blocking bot is no longer blocking
        index = robotstates.Count;
        while (index >= 1 && (ir_settings.ENABLE_BLOCKING || ir_settings.ENABLE_WALLBLOCKING))
        {
            index--;

            if( !robotstates[index].isblocking && robotstates[index].counting_up)
            {
                robotstates[index].counting_up = false;
                robotstates[index].counting_down = true;
                robotstates[index].start_time = time - (ir_settings.BLOCKING_DURATION - (time - robotstates[index].start_time));

            }
        }

        // Next, release any balls as required
        if( loading_blue.friends.Count > 0)
        {
            foreach( Transform currball in ballsheld_blue)
            {
                // Make sure currball still exists
                if( !currball) { continue;  }

                Rigidbody mybody = currball.GetComponent<Rigidbody>();
                mybody.isKinematic = false;
                mybody.collisionDetectionMode = CollisionDetectionMode.Continuous;
            }

            ballsheld_blue.Clear();
        }

        if (loading_red.friends.Count > 0)
        {
            foreach (Transform currball in ballsheld_red)
            {
                // Make sure currball still exists
                if (!currball) { continue; }

                Rigidbody mybody = currball.GetComponent<Rigidbody>();
                mybody.isKinematic = false;
                mybody.collisionDetectionMode = CollisionDetectionMode.Continuous;
            }

            ballsheld_red.Clear();
        }
    }


    public override void RobotCounterExpired(RobotID bot)
    {
        // Do nothing because we do it in the main loop
    }

    // IRL Blocking function
    // We could do event driven code here, but the code will be much more difficult to follow since it will be divided amongst multiple classes.
    // Will do instead a polling system so all related code is contained here and is easier maintained/modified (at the price of minor performance).
    private void Do_IRL_Blocking()
    {
        // Nothing to do here in client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        // G10 rule: If any robot is inside enemies protected zone and are touched by the opposing robot, create a foul.
        //   Once a robot is fouled, it won't be eligible for another until either enemy bots are more than the clearance distance away or it moves past the clear box.
        //
        // G11 rule: If any robot is inside it's own protected zone (except for tench run) and are touched by an opponent, then create a foul.
        //   Once a robot is fouled, it won't be eligible for another foul for xs ?

        int index = robotstates.Count;

        while (index >= 1)
        {
            // Decrement index
            index--;

            RobotStates currbot = robotstates[index];

            // Skip this bot if it's been destroyed
            if (!currbot.robot) { continue; }

            // ************ Release immunity if appropriate for both G10 and G11 *******************
            G10_Release(currbot);
            G11_Release(currbot);

            // ************ Check if G10 foul should be given *******************
            // *** Note: we will only allow one foul to be present at a time, thus G11 can blank G10 and vica-versa
            // currbot: the offending bot  - is it in enemy protected zone? is it being touched by the enemy?
            bool foul = false;
            RobotID contacting_bot;

            if (currbot.isRed )
            {
                if ((contacting_bot = G10_Check(currbot, BlueTrenchRun)) != null )
                {
                    foul = true;
                    currbot.clear_zone = BlueTrenchRun_clear;
                }
                else if ( (contacting_bot = G10_Check(currbot, BlueTargetZone)) != null)
                {
                    foul = true;
                    currbot.clear_zone = BlueTargetZone_clear;
                }
                else if ( (contacting_bot = G10_Check(currbot, BlueLoadingZone)) != null)
                {
                    foul = true;
                    currbot.clear_zone = BlueLoadingZone_clear;
                }
            }
            else
            {
                if ((contacting_bot = G10_Check(currbot, RedTrenchRun)) != null)
                {
                    foul = true;
                    currbot.clear_zone = RedTrenchRun_clear;
                }
                else if ((contacting_bot = G10_Check(currbot, RedTargetZone)) != null)
                {
                    foul = true;
                    currbot.clear_zone = RedTargetZone_clear;
                }
                else if ((contacting_bot = G10_Check(currbot, RedLoadingZone)) != null)
                {
                    foul = true;
                    currbot.clear_zone = RedLoadingZone_clear;
                }
            }

            // Give the foul
            if(foul)
            {
                currbot.immune_to_G10 = true;
                currbot.G10_G11_starttime = MyUtils.GetTimeMillis();

                if (currbot.isRed)
                {
                    penalties_blue += ir_settings.PENALTY_G10_G11;
                }
                else
                {
                    penalties_red += ir_settings.PENALTY_G10_G11;
                }

                // Increment contacting bot's OPR
                player_opr[GLOBALS.client_names[contacting_bot.id]] += ir_settings.PENALTY_G10_G11;

                // Instantiate the foul
                CreateFoul(currbot.robotID.id);
            }

            // ************ Check if G11 foul should be given *******************
            // currbot: the bot that is checked to be in its own protected zone. Enemy-bots touching it would become the offending bots
            foul = false;
            RobotStates enemybot;
            if (!currbot.isRed)
            {
                enemybot = G11_Check(currbot, BlueTargetZone);
                if( enemybot == null ) { enemybot = G11_Check(currbot, BlueLoadingZone); }

                if (enemybot != null )
                {
                    foul = true;
                }
            }
            else
            {
                enemybot = G11_Check(currbot, RedTargetZone);
                if (enemybot == null) { enemybot = G11_Check(currbot, RedLoadingZone); }
                if (enemybot != null)
                {
                    foul = true;
                }
            }

            // Give the foul
            if (foul)
            {

                if (currbot.isRed)
                {
                    penalties_red += ir_settings.PENALTY_G10_G11;
                }
                else
                {
                    penalties_blue += ir_settings.PENALTY_G10_G11;
                }

                // Increment my opr
                player_opr[GLOBALS.client_names[currbot.robotID.id]] += ir_settings.PENALTY_G10_G11;

                // Instantiate the foul
                CreateFoul(enemybot.robotID.id);
            }

        }
    }

    // Checks if the currbot should create a tech foul for G10 
    private RobotID G10_Check(RobotStates currbot, GenericFieldTracker zone)
    {
        // If not inside protected zone, than no foul
        if( ! zone.IsRobotInside( currbot.robot.myRobotID)) { return null; }

        // If robot is immune, than no foul
        if( currbot.immune_to_G10 || currbot.immune_to_G11) { return null; }

        // Check if contact with enemy robots
        foreach( RobotInterface3D enemybot in allrobots)
        {
            // Drop if not a real robot (or is dead)
            if (!enemybot || !enemybot.rb_body) { continue; }

            if ( enemybot.enemies.Contains( currbot.robot) ) { return enemybot.myRobotID; }
        }

        return null;
    }

    // Check if the robot can clear it's foul immunity
    private void G10_Release(RobotStates currbot)
    {
        if ((int)(MyUtils.GetTimeMillis() - currbot.G10_G11_starttime) < ir_settings.G10_G11_BLANKING)
        {
            return;
        }

        // See if the robot is clear of the clear-zone
        if (currbot.clear_zone && !currbot.clear_zone.IsRobotInside(currbot.robotID))
        {
            currbot.immune_to_G10 = false;
            currbot.clear_zone = null;

            // Release color
            if( !currbot.immune_to_G11)
            {
                currbot.robot.OverrideColor(-1);
            }
            return;
        }

        // See if enemies are clear 
        Vector3 my_pos = currbot.robot.rb_body.position;
        my_pos.y = 0f;

        foreach (RobotInterface3D enemybot in allrobots)
        {
            // Drop if not a real robot (or is dead)
            if( !enemybot || !enemybot.rb_body) { continue; }

            // Ignore friends
            if (currbot.isRed == enemybot.myRobotID.is_red) { continue; }

            //  Check distance between bodies exceeds a threshold
            Vector3 enemy_pos = enemybot.rb_body.position;
            enemy_pos.y = 0f;

            if (Vector3.Distance(my_pos, enemy_pos) < ir_settings.G10_CLEAR_DISTANCE)
            {
                // Enemy bot is too close, can't clear yet
                return;
            }
        }


        // Ok to clear
        currbot.immune_to_G10 = false;
        currbot.clear_zone = null;

        // Release color
        if (!currbot.immune_to_G11)
        {
            currbot.robot.OverrideColor(-1);
        }
        
    }

    // Checks if the currbot should create a tech foul for G10 
    private RobotStates G11_Check(RobotStates currbot, GenericFieldTracker zone)
    {
        // If not inside protected zone, than no foul
        if (!zone.IsRobotInside(currbot.robot.myRobotID)) { return null; }

        // Check if contact with enemy robots
        foreach( RobotInterface3D enemybot in currbot.robot.enemies)
        {
            // Drop if not a real robot (or is dead)
            if (!enemybot || !enemybot.rb_body) { continue; }

            // If the enemy bot is already immune, than discount it
            RobotStates enemybot_state = robotstates[ GetRobotStateIndex(enemybot)];
            if (enemybot_state.immune_to_G10 || enemybot_state.immune_to_G11) { continue; }


            // A valid foul is detected, mark the begginning of time
            enemybot_state.immune_to_G11 = true;
            enemybot_state.G10_G11_starttime = MyUtils.GetTimeMillis();
            return enemybot_state;
        }

        return null;
    }

    private void G11_Release(RobotStates currbot)
    {
        // If Robot is immune to G11, then see if timer expired
        if( !currbot.immune_to_G11 ) { return; }

        if( (int) (MyUtils.GetTimeMillis() - currbot.G10_G11_starttime) > ir_settings.G10_G11_BLANKING)
        {
            currbot.immune_to_G11 = false;
            currbot.G10_G11_starttime = -1;

            // Release color
            if (!currbot.immune_to_G10)
            {
                currbot.robot.OverrideColor(-1);
            }
        }
    }

    // Instantiates the foul graphics
    string foul_string = "";

    private void CreateFoul( int robotid)
    {
        // Get the robot
        if( ! allrobots_byid.ContainsKey(robotid)) { return; }

        RobotInterface3D robot = allrobots_byid[robotid];
        if( ! robot.rb_body) { return; }

        // Only do if not in headless mode
        if (!GLOBALS.HEADLESS_MODE)
        {
            GameObject foulinstance = Instantiate(fault_prefab, robot.rb_body.transform.position, Quaternion.identity) as GameObject;

            // Set the parent to the body of the robot
            foulinstance.transform.SetParent(robot.rb_body.transform);
        }

        // Change the robot's color to foul/holding
        robot.OverrideColor(0);

        // If this is a server, send the request to clients
        if ( GLOBALS.SERVER_MODE)
        {
            if( foul_string.Length > 0)
            {
                foul_string += ":";
            }
            foul_string += robotid;
        }
    }

    // Function triggered when new robots show up on the field or are updated
    private List<RobotStates> robotstates = new List<RobotStates>();
    public override void FieldChangedTrigger()
    {
        List<RobotStates> newstates = new List<RobotStates>();

        // Add a new robot state for each robot on the field
        for( int index = 0; index < allrobots.Count; index++)
        {
            int robotstateindex = GetRobotStateIndex(allrobots[index]);
            if (robotstateindex < 0)
            {
                RobotStates newstate = new RobotStates();
                newstate.robot = allrobots[index];

                // During destruction wierd thing happen, make sure this is all true
                if(allrobots[index].gameObject == null) { continue;  }
                if(allrobots[index].gameObject.GetComponent<RobotID>() == null) { continue; }
                newstate.isRed = allrobots[index].gameObject.GetComponent<RobotID>().starting_pos.StartsWith("Red");
                newstate.robotID = newstate.robot.myRobotID;
                newstates.Add(newstate);
            }
            else
            {
                newstates.Add(robotstates[robotstateindex]);
            }
        }

        // Assign robotstates to the list
        robotstates = newstates;
    }

    void ResetRobotStates()
    {
        foreach( RobotStates currbot in robotstates)
        {
            currbot.Reset();

            // Reset Robot powerup features
            if( currbot.robot)
            {
                // Reset Robot performance
                currbot.robot.TweakPerformance(-1f, -1f);

                // Reset blocking stuff
                currbot.robot.SetProgressBar(0);
                currbot.robot.HoldRobot(false);
            }
        }
    }

    // Returns the robot state index for the passed robot
    // Since # of robots is max 6, we will do sequential search which is comparable in time to a hash lookup.
    private int GetRobotStateIndex( RobotInterface3D robot)
    {
        for (int index = 0; index < robotstates.Count; index++)
        {
            if( robotstates[index].robot == robot )
            { return index;  }
        }

        return -1;
    }

    private RobotStates GetRobotState(RobotInterface3D robot)
    {
        for (int index = 0; index < robotstates.Count; index++)
        {
            if (robotstates[index].robot == robot)
            { return robotstates[index]; }
        }

        return null;

    }

    private RobotStates GetRobotStateById(int id)
    {
        if( !allrobots_byid.ContainsKey(id)) { return null;  }
        return GetRobotState(allrobots_byid[id]);
    }

    private void AddToPlayerOPR( Dictionary<int,int> indata, float multiplier, bool is_red)
    {
        foreach( int id in indata.Keys)
        {
            // If this came from an enemy robot, don't count it
            // But only if there is a myRobotId

            if (!allrobots_byid.ContainsKey(id) || (allrobots_byid[id] == null) || (allrobots_byid[id].myRobotID == null))
            {
                DoFieldChanged();
            }

            if (!allrobots_byid.ContainsKey(id) || (allrobots_byid[id] == null) || (allrobots_byid[id].myRobotID == null))
            {
                continue;
            }

            // If this came from an enemy robot, don't count it
            // But only if there is a myRobotId

            if(allrobots_byid[id].myRobotID.is_red != is_red) { continue; } 

            // Add the score to our robot
            if(player_opr.ContainsKey(GLOBALS.client_names[id])) { player_opr[GLOBALS.client_names[id]] += multiplier * indata[id]; }
            else { player_opr[GLOBALS.client_names[id]] = multiplier * indata[id];  }
        }
    }

    private void CalculateScores()
    {
        // Go through all the high, mid, low goals and get ball count
        // Also add end-game stuff
        if (timerstate == TimerState.RUNNING)
        {
            score_endgame_red = 0;
            score_endgame_blue = 0;

            // Increment our ball counter

            int old_pc = powercells_red;
            powercells_red += goal_high_red.number_of_balls + goal_mid_red.number_of_balls + goal_low_red.number_of_balls;
            cw_red_wheel.cw_ballcount += powercells_red - old_pc;

            old_pc = powercells_blue;

            powercells_blue += goal_high_blue.number_of_balls + goal_mid_blue.number_of_balls + goal_low_blue.number_of_balls;
            cw_blue_wheel.cw_ballcount += powercells_blue - old_pc;

            // Score the control panel
            if (ir_settings.GAMEVERSION == "2020")
            {
                score_cpanel_red = (cw_red_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE) ? 10 : 0;
                score_cpanel_blue = (cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE) ? 10 : 0;
            }
            else if (ir_settings.GAMEVERSION == "2021")
            {
                score_cpanel_red = (cw_red_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE) ? 15 : 0;
                score_cpanel_blue = (cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE) ? 15 : 0;
            }

            cw_rotation_r = cw_red_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE;
            cw_position_r = cw_red_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE;
            cw_rotation_b = cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE;
            cw_position_b = cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE;


            score_cpanel_red += (cw_red_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE) ? 20 : 0;
            score_cpanel_blue += (cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE) ? 20 : 0;

            // Score the balls in high/mid/low
            if (time_total.TotalSeconds > 135) // Auto score
            {
                auto_pc_low_r += goal_low_red.number_of_balls;
                auto_pc_mid_r += goal_mid_red.number_of_balls;
                auto_pc_high_r += goal_high_red.number_of_balls;
                auto_pc_low_b += goal_low_blue.number_of_balls;
                auto_pc_mid_b += goal_mid_blue.number_of_balls;
                auto_pc_high_b += goal_high_blue.number_of_balls;

                score_auto_red += 6 * goal_high_red.number_of_balls + 4 * goal_mid_red.number_of_balls + 2 * goal_low_red.number_of_balls;
                score_auto_blue += 6 * goal_high_blue.number_of_balls + 4 * goal_mid_blue.number_of_balls + 2 * goal_low_blue.number_of_balls;
                AddToPlayerOPR(goal_high_red.user_ball_count, 6, true);
                AddToPlayerOPR(goal_mid_red.user_ball_count, 4, true);
                AddToPlayerOPR(goal_low_red.user_ball_count, 2, true);
                AddToPlayerOPR(goal_high_blue.user_ball_count, 6, false);
                AddToPlayerOPR(goal_mid_blue.user_ball_count, 4, false);
                AddToPlayerOPR(goal_low_blue.user_ball_count, 2, false);
            }
            else
            {
                tele_pc_low_r += goal_low_red.number_of_balls;
                tele_pc_mid_r += goal_mid_red.number_of_balls;
                tele_pc_high_r += goal_high_red.number_of_balls;
                tele_pc_low_b += goal_low_blue.number_of_balls;
                tele_pc_mid_b += goal_mid_blue.number_of_balls;
                tele_pc_high_b += goal_high_blue.number_of_balls;

                score_teleop_red += 3 * goal_high_red.number_of_balls + 2 * goal_mid_red.number_of_balls + 1 * goal_low_red.number_of_balls;
                score_teleop_blue += 3 * goal_high_blue.number_of_balls + 2 * goal_mid_blue.number_of_balls + 1 * goal_low_blue.number_of_balls;
                AddToPlayerOPR(goal_high_red.user_ball_count, 3, true);
                AddToPlayerOPR(goal_mid_red.user_ball_count, 2, true);
                AddToPlayerOPR(goal_low_red.user_ball_count, 1, true);
                AddToPlayerOPR(goal_high_blue.user_ball_count, 3, false);
                AddToPlayerOPR(goal_mid_blue.user_ball_count, 2, false);
                AddToPlayerOPR(goal_low_blue.user_ball_count, 1, false);
            }

            // End-Game
            player_opr_endgame.Clear();

            bool red_no_balance = false;
            bool blue_no_balance = false;


            // Add hanging robots
            if (time_total.TotalSeconds < 30)
            {
                bots_parked_r = 0;
                bots_parked_b = 0;
                bots_hanging_r = 0;
                bots_hanging_b = 0;
                switch_level_r = false;
                switch_level_b = false;


                // Add hanging robots that aren't touching the floor
                double red_hanging_points = 0;
                int red_hanging_count = 0;

                foreach (RobotInterface3D currbot in redbalance.GetAllFriends())
                {
                    if (!floor.IsFriendInside(currbot) && !floor.IsEnemyInside(currbot))
                    {
                        red_hanging_points += 25;
                        player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] = 25;
                        red_hanging_count++;

                    }
                    else if( redpark.IsFriendInside(currbot))
                    {
                        // It counts as parked if the robot is inside the park
                        score_endgame_red += 5;
                        player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] = 5;
                        red_no_balance = true;
                        bots_parked_r += 1;
                    }
                    else
                    {
                        red_no_balance = true;
                    }
                }
                score_endgame_red += red_hanging_points;
                bots_hanging_r = red_hanging_count;

                // Add hanging robots that aren't touching the floor
                double blue_hanging_points = 0;
                int blue_hanging_count = 0;


                foreach (RobotInterface3D currbot in bluebalance.GetAllFriends())
                {
                    if (!floor.IsFriendInside(currbot) && !floor.IsEnemyInside(currbot))
                    {
                        blue_hanging_points += 25;
                        player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] = 25;
                        blue_hanging_count++;
                    }
                    else if( bluepark.IsFriendInside(currbot))
                    {
                        // It counts as parked
                        score_endgame_blue += 5;
                        player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] = 5;
                        blue_no_balance = true;
                        bots_parked_b += 1;
                    }
                    else
                    {
                        blue_no_balance = true;
                    }
                }
                score_endgame_blue += blue_hanging_points;
                bots_hanging_b = blue_hanging_count;

                // add parked robots
                foreach (RobotInterface3D currbot in redpark.GetAllFriends())
                {
                    if (!redbalance.IsFriendInside(currbot) )
                    {
                        score_endgame_red += 5;
                        player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] = 5;
                        bots_parked_r += 1;
                    }
                    
                }

                foreach (RobotInterface3D currbot in bluepark.GetAllFriends())
                {
                    if (!bluebalance.IsFriendInside(currbot) )
                    {
                        score_endgame_blue += 5;
                        player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] = 5;
                        bots_parked_b += 1;
                    }
                    
                }

                // Lastly add balance beams being level
                if (!red_no_balance && (red_hanging_count >= 1))
                {
                    if (Math.Abs(MyUtils.AngleWrap(redbalance.transform.rotation.eulerAngles.x)) < 8)
                    {
                        score_endgame_red += 15;
                        foreach (RobotInterface3D currbot in redbalance.GetAllFriends())
                        {
                            player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] += 15/ redbalance.GetAllFriends().Count;
                        }
                        switch_level_r = true;
                    }
                }

                if (!blue_no_balance && (blue_hanging_count >= 1))
                {
                    if (Math.Abs(MyUtils.AngleWrap(bluebalance.transform.rotation.eulerAngles.x)) < 8)
                    {
                        score_endgame_blue += 15;
                        foreach (RobotInterface3D currbot in bluebalance.GetAllFriends())
                        {
                            player_opr_endgame[GLOBALS.client_names[currbot.myRobotID.id]] += 15/ bluebalance.GetAllFriends().Count;
                        }
                        switch_level_b = true;
                    }
                }
            }

        }

        // Return the balls to either their holding position, or center of field if full
        ReturnBalls(goal_high_blue, ballsheld_red, ballholdingred);
        ReturnBalls(goal_mid_blue, ballsheld_red, ballholdingred);
        ReturnBalls(goal_low_blue, ballsheld_red, ballholdingred);
        ReturnBalls(goal_high_red, ballsheld_blue, ballholdingblue);
        ReturnBalls(goal_mid_red, ballsheld_blue, ballholdingblue);
        ReturnBalls(goal_low_red, ballsheld_blue, ballholdingblue);
    }

    public override void DoTimerFinished()
    {
        // Make sure to execute base
        base.DoTimerFinished();

        // Add in the endgame stuff
        foreach( string currid in player_opr_endgame.Keys)
        {
            if( player_opr.ContainsKey(currid))
            {
                player_opr[currid] += player_opr_endgame[currid];
            }
            else
            {
                player_opr[currid] = player_opr_endgame[currid];
            }
        }

        // Add Control Panel scoring with relaxed conditions
        // Score the control panel
        if (cw_red_wheel.cw_stateMachine == WheelStates.STAGE2_ROTATED) { cw_red_wheel.cw_stateMachine = WheelStates.STAGE2_COMPLETE;  }
        if (cw_blue_wheel.cw_stateMachine == WheelStates.STAGE2_ROTATED) { cw_blue_wheel.cw_stateMachine = WheelStates.STAGE2_COMPLETE; }
        if (cw_red_wheel.cw_stateMachine == WheelStates.STAGE3_ALIGNED) { cw_red_wheel.cw_stateMachine = WheelStates.STAGE3_COMPLETE; }
        if (cw_blue_wheel.cw_stateMachine == WheelStates.STAGE3_ALIGNED) { cw_blue_wheel.cw_stateMachine = WheelStates.STAGE3_COMPLETE; }

        score_cpanel_red = (cw_red_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE) ? 10 : 0;
        score_cpanel_blue = (cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE2_COMPLETE) ? 10 : 0;
        score_cpanel_red += (cw_red_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE) ? 20 : 0;
        score_cpanel_blue += (cw_blue_wheel.cw_stateMachine >= WheelStates.STAGE3_COMPLETE) ? 20 : 0;

        // Update field lights
        PP_SetLights(pp_red_array, cw_red_wheel, pp_material_red, wheellight_red, red_stage, marker_red);
        PP_SetLights(pp_blue_array, cw_blue_wheel, pp_material_blue, wheellight_blue, blue_stage, marker_blue);
    }

    public override int GetRedScore()
    {
        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }
        score_red = score_auto_red + score_teleop_red + score_endgame_red + penalties_red + score_cpanel_red; 

        return (int) score_red + score_redadj;
    }

    public override int GetBlueScore()
    {
        if( timerstate == TimerState.FINISHED )
        { return score_bluefinal;  }

        score_blue = score_auto_blue + score_teleop_blue + score_endgame_blue + penalties_blue + score_cpanel_blue;
        return (int) (score_blue) + score_blueadj;
    }

    // Helper function to go through balls in scoring zones and return them
    private void ReturnBalls(IR_scoringBox goal, List<Transform> ballsheld, Transform ballholding)
    {
        // Return any balls that scored
        foreach (Transform currball in goal.balls)
        {
            // Reset velocities
            Rigidbody mybody = currball.GetComponent<Rigidbody>();
            mybody.velocity = Vector3.zero;
            mybody.angularVelocity = Vector3.zero;

            // if there's room for more balls to be stored, store it
            if ( ((ir_settings.GAMEVERSION == "2020") && ballsheld.Count < 15) ||
                 ((ir_settings.GAMEVERSION == "2021") && ballsheld.Count < 14)
                )
            {
                ballsheld.Add(currball);
                currball.position = ballholding.Find("Ball" + ballsheld.Count).position;
                mybody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
                mybody.isKinematic = true;
            }
            else
            {
                // Random number generator
                System.Random rand = new System.Random();
                if (ir_settings.ENABLE_BALLRETURNTOBAY)
                {
                    Vector3 newpos = ballholding.Find("Ball3").position;
                    if( newpos.z < 0 ) { newpos.z += 0.3f; }
                    else { newpos.z -= 0.25f; }

                    // Add random positioning factor
                    newpos.y += 3f * ((float)rand.NextDouble());
                    newpos.x += -0.3f + 0.6f*((float)rand.NextDouble());
                    newpos.z += -0.1f + 0.2f*((float)rand.NextDouble());
                    currball.position = newpos;
                }
                else
                {
                    currball.position = new Vector3(0.5f * (float)rand.NextDouble() - 0.25f, 3f * ((float)rand.NextDouble()) + 3f, 0.5f * (float)rand.NextDouble() - 0.25f);
                }
            }
        }
        goal.Reset();
    }

    // Do not allow bot-royale in (change to FRC shooter)
    public override string CorrectRobotChoice(string requested_robot)
    {
        if( requested_robot == "Bot Royale")
        {
            return "FRC shooter";
        }

        return requested_robot;
    }


    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

        // Send state machine states
        serverFlags["CW_SM"] = ((int)(cw_red_wheel.cw_stateMachine)).ToString() + ":" + ((int)(cw_blue_wheel.cw_stateMachine)).ToString();
        
        // Send ball counts
        serverFlags["CW_BC"] = ((int)(cw_red_wheel.cw_ballcount)).ToString() + ":" + ((int)(cw_blue_wheel.cw_ballcount)).ToString();
        
        // Send trench light state
        serverFlags["CW_TL"] = (cw_red_wheel.cw_trench_light ? "1" : "0") + ":" + (cw_red_wheel.cw_flash_lights ? "1" : "0") + ":" +
                               (cw_blue_wheel.cw_trench_light ? "1" : "0") + ":" + (cw_blue_wheel.cw_flash_lights ? "1" : "0");

        // Send target color
        serverFlags["CW_TC"] = cw_red_wheel.cw_stage3_color.ToString() + ":" + cw_blue_wheel.cw_stage3_color.ToString();

        // Send foul animation
        serverFlags["FOUL"] = foul_string;
        foul_string = "";

        // Send Powerup States
        // First send robot powerup states
        string states = "";
        foreach (RobotStates state in robotstates)
        {
            foreach (PowerUpData currpu in state.mydeffensive_powerups)
            {
                states += ":" + state.robotID.id + "," + ((int)currpu.mypowerup) + "," + currpu.GetRemainingTime().ToString();
            }

            foreach (PowerUpData currpu in state.myoffensive_powerups)
            {
                states += ":" + state.robotID.id + "," + ((int)currpu.mypowerup) + "," + currpu.GetRemainingTime().ToString();
            }
        }
        serverFlags["PU"] = states;

        // Next send field power-up states
        states = ((powerup_center.IsDisabled()) ? "0" : ((int) powerup_center.myPower).ToString());
        for(int i = 0; i < powerup_home.Count; i++)
        {
            states += ":" + ((powerup_home[i].IsDisabled()) ? "0" : ((int)powerup_home[i].myPower).ToString());
        }
        serverFlags["PUcore"] = states;


    }

    void ResetPowerUps()
    {
        for (int i = 0; i < 5; i++)
        {
            PowerUpScript powerup = (i == 0) ? powerup_center : powerup_home[i - 1];
            powerup.ClearOwner();
        }
    }

    // Receiveflags from server
    int animation_played = 0;
    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

        // Get the state machine states
        if (serverFlags.ContainsKey("CW_SM"))
        {
            string[] statemachines = serverFlags["CW_SM"].Split(':');
            if (statemachines.Length >= 2)
            {
                cw_red_wheel.cw_stateMachine = (WheelStates)Enum.ToObject(typeof(WheelStates), int.Parse(statemachines[0]));
                cw_blue_wheel.cw_stateMachine = (WheelStates)Enum.ToObject(typeof(WheelStates), int.Parse(statemachines[1]));
            }
        }

        // Get the ball counts
        if (serverFlags.ContainsKey("CW_BC"))
        {

            string[] ballcount = serverFlags["CW_BC"].Split(':');
            if (ballcount.Length >= 2)
            {
                cw_red_wheel.cw_ballcount = int.Parse(ballcount[0]);
                cw_blue_wheel.cw_ballcount = int.Parse(ballcount[1]);
            }
        }
        // Ge tthe trench lights
        if (serverFlags.ContainsKey("CW_TL"))
        {

            string[] trenchlights = serverFlags["CW_TL"].Split(':');
            if (trenchlights.Length >= 4)
            {
                cw_red_wheel.cw_trench_light = (trenchlights[0][0] == '1');
                cw_red_wheel.cw_flash_lights = (trenchlights[1][0] == '1');
                cw_blue_wheel.cw_trench_light = (trenchlights[2][0] == '1');
                cw_blue_wheel.cw_flash_lights = (trenchlights[3][0] == '1');
            }
        }

        // Get the target color
        if (serverFlags.ContainsKey("CW_TC"))
        {

            string[] targetcolor = serverFlags["CW_TC"].Split(':');
            if (targetcolor.Length >= 2)
            {
                cw_red_wheel.cw_stage3_color = int.Parse(targetcolor[0]);
                cw_blue_wheel.cw_stage3_color = int.Parse(targetcolor[1]);
            }
        }

        // Get fouls
        if (serverFlags.ContainsKey("FOUL"))
        {

            string[] foul_ids = serverFlags["FOUL"].Split(':');
            foreach (string currid in foul_ids)
            {
                if (currid.Length <= 0) { continue; }
                int id = int.Parse(currid);
                CreateFoul(id);
            }
        }

        // Get PowerUps
        if (serverFlags.ContainsKey("PU"))
        {
            // First do robot power-ups

            // Clear all existing powerups
            foreach (RobotStates currstate in robotstates)
            {
                currstate.mydeffensive_powerups.Clear();
                currstate.myoffensive_powerups.Clear();
            }

            string[] all_pus = serverFlags["PU"].Split(':');
            foreach (string currpu in all_pus)
            {
                // Drop empty strings
                if (currpu.Length <= 0) { continue; }

                // Extract data
                string[] pu_data = currpu.Split(',');

                // Make sure correct data count exists
                if (pu_data.Length != 3) { continue; }

                int id = int.Parse(pu_data[0]);
                int pu = int.Parse(pu_data[1]);
                int duration = int.Parse(pu_data[2]);

                PowerUpData newpu = new PowerUpData();
                newpu.mypowerup = (PowerUpType)pu;
                newpu.duration = duration;
                newpu.start_time = MyUtils.GetTimeMillisSinceStart();

                // Get the state for this robot and quit if none exists
                RobotStates mystate = GetRobotStateById(id);
                if (mystate == null) { continue; }

                // Populate data
                switch (newpu.mypowerup)
                {
                    case PowerUpType.SPEED:
                    case PowerUpType.TORQUE:
                    case PowerUpType.INVISIBILITY:
                        mystate.myoffensive_powerups.Add(newpu);
                        break;
                    case PowerUpType.WEAK:
                    case PowerUpType.SLOW:
                    case PowerUpType.INVERTED:
                        mystate.mydeffensive_powerups.Add(newpu);
                        break;
                }
            }
        }

        // Get PowerUps Updated
        if (serverFlags.ContainsKey("PUcore"))
        {
            string[] all_pus = serverFlags["PUcore"].Split(':');

            for (int i = 0; (i < all_pus.Length) && (i < 5); i++)
            {
                bool new_disabled = (all_pus[i][0] == '0') ? true : false;
                int new_pu = int.Parse(all_pus[i]);

                PowerUpScript powerup = (i == 0) ? powerup_center : powerup_home[i - 1];

                if (powerup.IsDisabled() != new_disabled)
                {
                    if (!new_disabled) { powerup.PU_Enable((PowerUpType)new_pu); }
                    else { powerup.PU_DisableWithAnimation(); }
                }
            }
        }
    }
}