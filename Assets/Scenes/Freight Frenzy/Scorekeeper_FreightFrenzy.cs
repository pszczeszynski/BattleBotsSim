using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_FreightFrenzy : Scorekeeper {

    // Game specific settings
    FreightFrenzy_Settings ff_settings = null;

    // Game items 
    public Transform ducks_red;  // Top transform to all red extra ducks
    public Transform ducks_blue; // Top transform to all blue extra ducks

    public Transform duck_red_pos; // Carousel position of red duck
    int red_ducks_left = 0;
    public Transform duck_blue_pos; // Carousel position of blue duck
    int blue_ducks_left = 0;

    public Rigidbody platform_red;
    public Rigidbody platform_blue;

    public GameElementCounter carousel_red_detect;
    public GameElementCounter carousel_blue_detect;

    // all scorers
    public DuckChecker duck_scorer_red;
    public DuckChecker duck_scorer_blue;

    public GenericFieldTracker red_storage;
    public GenericFieldTracker red_partial_storage;

    public GenericFieldTracker blue_storage;
    public GenericFieldTracker blue_partial_storage;

    public GenericFieldTracker red_warehouse;
    public GenericFieldTracker red_partial_warehouse;

    public GenericFieldTracker blue_warehouse;
    public GenericFieldTracker blue_partial_warehouse;

    public GenericFieldTracker red_hub_low;
    public GenericFieldTracker red_hub_mid;
    public GenericFieldTracker red_hub_high;
    public GenericFieldTracker red_hub_common;

    public GenericFieldTracker blue_hub_low;
    public GenericFieldTracker blue_hub_mid;
    public GenericFieldTracker blue_hub_high;
    public GenericFieldTracker blue_hub_common;

    public GenericFieldTracker red_hub_balance;
    public RobotCollision red_hub_robotcollision;
    public GenericFieldTracker blue_hub_balance;
    public RobotCollision blue_hub_robotcollision;

    public GenericFieldTracker red_shared_balance;
    public GenericFieldTracker blue_shared_balance;
    public RobotCollision shared_robotcollision;
    public MeshRenderer shared_hub_pole;
    public MeshRenderer red_hub_pole;
    public MeshRenderer blue_hub_pole;
    public Material red_glow;
    public Material blue_glow;
    public Material no_glow;
    public Material red_no_glow;
    public Material blue_no_glow;


    public Transform barcode_red;
    public Transform barcode_blue;


    public GameObject pushback;

    // Preload blocks
    public Transform preload_redl;
    public Transform preload_redr;
    public Transform preload_bluel;
    public Transform preload_bluer;


    private double score_red;
    private int penalties_red; // penalties to be subtracted
    private int penalties_blue;  // penalties to be subtracted
    private double score_blue;

    private Vector3 old_gravity;


    // Rules variables
    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 150;
        GLOBALS.TIMER_AUTO = 30;
        GLOBALS.TIMER_ENDGAME = 30;

        // Change force of gravity to suit the FTC game
        old_gravity = Physics.gravity;
        Physics.gravity = new Vector3(0, -9.81f * 2f, 0);
    }


    private void OnDestroy()
    {
        // Return gravity back to original
        Physics.gravity = old_gravity;
    }

    GameObject fault_prefab;
    public override void ScorerInit()
    {
        ScorerReset();

        ff_settings = GameObject.Find("GameSettings").GetComponent<FreightFrenzy_Settings>();
        fault_prefab = Resources.Load("Prefabs/FaultAnimation") as GameObject;


        // Initialize Rules
        // No rules yet
    }

    public override void ScorerReset()
    {
        base.ScorerReset();

        score_red = 0;
        score_blue = 0;
        penalties_red = 0;
        penalties_blue = 0;
        red_resets = 0;
        blue_resets = 0;

        auto_red_score = 0;
        auto_blue_score = 0;
        red_ducks_delivered = 0;
        blue_ducks_delivered = 0;

        auto_red_delivered = false;
        auto_blue_delivered = false;

        teleop_red_score = 0;
        teleop_blue_score = 0;
        endgame_red_score = 0;
        endgame_blue_score = 0;

        red_fully_sparked = 0;
        blue_fully_sparked = 0;
        red_part_sparked = 0;
        blue_part_sparked = 0;

        red_fully_wparked = 0;
        blue_fully_wparked = 0;
        red_part_wparked = 0;
        blue_part_wparked = 0;
        
        red_auto_storage = 0;
        blue_auto_storage = 0;
        red_auto_hub = 0;
        blue_auto_hub = 0;

        red_auto_bonus = 0;
        blue_auto_bonus = 0;

        red_tele_storage = 0;
        blue_tele_storage = 0;

        red_hub_1 = 0;
        blue_hub_1 = 0;
        red_hub_2 = 0;
        blue_hub_2 = 0;
        red_hub_3 = 0;
        blue_hub_3 = 0;
        red_shared = 0;
        blue_shared = 0;

        red_balance = false;
        blue_balance = false;

        red_end_fully_wparked = 0;
        blue_end_fully_wparked = 0;
        red_end_part_wparked = 0;
        blue_end_part_wparked = 0;

        red_sbalance = false;
        blue_sbalance = false;


        // Initialize ducks
        red_ducks_left = ducks_red.childCount;
        blue_ducks_left = ducks_blue.childCount;

        // Reset duck state machine
        DuckStates[] all_ducks = GameObject.FindObjectsOfType<DuckStates>();
        foreach( DuckStates curr_duck in all_ducks )
        {
            if (curr_duck.mystate != DuckStates.DuckPositions.NonCarousel)
            {
                curr_duck.mystate = DuckStates.DuckPositions.Placed;
            }
        }


        // Reset duck scorer
        duck_scorer_blue.Reset();
        duck_scorer_red.Reset();

        // Reset all gameElements to be marked as "in" the warehouse (if applicable)
        // Reset duck state machine
        gameElement[] allelements = GameObject.FindObjectsOfType<gameElement>();
        foreach (gameElement curr_element in allelements)
        {
            if( curr_element.note2 == "out")
            {
                curr_element.note2 = "in";
            }
        }

    }

    public override void Restart()
    {
        base.Restart();

    }

    // A player requested a reset or a Referee requested a reset, thus penalize their score accordingly
    public override void PlayerReset(int id, bool referee = false)
    {
        base.PlayerReset(id);

        if( !game_running) { return; }

        if( allRobotID.ContainsKey(id) )
        {
            if (!referee)
            {
                if (allRobotID[id].is_red)  { red_resets += 1;  }
                else { blue_resets += 1; }
            }
            else
            {
                if (allRobotID[id].is_red) { penalties_red += ff_settings.REF_RESET_PENALTY; }
                else { penalties_blue += ff_settings.REF_RESET_PENALTY; ; }
            }
        }
    }

    private Dictionary<int, gameElement> found_elements = new Dictionary<int, gameElement>();
    bool game_running = false;
    public override void ScorerUpdate(bool last_frame = false)
    {
        DoAnimations();
        if (GLOBALS.CLIENT_MODE) { return; }

        game_running = true;
        if (base.timerstate != TimerState.PAUSED &&
            base.timerstate != TimerState.RUNNING
            )
        {
            game_running = false;
            pushback.SetActive(false);
        }    
        else
        {
            if (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO))
            {
                pushback.SetActive(true);
            }
            else
            {
                pushback.SetActive(false);
            }
        }

        if (preload_delay > 0)
        {
            preload_delay -= 1;

            if (preload_delay == 0)
            {
                PreloadAll();
            }
        }


        CalculateScores();


    }

    public override void FieldChangedTrigger()
    {

    }

    Dictionary<string, string> details = new Dictionary<string, string>();

    public override string GetDetails(bool red = true)
    {
        // Populate the dictionary with details
        GetScoreDetails(details);

        // If this is client, need to merge this with score_details
        if (GLOBALS.CLIENT_MODE)
        {
            foreach (string key in MyUtils.score_details.Keys)
            {
                details[key] = MyUtils.score_details[key];
            }
        }

        string team = (red) ? "_R" : "_B";

        // Now output the string

        return
            "<B>AUTO Score   = </B> " + details["Auto" + team] + "\n" +
            "    Delivered Duck: " + details["AutoDel" + team] + "\n" +
            "    # Bots In Storage: " + details["AutoBotStore" + team] + "\n" +
            "    # Bots Completely In Storage: " + details["AutoBotCStore" + team] + "\n" +
            "    # Bots In Warehouse: " + details["AutoBotWare" + team] + "\n" +
            "    # Bots Completely In Warehouse: " + details["AutoBotCWare" + team] + "\n" +
            "    # Freight In Storage: " + details["AutoFStore" + team] + "\n" +
            "    # Freight On Team Hub: " + details["AutoFHub" + team] + "\n" +
            "    # Auto Bonus: " + details["AutoBonus" + team] + "\n" +
            "\n" +
            "<B>TELEOP Score =</B> " + details["Tele" + team] + "\n" +
            "    # in Storage: " + details["TeleFStore" + team] + "\n" +
            "    # on Level 1: " + details["TeleHub1" + team] + "\n" +
            "    # on Level 2: " + details["TeleHub2" + team] + "\n" +
            "    # on Level 3: " + details["TeleHub3" + team] + "\n" +
            "    # on Shared:  " + details["TeleHubS" + team] + "\n" +
            "\n" +
            "<B>ENDGAME Score=</B> " + details["End" + team] + "\n" +
            "    # Ducks Delivered: " + details["EndDel" + team] + "\n" +
            "    Team Hub Balanced: " + details["EndBal" + team] + "\n" +
            "    Shared Hub Unbalanced: " + details["EndSBal" + team] + "\n" +
            "    # Bots In Warehouse: " + details["EndBotWare" + team] + "\n" +
            "    # Bots Completely In Warehouse: " + details["EndBotCWare" + team] + "\n" +
            "\n" +
            "<B>Resets = </B> " + details["Resets" + team] + "\n" + 
            "<B>PENALTIES = </B> " + details["Pen" + team];
    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        data["Pen_B"] = ((int)penalties_blue).ToString();
        data["Pen_R"] = ((int)penalties_red).ToString();
        data["Score_R"] = ((int)score_red + score_redadj).ToString();
        data["Score_B"] = ((int)score_blue + score_blueadj).ToString();
        data["Resets_B"] = ((int)red_resets).ToString();
        data["Resets_R"] = ((int)blue_resets).ToString();

        data["Auto_R"] = ((int)auto_red_score ).ToString();
        data["Auto_B"] = ((int)auto_blue_score ).ToString();
        data["AutoDel_R"] = (auto_red_delivered) ? "Y" : "N";
        data["AutoDel_B"] = (auto_blue_delivered) ? "Y" : "N";
        data["AutoBotStore_R"] = ((int)red_part_sparked).ToString();
        data["AutoBotCStore_R"] = ((int)red_fully_sparked).ToString();
        data["AutoBotStore_B"] = ((int)blue_part_sparked).ToString();
        data["AutoBotCStore_B"] = ((int)blue_fully_sparked).ToString();
        data["AutoBotWare_R"] = ((int)red_part_wparked).ToString();
        data["AutoBotCWare_R"] = ((int)red_fully_wparked).ToString();
        data["AutoBotWare_B"] = ((int)blue_part_wparked).ToString();
        data["AutoBotCWare_B"] = ((int)blue_fully_wparked).ToString();
        data["AutoFStore_R"] = ((int)red_auto_storage).ToString();
        data["AutoFStore_B"] = ((int)blue_auto_storage).ToString();
        data["AutoFHub_R"] = ((int)red_auto_hub).ToString();
        data["AutoFHub_B"] = ((int)blue_auto_hub).ToString();
        data["AutoBonus_R"] = ((int)red_auto_bonus).ToString();
        data["AutoBonus_B"] = ((int)blue_auto_bonus).ToString();

        data["Tele_R"] = ((int)teleop_red_score).ToString();
        data["Tele_B"] = ((int)teleop_blue_score).ToString();
        data["TeleFStore_R"] = ((int)red_tele_storage).ToString();
        data["TeleFStore_B"] = ((int)blue_tele_storage).ToString();
        data["TeleHub1_R"] = ((int)red_hub_1).ToString();
        data["TeleHub1_B"] = ((int)blue_hub_1).ToString();
        data["TeleHub2_R"] = ((int)red_hub_2).ToString();
        data["TeleHub2_B"] = ((int)blue_hub_2).ToString();
        data["TeleHub3_R"] = ((int)red_hub_3).ToString();
        data["TeleHub3_B"] = ((int)blue_hub_3).ToString();
        data["TeleHubS_R"] = ((int)red_shared).ToString();
        data["TeleHubS_B"] = ((int)blue_shared).ToString();

        data["End_R"] = ((int)endgame_red_score).ToString();
        data["End_B"] = ((int)endgame_blue_score).ToString();
        data["EndBal_R"] = (red_balance) ? "Y" : "N";
        data["EndBal_B"] = (blue_balance) ? "Y" : "N";
        data["EndBotWare_R"] = ((int)red_end_part_wparked).ToString();
        data["EndBotCWare_R"] = ((int)red_end_fully_wparked).ToString();
        data["EndBotWare_B"] = ((int)blue_end_part_wparked).ToString();
        data["EndBotCWare_B"] = ((int)blue_end_fully_wparked).ToString();
        data["EndSBal_R"] = (red_sbalance) ? "Y" : "N";
        data["EndSBal_B"] = (blue_sbalance) ? "Y" : "N";

        data["EndDel_R"] = ((int)red_ducks_delivered).ToString();
        data["EndDel_B"] = ((int)blue_ducks_delivered).ToString();
    }

    int red_resets = 0;
    int blue_resets = 0;
    int auto_red_score = 0;   // Total auto score (without penalties)
    int auto_blue_score = 0;
    int teleop_red_score = 0;
    int teleop_blue_score = 0;
    int endgame_red_score = 0;
    int endgame_blue_score = 0;
    bool auto_red_delivered = false;
    bool auto_blue_delivered = false;

    int red_ducks_delivered = 0; // Ducks delivered in endgame
    int blue_ducks_delivered = 0; // Ducks delivered in endgame

    int red_fully_sparked = 0;
    int blue_fully_sparked = 0;
    int red_part_sparked = 0;
    int blue_part_sparked = 0;

    int red_fully_wparked = 0;
    int blue_fully_wparked = 0;
    int red_part_wparked = 0;
    int blue_part_wparked = 0;

    int red_auto_storage = 0;
    int blue_auto_storage = 0;
    int red_auto_hub = 0;
    int blue_auto_hub = 0;
    int red_auto_bonus = 0;
    int blue_auto_bonus = 0;

    int red_tele_storage = 0;
    int blue_tele_storage = 0;
    int red_hub_1 = 0;
    int blue_hub_1 = 0;
    int red_hub_2 = 0;
    int blue_hub_2 = 0;
    int red_hub_3 = 0;
    int blue_hub_3 = 0;
    int red_shared = 0;
    int blue_shared = 0;

    bool red_balance = false;
    bool blue_balance = false;

    int red_end_fully_wparked = 0;
    int blue_end_fully_wparked = 0;
    int red_end_part_wparked = 0;
    int blue_end_part_wparked = 0;

    bool red_sbalance = false;
    bool blue_sbalance = false;

    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if (timerstate != TimerState.RUNNING)
        { return; }


        // *********************************************
        // *********************************************
        // Score Auto
        // *********************************************
        // *********************************************

        if (time_total.TotalSeconds > GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)
        {
            // **************************
            // Add Auto Ducks
            if( duck_scorer_red.processed_counter > 0) { auto_red_delivered = true;  }
            if (duck_scorer_blue.processed_counter > 0) { auto_blue_delivered = true; }

            // *****************************
            // Get storage of robots
            red_fully_sparked = 0;
            blue_fully_sparked = 0;
            red_part_sparked = 0;
            blue_part_sparked = 0;

            foreach ( RobotID currbot in allRobotID.Values)
            {
                if( currbot.is_red)
                {
                    // If robot is at least partially inside the storage box, count it
                    if( red_storage.IsAnyRobotInside() && (red_storage.IsRobotInside(currbot)))
                    {
                        if( red_partial_storage.IsRobotInside(currbot)) { red_part_sparked += 1; }
                        else { red_fully_sparked += 1; }
                    }
                }
                else
                {
                    // If robot is at least partially inside the storage box, count it
                    if (blue_storage.IsAnyRobotInside()  && (blue_storage.IsRobotInside(currbot)))
                    {
                        if (blue_partial_storage.IsRobotInside(currbot)) { blue_part_sparked += 1; }
                        else { blue_fully_sparked += 1; }
                    }
                }
            }

            // ******************************
            // Get warehouse of robots
            red_fully_wparked = 0;
            blue_fully_wparked = 0;
            red_part_wparked = 0;
            blue_part_wparked = 0;

            foreach (RobotID currbot in allRobotID.Values)
            {
                if (currbot.is_red)
                {
                    // If robot is at least partially inside the storage box, count it
                    if ((red_warehouse.IsAnyRobotInside()) && (red_warehouse.IsRobotInside(currbot)))
                    {
                        if (red_partial_warehouse.IsRobotInside(currbot)) { red_part_wparked += 1; }
                        else { red_fully_wparked += 1; }
                    }
                }
                else
                {
                    // If robot is at least partially inside the storage box, count it
                    if ((blue_warehouse.IsAnyRobotInside()) && (blue_warehouse.IsRobotInside(currbot)))
                    {
                        if (blue_partial_warehouse.IsRobotInside(currbot)) { blue_part_wparked += 1; }
                        else { blue_fully_wparked += 1; }
                    }
                }
            }

            // ************************************
            // Calculate Freight in Store
            red_auto_storage = GetValidFreight(red_storage, red_partial_storage);
            blue_auto_storage = GetValidFreight(blue_storage, blue_partial_storage);


            // ************************************
            // Calculate Freight in Team Hub
            red_auto_hub = GetValidFreight(red_hub_low) + GetValidFreight(red_hub_mid) + GetValidFreight(red_hub_high);
            blue_auto_hub = GetValidFreight(blue_hub_low) + GetValidFreight(blue_hub_mid) + GetValidFreight(blue_hub_high);

            // ***************************************
            // Calculate Auto Bonus
            red_auto_bonus = 0;
            blue_auto_bonus = 0;

            switch( auto_roll)
            {
                case 1:
                    red_auto_bonus = GetPreloadBonus(red_hub_low, true);
                    blue_auto_bonus = GetPreloadBonus(blue_hub_low, false);
                    break;
                case 2:
                    red_auto_bonus = GetPreloadBonus(red_hub_mid, true);
                    blue_auto_bonus = GetPreloadBonus(blue_hub_mid, false);
                    break;
                case 3:
                    red_auto_bonus = GetPreloadBonus(red_hub_high, true);
                    blue_auto_bonus = GetPreloadBonus(blue_hub_high, false);
                    break;
            }

            auto_red_score = ((auto_red_delivered) ? 10 : 0) + red_fully_sparked * 6 + red_part_sparked * 3 + red_fully_wparked * 10 + red_part_wparked * 5 + red_auto_storage * 2 + red_auto_hub * 6 + red_auto_bonus * 10; ;
            auto_blue_score = ((auto_blue_delivered) ? 10 : 0) + blue_fully_sparked * 6 + blue_part_sparked * 3 + blue_fully_wparked * 10 + blue_part_wparked * 5 + blue_auto_storage * 2 + blue_auto_hub * 6 + blue_auto_bonus * 10;
        }

        // *********************************************
        // *********************************************



        // *********************************************
        // *********************************************
        // Teleop Score
        // *********************************************
        // *********************************************

        if (time_total.TotalSeconds <= GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)
        {
            // ************************************
            // Calculate Freight in Store
            red_tele_storage = GetValidFreight(red_storage, red_partial_storage);
            blue_tele_storage = GetValidFreight(blue_storage, blue_partial_storage);

            // ************************************
            // Calculate Freight in Team Hub
            red_hub_1 = GetValidFreight(red_hub_low);
            red_hub_2 = GetValidFreight(red_hub_mid);
            red_hub_3 = GetValidFreight(red_hub_high);
            red_shared = GetValidFreight(red_hub_common);

            blue_hub_1 = GetValidFreight(blue_hub_low);
            blue_hub_2 = GetValidFreight(blue_hub_mid);
            blue_hub_3 = GetValidFreight(blue_hub_high);
            blue_shared = GetValidFreight(blue_hub_common);

            teleop_red_score = red_tele_storage + 2 * red_hub_1 + 4 * red_hub_2 + 6 * red_hub_3 + 4 * red_shared;
            teleop_blue_score = blue_tele_storage + 2 * blue_hub_1 + 4 * blue_hub_2 + 6 * blue_hub_3 + 4 * blue_shared;
        }

        // *********************************************
        // *********************************************
        // EndGame Score
        // *********************************************
        // *********************************************

        // Before end-game, reset duck_scorers
        if (time_total.TotalSeconds > GLOBALS.TIMER_ENDGAME)
        {
            if (duck_scorer_red.processed_counter > 0) { duck_scorer_red.Reset(); }
            if (duck_scorer_blue.processed_counter > 0) { duck_scorer_blue.Reset(); }
        }

        // Alliance Shipping Hub Balanced
        // Score Hub Balance
        if (!red_hub_balance.IsAnyGameElementInside() && !red_hub_balance.IsAnyRobotInside() && red_hub_robotcollision.GetRobotCount() <= 0) { red_balance = true; }
        else { red_balance = false; }
        if (!blue_hub_balance.IsAnyGameElementInside() && !blue_hub_balance.IsAnyRobotInside() && blue_hub_robotcollision.GetRobotCount() <= 0) { blue_balance = true; }
        else { blue_balance = false; }

        // Make sure no interference happened
        foreach (RobotID currbot in allRobotID.Values)
        {
            if (currbot.is_red)
            {
                // If touching oponents hub, give balance to oponents
                if ((blue_hub_balance.IsAnyRobotInside()) && (blue_hub_balance.IsRobotInside(currbot)) ||
                    (blue_hub_robotcollision.GetRobotCount() > 0) && (blue_hub_robotcollision.IsRobotInside(currbot)))
                {
                    blue_balance = true;
                }
            }
            else
            {
                // If touching oponents hub, give balance to oponents
                if ((red_hub_balance.IsAnyRobotInside()) && (red_hub_balance.IsRobotInside(currbot)) ||
                    (red_hub_robotcollision.GetRobotCount() > 0) && (red_hub_robotcollision.IsRobotInside(currbot)))
                {
                    red_balance = true;
                }
            }
        }

        // Shared Hub Algorithm

        // 
        // Score Hub Balance
        // First see if hub is touching floor without any bots touching it
        red_sbalance = false;
        blue_sbalance = false;

        if (!red_shared_balance.IsAnyRobotInside() && (shared_robotcollision.GetRobotCount() <= 0) && (red_shared_balance.GetGameElementCount() == 1))
        {
            // See if gameElement is floor
            if (red_shared_balance.game_elements[0].note == "FLOOR") { red_sbalance = true; }
        }

        if (!blue_shared_balance.IsAnyRobotInside() && (shared_robotcollision.GetRobotCount() <= 0) && (blue_shared_balance.GetGameElementCount() == 1))
        {
            // See if gameElement is floor
            if (blue_shared_balance.game_elements[0].note == "FLOOR") { blue_sbalance = true; }
        }

        // If both are true, then none get it
        if (red_sbalance && blue_sbalance)
        {
            red_sbalance = false;
            blue_sbalance = false;
        }

        // Next check for interference
        bool blue_interference = false;
        bool red_interference = false;

        // Make sure no interference happened
        foreach (RobotID currbot in allRobotID.Values)
        {
            bool interference = false;

            if ((shared_robotcollision.GetRobotCount() > 0) && shared_robotcollision.IsRobotInside(currbot)) { interference = true; }
            else if (red_shared_balance.IsAnyRobotInside() && red_shared_balance.IsRobotInside(currbot)) { interference = true; }
            else if (blue_shared_balance.IsAnyRobotInside() && blue_shared_balance.IsRobotInside(currbot)) { interference = true; }

            if (interference && currbot.is_red) { red_interference = true; }
            if (interference && !currbot.is_red) { blue_interference = true; }
        }

        // If only one side is doing interference, then give balance to other team
        if (blue_interference && !red_interference) { red_sbalance = true; }
        if (!blue_interference && red_interference) { blue_sbalance = true; }

        endgame_red_score = 0;
        endgame_blue_score = 0;

        // Score EndGame
        if (time_total.TotalSeconds <= GLOBALS.TIMER_ENDGAME)
        {

            // Add Ducks as required
            AddDucks();

            // **********************************
            // Ducks delivered
            red_ducks_delivered = duck_scorer_red.processed_counter;
            blue_ducks_delivered = duck_scorer_blue.processed_counter;

 
            // ******************************
            // Get warehouse of robots
            red_end_fully_wparked = 0;
            blue_end_fully_wparked = 0;
            red_end_part_wparked = 0;
            blue_end_part_wparked = 0;

            foreach (RobotID currbot in allRobotID.Values)
            {
                bool part_wparked = false;
                bool fully_wparked = false;

                // Check red wharehouse
                if ((red_warehouse.IsAnyRobotInside()) && (red_warehouse.IsRobotInside(currbot)))
                {
                    if (red_partial_warehouse.IsRobotInside(currbot)) { part_wparked = true; }
                    else { fully_wparked = true; }
                }

                // Check Blue wharehouse
                if ((blue_warehouse.IsAnyRobotInside()) && (blue_warehouse.IsRobotInside(currbot)))
                {
                    if (blue_partial_warehouse.IsRobotInside(currbot)) { part_wparked = true; }
                    else { fully_wparked = true; }
                }

                if( currbot.is_red )
                {
                    red_end_part_wparked += ((part_wparked) ? 1 : 0);
                    red_end_fully_wparked += ((fully_wparked) ? 1 : 0);
                }
                else
                {
                    blue_end_part_wparked += ((part_wparked) ? 1 : 0);
                    blue_end_fully_wparked += ((fully_wparked) ? 1 : 0);
                }
            }

            endgame_red_score = red_ducks_delivered * 6 + ((red_balance) ? 10 : 0) + red_end_part_wparked * 3 + red_end_fully_wparked * 6 + ((red_sbalance) ? 20 : 0);
            endgame_blue_score = blue_ducks_delivered * 6 + ((blue_balance) ? 10 : 0) + blue_end_part_wparked * 3 + blue_end_fully_wparked * 6 + ((blue_sbalance) ? 20 : 0);



        }

        // **********************************************
        // PENALTIES
        // **********************************************

        // Do possesion limit
        if (ff_settings.ENABLE_POSSESSION_LIMIT)
        {
            foreach (RobotID currbot in allRobotID.Values)
            {
                if( !currbot ) { continue;  }

                // Check if possesion detection is present
                PossessionDetect curr_detector = currbot.GetComponent<PossessionDetect>();
                if( !curr_detector) { continue; }

                // Check if limit exceeded
                if( curr_detector.GetFaultDuration() > 1f)
                {
                    if( currbot.is_red) { penalties_red += ff_settings.PENALTIES_MINOR;  }
                    else { penalties_blue += ff_settings.PENALTIES_MINOR; }

                    CreateFoul(currbot.id);
                    curr_detector.ResetFault(4f); // MAke 1s be reached in 5s
                }
    
            }
        }

        score_red = auto_red_score + teleop_red_score + endgame_red_score - penalties_red - red_resets * ff_settings.RESTART_POS_PENALTY;
        score_blue = auto_blue_score  + teleop_blue_score + endgame_blue_score - penalties_blue - blue_resets * ff_settings.RESTART_POS_PENALTY;
           

    }

    // Gets called if a robot drops and item
    public override void RobotDroppedItem(int robotid, gameElement item)
    {
        // Only address it if there is an active game on
        if( !game_running) { return; }

        // If game element isn't "in" anything, then ignore this
        if( item.note2 != "in") { return; }

        // Check to see if robot is inside either warehouse
        if( red_warehouse.IsRobotInside( allRobotID[robotid])  || blue_warehouse.IsRobotInside(allRobotID[robotid]) )
        {
            // Reset block to original 
            GameElementLeftWarehouse(item);
            return;
        }

        // Otherwise we clear the item from being reset in the future
        item.note2 = "out";
    }


    // Called when an element leaves the warehouse and it's not held by a robot
    public void GameElementLeftWarehouse( gameElement item, GenericFieldTracker warehouse = null)
    {
        // Only address it if there is an active game on
        if (!game_running) { return; }

        // If warehouse wasn't specified, get it from the item
        if( warehouse == null)
        {
            warehouse = item.tracker;
            if(warehouse == null) { return; }
        }

        // Reset position to the warehouse top
        Vector3 new_position = warehouse.transform.position;

        new_position.y = UnityEngine.Random.Range(2.5f, 4f);
        new_position.x += UnityEngine.Random.Range(-0.3f, 0.3f);
        new_position.z += UnityEngine.Random.Range(-0.5f, 0.3f);

        item.transform.position = new_position;
    }

    private int red_duck_lockout = 0;
    private int blue_duck_lockout = 0;
    private void AddDucks()
    {
        // If ducks are left, no ducks inside and carousel is stopped
        if( (red_ducks_left > 0) &&
            (carousel_red_detect.GetElementCount() == 0) &&
            (platform_red.angularVelocity.magnitude < 0.01f) &&
            (red_duck_lockout <= 0))

        {
            // Decrement ducks
            red_ducks_left -= 1;

            // Move into position
            Transform duck = ducks_red.GetChild(red_ducks_left);
            duck.SetPositionAndRotation(duck_red_pos.position, duck_red_pos.rotation);
            duck.GetComponent<DuckStates>().mystate = DuckStates.DuckPositions.Placed;

            // Lockout more ducks for 10 frames
            red_duck_lockout = 10;

        }

        // If ducks are left, no ducks inside and carousel is stopped
        if ((blue_ducks_left > 0) &&
            (carousel_blue_detect.GetElementCount() == 0) &&
            (platform_blue.angularVelocity.magnitude < 0.01f) &&
            (blue_duck_lockout <= 0))

        {
            // Decrement ducks
            blue_ducks_left -= 1;

            // Move into position
            Transform duck = ducks_blue.GetChild(blue_ducks_left);
            duck.SetPositionAndRotation(duck_blue_pos.position, duck_blue_pos.rotation);
            duck.GetComponent<DuckStates>().mystate = DuckStates.DuckPositions.Placed;

            blue_duck_lockout = 10;
        }

        // Decrement lockout counters
        if( red_duck_lockout > 0 ) { red_duck_lockout--; }
        if (blue_duck_lockout > 0) { blue_duck_lockout--; }
    }

    public override int GetRedScore()
    {
        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }

        return (int) score_red + score_redadj;
    }

    public override int GetBlueScore()
    {
        if( timerstate == TimerState.FINISHED )
        { return score_bluefinal;  }

        return (int) score_blue + score_blueadj;
    }


    private int GetValidFreight(GenericFieldTracker hub, GenericFieldTracker keepout = null)
    {
        int count = 0;
        foreach (gameElement currfreight in hub.game_elements)
        {
            // Make sure it's not in keepout region
            if( keepout)
            {
                if (keepout.IsGameElementInside(currfreight)) { continue; }
            }

            // Only score gameElements marked with "score" 
            if (currfreight.note != "score") { continue; }

            // Make sure element is not being touched by a robot
            RobotCollision robotchecker = currfreight.GetComponent<RobotCollision>();
            if (!robotchecker || (robotchecker.GetRobotCount() > 0)) { continue; }

            count += 1;
        }

        return count;
    }

    private int GetPreloadBonus(GenericFieldTracker level, bool isred)
    {
        int count = 0;
        foreach (gameElement currfreight in level.game_elements)
        {
            // Make sure it's not in keepout region

            // Only score gameElements marked with "score"
            if (currfreight.note != "score") { continue; }

            // Make sure element is not being touched by a robot
            RobotCollision robotchecker = currfreight.GetComponent<RobotCollision>();
            if (!robotchecker || (robotchecker.GetRobotCount() > 0)) { continue; }

            // See if this is an appropriate preload box
            PreloadMarker preload = currfreight.GetComponent<PreloadMarker>();
            if( !preload) { continue;  }

            // Count up if appropriate duck is inside
            if( (preload.red && isred) || (preload.blue && !isred)) { count += 1; }

        }

        return count;
    }

    private int preload_delay = 0;

    private void PreloadAll()
    {
        // Move rings to preload locations if applicable
        foreach (RobotInterface3D currbot in allrobots)
        {
            switch (currbot.myRobotID.starting_pos)
            {
                case "Red Left":
                    PreloadItem(currbot, preload_redl);
                    break;

                case "Red Right":
                    PreloadItem(currbot, preload_redr);
                    break;

                case "Blue Left":
                    PreloadItem(currbot, preload_bluel);
                    break;

                case "Blue Right":
                    PreloadItem(currbot, preload_bluer);
                    break;
            }
        }
    }



    private Vector3 delta_preload = new Vector3(0, 0, 0); // Only 1 item in preloads, thus don't need offset. Keeping this for future uses.

    // Preload the starting rings
    private void PreloadItem(RobotInterface3D bot, Transform preload_items)
    {
        // Find the preload marker
        PreloadID marker = bot.GetComponentInChildren<PreloadID>();
        if (!marker) { return; }

        // Marker found, now set rings position
        float i = 0f;
        foreach (Transform curr_preload in preload_items)
        {
            curr_preload.SetPositionAndRotation(marker.transform.position + i * delta_preload.magnitude * marker.transform.up, marker.transform.rotation * curr_preload.rotation);
            i += 1f;
        }
    }

    private int auto_roll = 1; // Which number did autonomouse roll: 1 to 3 position

    public override void OnTimerStart()
    {
        // Mark to do preloads, except we have to do it on the next update to make sure robot got reset
        preload_delay = 2;

        base.OnTimerStart();

        // Force a restart: probably not needed in this game
        Restart();

        // Get the game option

        // Roll a random number and initialize 
        System.Random rand = new System.Random();

        if (GLOBALS.game_option > 1)
        {
            auto_roll = GLOBALS.game_option - 1;
        }
        else
        {
            auto_roll = rand.Next(1, 4);
        }

        if( auto_roll > 3) { auto_roll = 3;  }

        // Move the duck position to appropriate position
        Vector3 pos = barcode_red.position;
        pos.x = -1f * ((float)(auto_roll-1)) * 0.415f;
        barcode_red.position = pos;

        pos = barcode_blue.position;
        pos.x = ((float)(auto_roll - 1)) * 0.415f;
        barcode_blue.position = pos;

    }


    // Instantiates the foul graphics
    string foul_string = "";

    // Creates the faul animatino around a robot
    private void CreateFoul(int robotid)
    {
        // Get the robot
        if (!allrobots_byid.ContainsKey(robotid)) { return; }

        RobotInterface3D robot = allrobots_byid[robotid];
        if (!robot.rb_body) { return; }

        // Only do if not in headless mode
        if (!GLOBALS.HEADLESS_MODE)
        {
            GameObject foulinstance = Instantiate(fault_prefab, robot.rb_body.transform.position, Quaternion.identity) as GameObject;

            // Set the parent to the body of the robot
            foulinstance.transform.SetParent(robot.rb_body.transform);
        }

        // If this is a server, send the request to clients
        if (GLOBALS.SERVER_MODE)
        {
            if (foul_string.Length > 0)
            {
                foul_string += ":";
            }
            foul_string += robotid;
        }
    }

    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);     

        // Send foul animation
        serverFlags["FOUL"] = foul_string;
        foul_string = "";

        serverFlags["SHUB"] = "0";
        if( red_sbalance ) { serverFlags["SHUB"] = "1";  }
        if (blue_sbalance) { serverFlags["SHUB"] = "2"; }

        serverFlags["RHUB"] = (red_balance) ? "1" : "0";
        serverFlags["BHUB"] = (blue_balance) ? "1" : "0";
    }

    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

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

        if (serverFlags.ContainsKey("SHUB"))
        {
            red_sbalance = false;
            blue_sbalance = false;

            if( serverFlags["SHUB"] == "1") { red_sbalance = true; }
            if( serverFlags["SHUB"] == "2") { blue_sbalance = true; }
        }

        if (serverFlags.ContainsKey("RHUB"))
        {
            red_balance = serverFlags["RHUB"] == "1";
        }

        if (serverFlags.ContainsKey("BHUB"))
        {
            blue_balance = serverFlags["BHUB"] == "1";
        }

    }

    void DoAnimations()
    {
        if (red_sbalance == blue_sbalance)
        {
            if (!shared_hub_pole.material.name.StartsWith(no_glow.name)) 
            {
                Destroy(shared_hub_pole.material);
                shared_hub_pole.material = no_glow; 
            }
        }
        else
        {
            if (red_sbalance)
            {
                if (!shared_hub_pole.material.name.StartsWith(red_glow.name))
                {
                    Destroy(shared_hub_pole.material);
                    shared_hub_pole.material = red_glow; 
                }

            }
            else 
            { 
                if (!shared_hub_pole.material.name.StartsWith(blue_glow.name)) 
                {
                    Destroy(shared_hub_pole.material);
                    shared_hub_pole.material = blue_glow; 
                } 
            }
        }

        if( red_balance )
        {
            if (!red_hub_pole.material.name.StartsWith(red_glow.name)) 
            {
                Destroy(red_hub_pole.material);
                red_hub_pole.material = red_glow; 
            }
        }
        else
        {
            if (!red_hub_pole.material.name.StartsWith( red_no_glow.name)) 
            {
                Destroy(red_hub_pole.material);
                red_hub_pole.material = red_no_glow; 
            }
        }

        if (blue_balance)
        {
            if (!blue_hub_pole.material.name.StartsWith( blue_glow.name)) 
            {
                Destroy(blue_hub_pole.material);
                blue_hub_pole.material = blue_glow; 
            }
        }
        else
        {
            if (!blue_hub_pole.material.name.StartsWith( blue_no_glow.name)) 
            {
                Destroy(blue_hub_pole.material);
                blue_hub_pole.material = blue_no_glow; 
            }
        }
    }
}

