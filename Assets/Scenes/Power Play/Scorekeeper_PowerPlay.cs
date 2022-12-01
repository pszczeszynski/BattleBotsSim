using System.Collections.Generic;
using UnityEngine;

public class Scorekeeper_PowerPlay : Scorekeeper {

    bool DEBUG = false;

    // Links to relevant classes/objects
    public PowerPlay_Settings pp_settings = null;
    public GameObject pushback;
    public JunctionStuff[] all_junctions;
    public Wire[] all_wires;
    public List<Wire> all_wires_sorted;

    public JunctionStuff red_start_junction;
    public JunctionStuff red_end_junction;
    public JunctionStuff blue_start_junction;
    public JunctionStuff blue_end_junction;

    public GenericFieldTracker substation_red;
    public GenericFieldTracker substation_blue;
    public GenericFieldTracker terminal_red1;
    public GenericFieldTracker terminal_red2;
    public GenericFieldTracker terminal_blue1;
    public GenericFieldTracker terminal_blue2;
    public GenericFieldTracker signal_red1;
    public GenericFieldTracker signal_red2;
    public GenericFieldTracker signal_red3;
    public GenericFieldTracker signal_keepout_red;
    public GenericFieldTracker signal_blue1;
    public GenericFieldTracker signal_blue2;
    public GenericFieldTracker signal_blue3;
    public GenericFieldTracker signal_keepout_blue;
    public Transform signal_cone_red1;
    public Transform signal_cone_red2;
    public Transform signal_cone_blue1;
    public Transform signal_cone_blue2;
    public Transform red_extra_cones;
    public Transform blue_extra_cones;
    public Transform red_extra_beacons;
    public Transform blue_extra_beacons;
    public Transform red_extra_target;
    public Transform blue_extra_target;


    // All score breakdowns
    public class ScoreBreakdown
    {
        public int score = 0;
        public int penalties = 0;
        
        public int reset_count = 0;

        public int auto_score = 0;
        public int auto_parked_count = 0;
        public int auto_terminal_count = 0;
        public int auto_ground_count = 0;
        public int auto_low_count = 0;
        public int auto_medium_count = 0;
        public int auto_high_count = 0;
        public int auto_signal_count = 0;

        // Teleop
        public int tele_score = 0;
        public int tele_terminal_count = 0;
        public int tele_ground_count = 0;
        public int tele_low_count = 0;
        public int tele_medium_count = 0;
        public int tele_high_count = 0;

        // End game
        public int end_score = 0;
        public int end_parked_count = 0;
        public int end_junctions_count = 0;
        public int end_beacon_count = 0;
        public bool end_circuit = false;

        public void ResetAuto()
        {
            auto_score = 0;
            auto_parked_count = 0;
            auto_terminal_count = 0;
            auto_ground_count = 0;
            auto_low_count = 0;
            auto_medium_count = 0;
            auto_high_count = 0;
            auto_signal_count = 0;
        }

        public void ResetTeleop()
        {
            // Teleop
            tele_score = 0;
            tele_terminal_count = 0;
            tele_ground_count = 0;
            tele_low_count = 0;
            tele_medium_count = 0;
            tele_high_count = 0;
        }

        public void ResetEnd()
        {
            // End game
            end_score = 0;
            end_parked_count = 0;
            end_junctions_count = 0;
            end_beacon_count = 0;
            end_circuit = false;
        }

        public void Reset()
        {
            score = 0;
            penalties = 0;
            reset_count = 0;

            ResetAuto();
            ResetTeleop();
            ResetEnd();
        }

        public void UpdateTotalScores(PowerPlay_Settings pp_settings)
        {
            auto_score = 2 * auto_parked_count + auto_terminal_count + 2 * auto_ground_count + 3 * auto_low_count + 4 * auto_medium_count + 5 * auto_high_count + 10 * auto_signal_count;
            tele_score = tele_terminal_count + 2 * tele_ground_count + 3 * tele_low_count + 4 * tele_medium_count + 5* tele_high_count;
            end_score = 2 * end_parked_count + 3 * end_junctions_count + 10 * end_beacon_count + ((end_circuit) ? 20 : 0);

            score = auto_score + tele_score + end_score + penalties + reset_count * pp_settings.RESTART_POS_PENALTY;
        }
    }

    ScoreBreakdown red_scores = new ScoreBreakdown();
    ScoreBreakdown blue_scores = new ScoreBreakdown();


    // My own OPR temporary placeholder
    public Dictionary<string, Dictionary<string, float>> player_detailed_opr = new Dictionary<string, Dictionary<string, float>>();

    private Vector3 old_gravity;

    private void Awake()
    {

         
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 150;
        GLOBALS.TIMER_AUTO = 30;
        GLOBALS.TIMER_ENDGAME = 30;

        // Change force of gravity to suit the FTC game
        old_gravity = Physics.gravity;
        Physics.gravity = new Vector3(0, -9.81f * 2f, 0);

        // Also change fixed update.
        // The physics engine gets real slow here due to the massive number of polygons, however don't believe ultra high speed is required since there are no fast
        // moving objects, thus slowing it down is ok.
        // Default time is 250frames/sec, lets reduce it to 120f/s
        //old_fixed_update = Time.fixedDeltaTime;
        //Time.fixedDeltaTime = 1f / 120f;

        // Get all the junctions and wires
        all_junctions = GameObject.FindObjectsOfType<JunctionStuff>();
        all_wires = GameObject.FindObjectsOfType<Wire>();

        // Sort the list (for networking sync)
        all_wires_sorted = new List<Wire>(all_wires);
        all_wires_sorted.Sort((a, b) => a.name.CompareTo(b.name));

    }

    private void OnDestroy()
    {
        // Return gravity back to original
        Physics.gravity = old_gravity;

    }


    // Rules variables

    public override void ScorerInit()
    {
        ScorerReset();

        // Get settings
        pp_settings = GameObject.Find("GameSettings").GetComponent<PowerPlay_Settings>();

        // Initialize the terminal wires
        InitTerminalWires(red_start_junction);
        InitTerminalWires(red_end_junction);
        InitTerminalWires(blue_start_junction);
        InitTerminalWires(blue_end_junction);
        return;

    }

    // Initialize the terminal wires to point to the terminal
    private void InitTerminalWires( JunctionStuff terminal)
    {
        foreach(Wire currwire in terminal.connected_wires)
        {
            currwire.node2 = terminal;
        }
    }


    public override void ScorerReset()
    {
        base.ScorerReset();

        red_scores.Reset();
        blue_scores.Reset();

        curr_red_cone = 0;
        curr_blue_cone = 0;
        curr_red_beacon = 0;
        curr_blue_beacon = 0;

        ResetCones();

    }

    // Curr red and blue extra cone
    // Note that the last 2 cones are beacons as well
    int curr_red_cone = 0;
    int curr_blue_cone = 0;
    int curr_red_beacon = 0;
    int curr_blue_beacon = 0;

    void ResetCones()
    {
        // Reset red cone states
        foreach( Transform curr_cone in red_extra_cones)
        {
            curr_cone.GetComponent<Rigidbody>().isKinematic = true;
        }

        // Reset red cone states
        foreach (Transform curr_cone in red_extra_beacons)
        {
            curr_cone.GetComponent<Rigidbody>().isKinematic = true;
        }

        // Reset blue cone states
        foreach (Transform curr_cone in blue_extra_cones)
        {
            curr_cone.GetComponent<Rigidbody>().isKinematic = true;
        }

        // Reset blue cone states
        foreach (Transform curr_cone in blue_extra_beacons)
        {
            curr_cone.GetComponent<Rigidbody>().isKinematic = true;
        }
    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        // Add this game specific data

        data["Pen_B"] = ((int)blue_scores.penalties).ToString();
        data["Pen_R"] = ((int)red_scores.penalties).ToString();
        data["Score_R"] = ((int)red_scores.score + score_redadj).ToString();
        data["Score_B"] = ((int)blue_scores.score + score_blueadj).ToString();
        data["Resets_B"] = ((int)blue_scores.reset_count).ToString();
        data["Resets_R"] = ((int)red_scores.reset_count).ToString();

        // Auto Details
        data["Auto_R"] = ((int)red_scores.auto_score).ToString();
        data["AutoParkC_R"] = ((int)red_scores.auto_parked_count).ToString();
        data["AutoTermC_R"] = ((int)red_scores.auto_terminal_count).ToString();
        data["AutoGroundC_R"] = ((int)red_scores.auto_ground_count).ToString();
        data["AutoLowC_R"] = ((int)red_scores.auto_low_count).ToString();
        data["AutoMedC_R"] = ((int)red_scores.auto_medium_count).ToString();
        data["AutoHighC_R"] = ((int)red_scores.auto_high_count).ToString();
        data["AutoSigC_R"] = ((int)red_scores.auto_signal_count).ToString();
        data["Auto_B"] = ((int)blue_scores.auto_score).ToString();
        data["AutoParkC_B"] = ((int)blue_scores.auto_parked_count).ToString();
        data["AutoTermC_B"] = ((int)blue_scores.auto_terminal_count).ToString();
        data["AutoGroundC_B"] = ((int)blue_scores.auto_ground_count).ToString();
        data["AutoLowC_B"] = ((int)blue_scores.auto_low_count).ToString();
        data["AutoMedC_B"] = ((int)blue_scores.auto_medium_count).ToString();
        data["AutoHighC_B"] = ((int)blue_scores.auto_high_count).ToString();
        data["AutoSigC_B"] = ((int)blue_scores.auto_signal_count).ToString();

        // Teleop
        data["Tele_R"] = ((int)red_scores.tele_score).ToString();
        data["TeleTermC_R"] = ((int)red_scores.tele_terminal_count).ToString();
        data["TeleGroundC_R"] = ((int)red_scores.tele_ground_count).ToString();
        data["TeleLowC_R"] = ((int)red_scores.tele_low_count).ToString();
        data["TeleMedC_R"] = ((int)red_scores.tele_medium_count).ToString();
        data["TeleHighC_R"] = ((int)red_scores.tele_high_count).ToString();
        data["Tele_B"] = ((int)blue_scores.tele_score).ToString();
        data["TeleTermC_B"] = ((int)blue_scores.tele_terminal_count).ToString();
        data["TeleGroundC_B"] = ((int)blue_scores.tele_ground_count).ToString();
        data["TeleLowC_B"] = ((int)blue_scores.tele_low_count).ToString();
        data["TeleMedC_B"] = ((int)blue_scores.tele_medium_count).ToString();
        data["TeleHighC_B"] = ((int)blue_scores.tele_high_count).ToString();

        //Endgame
        data["End_R"] = ((int)red_scores.end_score).ToString();
        data["EndParkC_R"] = ((int)red_scores.end_parked_count).ToString();
        data["EndJunctionsC_R"] = ((int)red_scores.end_junctions_count).ToString();
        data["EndBeaconC_R"] = ((int)red_scores.end_beacon_count).ToString();
        data["EndCircuit_R"] = (red_scores.end_circuit) ? "Y" : "N";
        data["End_B"] = ((int)blue_scores.end_score).ToString();
        data["EndParkC_B"] = ((int)blue_scores.end_parked_count).ToString();
        data["EndJunctionsC_B"] = ((int)blue_scores.end_junctions_count).ToString();
        data["EndBeaconC_B"] = ((int)blue_scores.end_beacon_count).ToString();
        data["EndCircuit_B"] = (blue_scores.end_circuit) ? "Y" : "N";

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

        string team = (red) ? "_R" : "_B";

        // Now output the string
        return 
            "<B>AUTO Score   = </B> " + details["Auto" + team] + "\n" +
            "    # Bots Parked: " + details["AutoParkC" + team] + "\n" +
            "    # Cones in junctions \n" +
            "        terminal: " + details["AutoTermC" + team] + "\n" +
            "        ground  : " + details["AutoGroundC" + team] + "\n" +
            "        low     : " + details["AutoLowC" + team] + "\n" +
            "        medium  : " + details["AutoMedC" + team] + "\n" +
            "        high    : " + details["AutoHighC" + team] + "\n" +
            "    # Signals: " + details["AutoSigC" + team] + "\n" +
            "\n" +
            "<B>TELEOP Score =</B> " + details["Tele" + team] + "\n" +
            "    # Cones in junctions \n" +
            "        terminal: " + details["TeleTermC" + team] + "\n" +
            "        ground  : " + details["TeleGroundC" + team] + "\n" +
            "        low     : " + details["TeleLowC" + team] + "\n" +
            "        medium  : " + details["TeleMedC" + team] + "\n" +
            "        high    : " + details["TeleHighC" + team] + "\n" +
            "\n" +
            "<B>ENDGAME Score=</B> " + details["End" + team] + "\n" +
            "    # Bots Parked: " + details["EndParkC" + team] + "\n" +
            "    # Junctions: " + details["EndJunctionsC" + team] + "\n" +
            "    # Beacons: " + details["EndBeaconC" + team] + "\n" +
            "    Circuit Complete? : " + details["EndCircuit" + team] + "\n" +
            "\n" +
            "<B>Resets = </B> " + details["Resets" + team] + "\n" +
            "<B>PENALTIES = </B> " + details["Pen" + team];

    }


    // Clears the END GMAE OPR value for everyone
    private void OPR_clearitem(string item)
    {
        foreach (Dictionary<string, float> curropr in player_detailed_opr.Values)
        {
            curropr[item] = 0;
        }
    }

    public override void Restart()
    {
        base.Restart();

        // Nothing to do in Client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        pushback.SetActive(false);
    }



    // Gets run at the beggining of a match
    // This one is guarantted to run after all player positions have been reset to be the starting position, thus is we're placing objects inside the
    // robots, it needs to happen here.

    private int auto_roll = 1; // Which number did autonomouse roll: 1 to 3 position
                              
    // Delay preload to guarantee robot is instantiated correctly
    private int preload_delay = 0;
    
    public override void OnTimerStart()
    {
        // Mark to do preloads, except we have to do it on the next update to make sure robot got reset
        preload_delay = 2;

        base.OnTimerStart();

        // Force a restart: we need to have all rings divided up in the ringsheld lists
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

        if (auto_roll > 3) { auto_roll = 3; }

        // Rotate the signals to appropriate oritentation
        signal_cone_red1.Rotate(0f, 120f * ((float)auto_roll - 1f), 0f);
        signal_cone_red2.Rotate(0f, 120f * ((float)auto_roll - 1f), 0f);
        signal_cone_blue1.Rotate(0f, 120f * ((float)auto_roll - 1f), 0f);
        signal_cone_blue2.Rotate(0f, 120f * ((float)auto_roll - 1f), 0f);

    }


    private bool game_running = false;

    public override void ScorerUpdate(bool last_frame = false)
    {
        // Update score
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
                LoadAllCones();
            }
        }


        CalculateScores();
    }




    // Function triggered when new robots show up on the field or are updated
    public override void FieldChangedTrigger()
    {
      
    }

    // A player requested a reset or a Referee requested a reset, thus add penalties to opponents score
    public override void PlayerReset(int id, bool referee = false)
    {
        base.PlayerReset(id);

        if (!game_running) { return; }

        if (allRobotID.ContainsKey(id))
        {
            if (!referee)
            {
                if (allRobotID[id].is_red) { blue_scores.reset_count += 1; }
                else { red_scores.reset_count += 1; }
            }
            else
            {
                if (allRobotID[id].is_red) { blue_scores.penalties += pp_settings.REF_RESET_PENALTY; }
                else { red_scores.penalties += pp_settings.REF_RESET_PENALTY; }
            }
        }
    }

    private void LoadAllCones()
    {
        // Move rings to preload locations if applicable
        foreach (RobotInterface3D currbot in allrobots)
        {
            switch (currbot.myRobotID.starting_pos)
            {
                case "Red Left":
                case "Red Right":
                    if( LoadStartingCone(currbot, red_extra_cones, curr_red_cone) )
                    {
                        curr_red_cone++;
                    }
                    break;

                case "Blue Left":
                case "Blue Right":
                    if (LoadStartingCone(currbot, blue_extra_cones, curr_blue_cone))
                    {
                        curr_blue_cone++;
                    }
                    break;

            }
        }
    }

    // Preload the starting cones
    private bool LoadStartingCone(RobotInterface3D bot, Transform cones, int cone_id = 0)
    {
        if (!bot) { return false;  }

        // Get the preload markers
        // There may be more than one for different games

        PreloadID[] markers = bot.GetComponentsInChildren<PreloadID>();
        if (markers.Length < 1) { return false; }

        PreloadID marker = null;

        // Find the Power Play marker (only 1 should exist)
        foreach( PreloadID currmarker in markers)
        {
            if( currmarker.value.ToLower() == "power play")
            {
                marker = currmarker;
                break;
            }
        }

        // If none found, return
        if( !marker ) { return false; }

        // Move the cone
        Transform preload = cones.GetChild(cone_id);
        preload.SetPositionAndRotation(marker.transform.position, marker.transform.rotation);

        // Make it non kinematic
        preload.GetComponent<Rigidbody>().isKinematic = false;
        return true;
    }

    private void AddToPlayerOPR(Dictionary<int, int> indata, float points, bool is_red)
    {
        foreach (int id in indata.Keys)
        {
            // Validate robot exists (what if they disconnected and this is some phantom data?)
            if (!allrobots_byid.ContainsKey(id) || (allrobots_byid[id] == null) || (allrobots_byid[id].myRobotID == null))
            {
                DoFieldChanged();
            }

            if (!allrobots_byid.ContainsKey(id) || (allrobots_byid[id] == null) || (allrobots_byid[id].myRobotID == null))
            {
                continue;
            }

            // If this came from an enemy robot, make it score negative.
            float negate_score = 1f;

            if (allrobots_byid[id].myRobotID.is_red != is_red) { negate_score = -1f; }

            // Add the score to our robot
            if (player_opr.ContainsKey(GLOBALS.client_names[id])) { player_opr[GLOBALS.client_names[id]] += negate_score * points * indata[id]; }
            else { player_opr[GLOBALS.client_names[id]] = negate_score * points * indata[id]; }
        }
    }

  

    private void AddToPlayerOPR(RobotID robot, float points)
    {
        // Validate robot exists (what if they disconnected and this is some phantom data?)
        if (!allrobots_byid.ContainsKey(robot.id) )
        {
            DoFieldChanged();
        }

        string robot_name = GLOBALS.client_names[robot.id];

        // Add the score to our robot
        if (player_opr.ContainsKey(robot_name)) { player_opr[robot_name] += points; }
        else { player_opr[robot_name] = points; }
    }

    private void SubFromPlayerOPR(Dictionary<int, int> indata, float score)
    {
        foreach (int id in indata.Keys)
        {
            // Validate robot exists (what if they disconnected and this is some phantom data?)
            if (!allrobots_byid.ContainsKey(id) || (allrobots_byid[id] == null) || (allrobots_byid[id].myRobotID == null))
            {
                DoFieldChanged();
            }

            if (!allrobots_byid.ContainsKey(id) || (allrobots_byid[id] == null) || (allrobots_byid[id].myRobotID == null))
            {
                continue;
            }

            // Subtract the score from our robot
            if (player_opr.ContainsKey(GLOBALS.client_names[id])) { player_opr[GLOBALS.client_names[id]] -= score * indata[id]; }
            else { player_opr[GLOBALS.client_names[id]] = -1f * score * indata[id]; }
        }
    }

   

    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if (!DEBUG && (timerstate != TimerState.RUNNING))
        { return; }

        // Make sure settings have loaded
        if (!pp_settings) { return; }

        // *********************************************
        // *********************************************
        // Score Auto: don't score anything other than auto during auto
        // *********************************************
        // *********************************************

        if (time_total.TotalSeconds > GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)
        {
            red_scores.ResetAuto();
            blue_scores.ResetAuto();

            // **************************
            // Parking
            red_scores.auto_parked_count = terminal_red1.GetRedRobotCount() + substation_red.GetRedRobotCount();
            blue_scores.auto_parked_count = terminal_blue1.GetBlueRobotCount() + substation_blue.GetBlueRobotCount();


            // **************************
            // Terminal
            red_scores.auto_terminal_count = terminal_red1.GetGameElementCount(ElementType.Red1) + terminal_red2.GetGameElementCount(ElementType.Red1);
            blue_scores.auto_terminal_count = terminal_blue1.GetGameElementCount(ElementType.Blue1) + terminal_blue2.GetGameElementCount(ElementType.Blue1);

            // **************************
            // Junctions
            foreach (JunctionStuff junction in all_junctions)
            {
                int redcones = junction.GetRedCount();
                int bluecones = junction.GetBlueCount();

                switch (junction.myType)
                {
                    case JunctionStuff.JunctionType.ground:
                        red_scores.auto_ground_count += redcones;
                        blue_scores.auto_ground_count += bluecones;
                        break;

                    case JunctionStuff.JunctionType.low:
                        red_scores.auto_low_count += redcones;
                        blue_scores.auto_low_count += bluecones;
                        break;

                    case JunctionStuff.JunctionType.medium:
                        red_scores.auto_medium_count += redcones;
                        blue_scores.auto_medium_count += bluecones;
                        break;

                    case JunctionStuff.JunctionType.high:
                        red_scores.auto_high_count += redcones;
                        blue_scores.auto_high_count += bluecones;
                        break;
                }

            }

            // **************************
            // Signals
            GenericFieldTracker signal_red = (auto_roll == 1) ? signal_red1 : (auto_roll == 2) ? signal_red2 : signal_red3;
            GenericFieldTracker signal_blue = (auto_roll == 1) ? signal_blue1 : (auto_roll == 2) ? signal_blue2 : signal_blue3;

            foreach( RobotID currbot in signal_red.robots)
            {
                if( signal_keepout_red.IsRobotInside(currbot)) { continue; }
                red_scores.auto_signal_count += 1;
            }

            foreach (RobotID currbot in signal_blue.robots)
            {
                if (signal_keepout_blue.IsRobotInside(currbot)) { continue; }
                blue_scores.auto_signal_count += 1;
            }
        }

        // *********************************************
        // *********************************************
        // Score Teleop
        // *********************************************
        // *********************************************

        else
        {
            red_scores.ResetTeleop();
            blue_scores.ResetTeleop();


            // **************************
            // Terminal
            red_scores.tele_terminal_count = terminal_red1.GetGameElementCount(ElementType.Red1) + terminal_red2.GetGameElementCount(ElementType.Red1);
            blue_scores.tele_terminal_count = terminal_blue1.GetGameElementCount(ElementType.Blue1) + terminal_blue2.GetGameElementCount(ElementType.Blue1);

            // **************************
            // Junctions
            foreach (JunctionStuff junction in all_junctions)
            {
                int redcones = junction.GetRedCount();
                int bluecones = junction.GetBlueCount();

                switch (junction.myType)
                {
                    case JunctionStuff.JunctionType.ground:
                        red_scores.tele_ground_count += redcones;
                        blue_scores.tele_ground_count += bluecones;
                        break;

                    case JunctionStuff.JunctionType.low:
                        red_scores.tele_low_count += redcones;
                        blue_scores.tele_low_count += bluecones;
                        break;

                    case JunctionStuff.JunctionType.medium:
                        red_scores.tele_medium_count += redcones;
                        blue_scores.tele_medium_count += bluecones;
                        break;

                    case JunctionStuff.JunctionType.high:
                        red_scores.tele_high_count += redcones;
                        blue_scores.tele_high_count += bluecones;
                        break;
                }

            }
        }

        // *********************************************
        // *********************************************
        // Score EndGame
        // *********************************************
        // *********************************************
        
        // Clear all wire circuits
        ClearCircuits();

        if (DEBUG || (time_total.TotalSeconds <= GLOBALS.TIMER_ENDGAME))
        {
            red_scores.ResetEnd();
            blue_scores.ResetEnd();

            // Parking in Terminal
            red_scores.end_parked_count = terminal_red1.GetRedRobotCount() + terminal_red2.GetRedRobotCount();
            blue_scores.end_parked_count = terminal_blue1.GetBlueRobotCount() + terminal_blue2.GetBlueRobotCount();

            // Junctions Ownership
            foreach (JunctionStuff junction in all_junctions)
            {
                // Skip if its a terminal junction (not real, only used here to find the circuit)
                if( junction.myType == JunctionStuff.JunctionType.terminal) { continue; }

                if (junction.red_capped) { red_scores.end_beacon_count += 1; }
                else if (junction.red_owned) { red_scores.end_junctions_count += 1; }

                if (junction.blue_capped) { blue_scores.end_beacon_count += 1; }
                else if (junction.blue_owned) { blue_scores.end_junctions_count += 1; }


            }

            // Completed circuit check
            if ((terminal_red1.GetGameElementCount(ElementType.Red1) > 0) && (terminal_red2.GetGameElementCount(ElementType.Red1)>0) )
            {
                red_scores.end_circuit = CheckCircuit(red_start_junction, red_end_junction);

                // Find shortest path
                if (red_scores.end_circuit) { FindShortestCircuit(red_end_junction); }
            }
            else
            {
                red_scores.end_circuit = false;
            }

            // Completed circuit check
            if ((terminal_blue1.GetGameElementCount(ElementType.Blue1) > 0) && (terminal_blue2.GetGameElementCount(ElementType.Blue1) > 0))
            {
                blue_scores.end_circuit = CheckCircuit(blue_start_junction, blue_end_junction);

                // Find shortest path
                if (blue_scores.end_circuit) { FindShortestCircuit(blue_end_junction); }
            }
            else
            {
                blue_scores.end_circuit = false;
            }
        }

        // Calculate final score in the breakdowns
        red_scores.UpdateTotalScores(pp_settings);
        blue_scores.UpdateTotalScores(pp_settings);

        // Add cones
        if(time_total.TotalSeconds < GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)
        {
            AddCones(time_total.TotalSeconds <= GLOBALS.TIMER_ENDGAME);
        }
    }

    void ClearCircuits()
    {
        // Reset all weight factors
        foreach (Wire curr in all_wires)
        {
            curr.partofcircuit = false;
            curr.weight = 9999;
        }
    }

    // Light up the shortest circuit
    void FindShortestCircuit( JunctionStuff end)
    {
        foreach ( Wire currwire in end.connected_wires)
        {
            // See if it's the next node
            if ( currwire.weight < end.weight)
            {
                currwire.partofcircuit = true;

                if( currwire.node1 != end )
                {
                    FindShortestCircuit(currwire.node1);
                }
                else
                {
                    FindShortestCircuit(currwire.node2);
                }
            }
        }
    }

    private bool CheckCircuit(JunctionStuff start, JunctionStuff end )
    {
        // Reset all weight factors
        foreach( JunctionStuff curr in all_junctions)
        {
            curr.weight = 9999;
        }

        start.weight = 1;

        // Find the path recursively
        RecursiveCircuitCheck(start, end);

        // If we found a path, return true
        if( end.weight < 9999) { return true; }

        return false;
    }

    void RecursiveCircuitCheck(JunctionStuff start, JunctionStuff end)
    {
        // Go through each start wire
        foreach (Wire currwire in start.connected_wires)
        {

            // Get next Junction
            JunctionStuff next = currwire.node1;
            if (next == start)
            { next = currwire.node2; }

            // If it's not owned by our team, exit
            if (!(next.red_owned && start.red_owned || next.blue_owned && currwire.blue_owned))
            {
                continue;
            }

            // If this junction has a higher weight, recurse into it
            if (next.weight > start.weight + 1)
            {
                next.weight = start.weight + 1;
                currwire.weight = start.weight;

                // If this is the end node, see if there is a shorter path
                if (next == end) { continue; }

                RecursiveCircuitCheck(next, end);
            }
        }
    }

    private int red_cone_lockout = 0;
    private int blue_cone_lockout = 0;
    void AddCones(bool endgame = false)
    {
        //*** RED ****

        // If this is end-game, prioritize beacons
        if (endgame && (curr_red_beacon < red_extra_beacons.childCount))
        {
            // Add cones if space is empty
            if ((red_cone_lockout <= 0) && !substation_red.IsAnyRobotInside() && !substation_red.IsAnyGameElementInside())
            {
                // Get next cone
                Transform new_cone = red_extra_beacons.GetChild(curr_red_beacon++);

                // Move cone to new position
                new_cone.SetPositionAndRotation(red_extra_target.position, red_extra_target.rotation);

                // Enable physics
                new_cone.GetComponent<Rigidbody>().isKinematic = false;

                // Lockout more cones for 10 frames
                red_cone_lockout = 10;
            }
        }
        else
        {

            // Add cones if space is empty
            if ((red_cone_lockout <= 0) && (curr_red_cone < red_extra_cones.childCount) && !substation_red.IsAnyRobotInside() && !substation_red.IsAnyGameElementInside())
            {
                // Get next cone
                Transform new_cone = red_extra_cones.GetChild(curr_red_cone++);

                // Move cone to new position
                new_cone.SetPositionAndRotation(red_extra_target.position, red_extra_target.rotation);

                // Enable physics
                new_cone.GetComponent<Rigidbody>().isKinematic = false;

                // Lockout more cones for 10 frames
                red_cone_lockout = 10;
            }
        }

        //*** BLUE ****
        // If this is end-game, prioritize beacons
        if (endgame && (curr_blue_beacon < blue_extra_beacons.childCount))
        {            
            // Add cones if space is empty
            if ((blue_cone_lockout <= 0) && !substation_blue.IsAnyRobotInside() && !substation_blue.IsAnyGameElementInside())
            {
                // Get next cone
                Transform new_cone = blue_extra_beacons.GetChild(curr_blue_beacon++);

                // Move cone to new position
                new_cone.SetPositionAndRotation(blue_extra_target.position, blue_extra_target.rotation);

                // Enable physics
                new_cone.GetComponent<Rigidbody>().isKinematic = false;

                // Lockout more cones for 10 frames
                blue_cone_lockout = 10;
            }
        }
        else
        {
            // Add cones if space is empty
            if ((blue_cone_lockout <= 0) && (curr_blue_cone < blue_extra_cones.childCount) && !substation_blue.IsAnyRobotInside() && !substation_blue.IsAnyGameElementInside())
            {
                // Get next cone
                Transform new_cone = blue_extra_cones.GetChild(curr_blue_cone++);

                // Move cone to new position
                new_cone.SetPositionAndRotation(blue_extra_target.position, blue_extra_target.rotation);

                // Enable physics
                new_cone.GetComponent<Rigidbody>().isKinematic = false;

                // Lockout more cones for 10 frames
                blue_cone_lockout = 10;
            }
        }

        // Decrement lockout counters
        if (red_cone_lockout > 0) { red_cone_lockout--; }
        if (blue_cone_lockout > 0) { blue_cone_lockout--; }
    }

    public override void DoTimerFinished()
    {
        // Make sure to execute base
        base.DoTimerFinished();

        // Add in the endgame stuff
        foreach( string currid in player_detailed_opr.Keys)
        {
            if( player_opr.ContainsKey(currid))
            {
                player_opr[currid] += player_detailed_opr[currid]["END"];
            }
            else
            {
                player_opr[currid] = player_detailed_opr[currid]["END"];
            }
        }
    }

    public override int GetRedScore()
    {
        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }
      

        return red_scores.score + score_redadj;
    }

    public override int GetBlueScore()
    {
        if( timerstate == TimerState.FINISHED )
        { return score_bluefinal;  }

     
        return blue_scores.score + score_blueadj;
    }



    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

        // Add Wire States
        serverFlags["PP_WIRES"] = "";

        string outcodes = "";

        // Add all wires
        for( int i =0; i < all_wires_sorted.Count; i++)
        {
            outcodes += all_wires_sorted[i].GetState(pp_settings.ENABLE_WIRES);
        }

        serverFlags["PP_WIRES"] = outcodes;
  

    }

    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

        // Get our wire states

        string wires = serverFlags["PP_WIRES"];

        for( int i =0; (i < wires.Length) && (i < all_wires_sorted.Count); i++)
        {
            all_wires_sorted[i].SetState(wires[i]);
        }

    }
}