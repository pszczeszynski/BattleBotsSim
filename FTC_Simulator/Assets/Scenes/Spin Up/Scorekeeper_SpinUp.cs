using System.Collections.Generic;
using UnityEngine;

public class Scorekeeper_SpinUp : Scorekeeper {
    // DEBUG
    public bool DEBUG = false;
    public float normal_timescale = 1f;

    // Game specific settings
    SpinUp_Settings su_settings = null;

    // Game objects
    public List<RollerDetect> rollers = new List<RollerDetect>();  // Rollers
    public GenericFieldTracker red_low_goal;
    public GenericFieldTracker red_low_keepout;
    public GenericFieldTracker red_high_goal;
    public GenericFieldTracker blue_low_goal;
    public GenericFieldTracker blue_low_keepout;
    public GenericFieldTracker blue_high_goal;
    public Transform tiles_parent;
    private GenericFieldTracker[] all_tiles;
    public GameObject pushback;

    public Transform extra_rings_red;
    public Transform extra_rings_blue;
    public Transform ringreturn_red;
    public Transform ringreturn_blue;
    public FRC_DepotHandler detect_red;
    public FRC_DepotHandler detect_blue;
    public GenericFieldTracker redrampdetector;
    public GenericFieldTracker bluerampdetector;

    // Total scores
    public double score_red = 0;
    public double score_blue = 0;

    // auto
    public double score_auto_red = 0;
    public double score_auto_blue = 0;
    public bool blue_winpoint = false;
    public bool red_winpoint = false;
    public float blue_auto_bonus = 0;
    public float red_auto_bonus = 0;
    public int discs_high_red_auto = 0;
    public int discs_low_red_auto = 0;
    public int discs_high_blue_auto = 0;
    public int discs_low_blue_auto = 0;
    public int rollers_red_auto = 0;
    public int rollers_blue_auto = 0;

    // Teleop
    public int discs_high_red=0;
    public int discs_low_red=0;
    public int rollers_red = 0;
    public int rollers_blue = 0;
    public int tiles_red = 0;
    public int tiles_blue = 0;
    public int discs_high_blue=0;
    public int discs_low_blue=0;




    // My own OPR temporary placeholder
    public Dictionary<string, Dictionary<string, float>> player_detailed_opr = new Dictionary<string, Dictionary<string, float>>();
    // public Dictionary<string, float> red_detailed_score = new Dictionary<string, float>();
    // public Dictionary<string, float> blue_detailed_score = new Dictionary<string, float>();


    private Vector3 old_gravity;

    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 15;
        GLOBALS.TIMER_ENDGAME = 0; // No endgame in this VEX game

        // Change force of gravity to suit the FTC/Vex game
        old_gravity = Physics.gravity;
        Physics.gravity = new Vector3(0, -9.81f * 2f, 0);

        // Get all tiles
        all_tiles = tiles_parent.GetComponentsInChildren<GenericFieldTracker>();
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
        su_settings = GameObject.Find("GameSettings").GetComponent<SpinUp_Settings>();

        return;

    }


    public override void ScorerReset()
    {
        base.ScorerReset();


        // Total scores
        score_red = 0;
        score_blue = 0;

        // auto
        score_auto_red = 0;
        score_auto_blue = 0;
        blue_winpoint = false;
        red_winpoint = false;
        blue_auto_bonus = 0;
        red_auto_bonus = 0;
        discs_high_red_auto = 0;
        discs_low_red_auto = 0;
        discs_high_blue_auto = 0;
        discs_low_blue_auto = 0;
        rollers_red_auto = 0;
        rollers_blue_auto = 0;

        // Teleop
        discs_high_red = 0;
        discs_low_red = 0;
        rollers_red = 0;
        rollers_blue = 0;
        tiles_red = 0;
        tiles_blue = 0;
        discs_high_blue = 0;
        discs_low_blue = 0;


    }

    private void ResetDetails()
    {
        /*
        red_detailed_score.Clear();
        blue_detailed_score.Clear();

        red_detailed_score["DISCS_H"] = 0;
        blue_detailed_score["DISCS_H"] = 0;
        red_detailed_score["DISCS_L"] = 0;
        blue_detailed_score["DISCS_L"] = 0;
        red_detailed_score["ROLLERS"] = 0;
        blue_detailed_score["ROLLERS"] = 0;

        red_detailed_score["TILES"] = 0;  
        blue_detailed_score["TILES"] = 0; 

        red_detailed_score["AUTO"] = 0;
        blue_detailed_score["AUTO"] = 0;
        red_detailed_score["AUTO_WP"] = 0;
        blue_detailed_score["AUTO_WP"] = 0;
        red_detailed_score["AUTO_BONUS"] = 0;
        blue_detailed_score["AUTO_BONUS"] = 0;
        red_detailed_score["AUTO_DISCS_H"] = 0;
        blue_detailed_score["AUTO_DISCS_H"] = 0;
        red_detailed_score["AUTO_DISCS_L"] = 0;
        blue_detailed_score["AUTO_DISCS_L"] = 0;
        red_detailed_score["AUTO_ROLLERS"] = 0;
        blue_detailed_score["AUTO_ROLLERS"] = 0;
        */
    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        // Add this game specific data

        // Total Scores
        data["R_Score"] = ((int)score_red + score_redadj).ToString();
        data["B_Score"] = ((int)score_blue + score_blueadj).ToString();

        // Auto Details
        data["R_S_Auto"] = ((int)score_auto_red).ToString();
        data["B_S_Auto"] = ((int)score_auto_blue).ToString();
        data["R_WP"] = (red_winpoint) ? "Y" : "N";
        data["B_WP"] = (blue_winpoint) ? "Y" : "N";
        data["R_S_Bonus"] = ((int)(10f * red_auto_bonus)).ToString();
        data["B_S_Bonus"] = ((int)(10f * blue_auto_bonus)).ToString();
        data["R_C_AUTO_DISCS_H"] = discs_high_red_auto.ToString();
        data["R_C_AUTO_DISCS_L"] = discs_low_red_auto.ToString();
        data["B_C_AUTO_DISCS_H"] = discs_high_blue_auto.ToString();
        data["B_C_AUTO_DISCS_L"] = discs_low_blue_auto.ToString();
        data["R_C_AUTO_ROLLERS"] = rollers_red_auto.ToString();
        data["B_C_AUTO_ROLLERS"] = rollers_blue_auto.ToString();

        // Teleop
        data["R_C_DISCS_H"] = discs_high_red.ToString();
        data["R_C_DISCS_L"] = discs_low_red.ToString();
        data["B_C_DISCS_H"] = discs_high_blue.ToString();
        data["B_C_DISCS_L"] = discs_low_blue.ToString();
        data["R_C_ROLLERS"] = rollers_red.ToString();
        data["B_C_ROLLERS"] = rollers_blue.ToString();
        data["R_C_TILES"] = tiles_red.ToString();
        data["B_C_TILES"] = tiles_blue.ToString();
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
        string outstring = "";

        if (!GLOBALS.SINGLEPLAYER_MODE)
        {
            outstring = "<B>AUTO Score   = </B> " + details[team + "_S_Auto"] + "\n" +
            "    # High Goal: " + details[team + "_C_AUTO_DISCS_H"] + "\n" +
            "    # Low Goal: " + details[team + "_C_AUTO_DISCS_L"] + "\n" +
            "    # Rollers: " + details[team + "_C_AUTO_ROLLERS"] + "\n" +
            "<B>AUTO Win Point = </B> " + details[team + "_WP"] + "\n" +
            "\n";
        }

        return outstring +          
        "<B>Score =</B> " + details[team + "_Score"] + "\n" +
        ((!GLOBALS.SINGLEPLAYER_MODE) ? "    Auto Bonus: " + details[team + "_S_Bonus"] + "\n" : "") +
        "    # High Goal: " + details[team + "_C_DISCS_H"] + "\n" +
        "    # Low Goal: " + details[team + "_C_DISCS_L"] + "\n" +
        "    # Rollers: " + details[team + "_C_ROLLERS"] + "\n" +
        "    # Tiles: " + details[team + "_C_TILES"] + "\n" +
        ((!GLOBALS.SINGLEPLAYER_MODE) ?  "    Ref adjustments: " + ((red) ? details["RedADJ"] : details["BlueADJ"]) + "\n" : "");
    }


    // Clears the END GMAE OPR value for everyone
    private void OPR_clearitem(string item)
    {
        foreach (Dictionary<string, float> curropr in player_detailed_opr.Values)
        {
            curropr[item] = 0;
        }
    }

    private bool skills_game = false;

    public override void Restart()
    {
        base.Restart();

        // Nothing to do in Client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        // If this game mode is option 2 in single player, then change timer to 1m
        if (GLOBALS.SINGLEPLAYER_MODE && GLOBALS.topsingleplayer && (GLOBALS.game_option == 2))
        {
            GLOBALS.TIMER_TOTAL = 60;
            GLOBALS.TIMER_AUTO = 0;
            skills_game = true;
        }
        else
        {
            GLOBALS.TIMER_TOTAL = 120;
            GLOBALS.TIMER_AUTO = 15;
            skills_game = false;
        }

    }

    // Load rings alogirthms
    private int loadrings_delay = 0;
    private Vector3 delta_ring = new Vector3(0, 0.048f, 0);

    private void LoadAllRings()
    {
        // Move rings to preload locations if applicable
        foreach (RobotInterface3D currbot in allrobots)
        {
            switch (currbot.myRobotID.starting_pos)
            {
                case "Red Left":
                case "Red Right":
                    LoadStartingRings(currbot, extra_rings_red);

                    // If this is skills, then remove extra rings
                    if( skills_game)
                    {
                        RemoveExtraRings(extra_rings_red, 2);
                        RemoveExtraRings(extra_rings_blue, 4);
                    }
                    break;

                case "Blue Left":
                case "Blue Right":
                    LoadStartingRings(currbot, extra_rings_blue);

                    if (skills_game)
                    {
                        RemoveExtraRings(extra_rings_blue, 2);
                        RemoveExtraRings(extra_rings_red, 4);
                    }
                    break;

            }
        }
    }

    // Preload the starting rings
    // If bot = null, then rings will be enabled but not transformed
    private void LoadStartingRings(RobotInterface3D bot, Transform rings, int count = 2)
    {
        // Find the preload marker
        PreloadID marker = null;
        if (bot)
        {
            marker = bot.GetComponentInChildren<PreloadID>();
            if (!marker) { return; }
        }

        // Marker found, now set rings position
        float i = 0f;
        foreach (Transform curr_ring in rings)
        {
            // If we loaded 2 rings already, then leave
            if( i>= count)
            {
                break;
            }

            // If this ring is not kinematic, then it means its not used (need object to be active to get it's gamedata, etc..
            if( !curr_ring.GetComponent<Rigidbody>().isKinematic) { continue; }

            // Set discs position
            if (marker)
            {
                curr_ring.SetPositionAndRotation(marker.transform.position + (i + 1f) * delta_ring.magnitude * marker.transform.up, marker.transform.rotation * curr_ring.rotation);                
            }

            i += 1f;

            // Make it active
            curr_ring.GetComponent<Rigidbody>().isKinematic = false;
        }
    }

    private void RemoveExtraRings(Transform rings, int count)
    {
        LoadStartingRings(null, rings, count);
    }


    // Gets run at the beggining of a match
    // This one is guarantted to run after all player positions have been reset to be the starting position, thus is we're placing objects inside the
    // robots, it needs to happen here.
    public override void OnTimerStart()
    {
        // Force a restart: we need to have all rings divided up in the ringsheld lists
        Restart();


        // Mark to do load rings, except we have to do it on the next update to make sure robot got reset
        loadrings_delay = 2;

        // Initialize Triggers to be visible
        detect_red.GetComponent<MeshRenderer>().enabled = true;
        detect_blue.GetComponent<MeshRenderer>().enabled = true;
        red_rings_exhausted = false;
        blue_rings_exhausted = false;


        // Reset extra rings to be kinematic
        foreach (Transform curr_ring in extra_rings_red)
        {
            Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
            if (rb) { rb.isKinematic = true; }
        }
        foreach (Transform curr_ring in extra_rings_blue)
        {
            Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
            if (rb) { rb.isKinematic = true; }
        }

        // Clear triggers just in case
        detect_red.Clear();
        detect_blue.Clear();
        redrampdetector.Reset();
        bluerampdetector.Reset();
    }



    public override void ScorerUpdate(bool last_frame = false)
    {
        // Update score
        if (GLOBALS.CLIENT_MODE) { return; }
        
        // ******* DEBUG
        if (DEBUG) { Time.timeScale = normal_timescale; }

        bool game_running = true;
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

        CalculateScores();

        // Load all rings
        if (loadrings_delay > 0)
        {
            loadrings_delay -= 1;

            if (loadrings_delay == 0)
            {
                LoadAllRings();
            }
        }
    }


    // Function triggered when new robots show up on the field or are updated
    private List<RobotStates> robotstates = new List<RobotStates>();
    public override void FieldChangedTrigger()
    {
        List<RobotStates> newstates = new List<RobotStates>();

        // Add a new robot state for each robot on the field
        for (int index = 0; index < allrobots.Count; index++)
        {
            int robotstateindex = GetRobotStateIndex(allrobots[index]);
            if (robotstateindex < 0)
            {
                RobotStates newstate = new RobotStates();
                newstate.robot = allrobots[index];

                // During destruction wierd thing happen, make sure this is all true
                if (allrobots[index].gameObject == null) { continue; }
                if (allrobots[index].gameObject.GetComponent<RobotID>() == null) { continue; }
                newstate.isRed = allrobots[index].gameObject.GetComponent<RobotID>().starting_pos.StartsWith("Red");
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

    // Returns the robot state index for the passed robot
    // Since # of robots is max 6, we will do sequential search which is comparable in time to a hash lookup.
    private int GetRobotStateIndex(RobotInterface3D robot)
    {
        for (int index = 0; index < robotstates.Count; index++)
        {
            if (robotstates[index].robot == robot)
            { return index; }
        }

        return -1;
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

    private void AddToPlayerOPR(UG_powershots indata, float points, bool is_red)
    {
        // If this isn't scored or invalid user, skip
        if ((!indata.scored && !indata.unscored) || indata.hit_by_id < 1) { return; }

        // Validate robot exists (what if they disconnected and this is some phantom data?)
        if (!allrobots_byid.ContainsKey(indata.hit_by_id) || (allrobots_byid[indata.hit_by_id] == null) || (allrobots_byid[indata.hit_by_id].myRobotID == null))
        {
            DoFieldChanged();
        }

        if (!allrobots_byid.ContainsKey(indata.hit_by_id) || (allrobots_byid[indata.hit_by_id] == null) || (allrobots_byid[indata.hit_by_id].myRobotID == null))
        {
            return;
        }

        // If this came from an enemy robot, make it score negative.
        float negate_score = 1f;
        if (allrobots_byid[indata.hit_by_id].myRobotID.is_red != is_red) { negate_score = -1f; }

        // If this is unscored, then negate is as well (you could get double negation here which is as it should be)
        float negate_score2 = (indata.unscored) ? -1f : 1f; ;

        // Add the score to our robot
        if (player_opr.ContainsKey(GLOBALS.client_names[indata.hit_by_id])) { player_opr[GLOBALS.client_names[indata.hit_by_id]] += negate_score * negate_score2 * points; }
        else { player_opr[GLOBALS.client_names[indata.hit_by_id]] = negate_score * negate_score2 * points; }

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

    // Substract the penalty from player opr
    private void SubFromPlayerOPR(UG_powershots indata, float points)
    {
        // If this isn't scored or no penatly, then skip
        if (!indata.scored || (indata.hit_by_id < 1) || !indata.penalty) { return; }

        // Validate robot exists (what if they disconnected and this is some phantom data?)
        if (!allrobots_byid.ContainsKey(indata.hit_by_id) || (allrobots_byid[indata.hit_by_id] == null) || (allrobots_byid[indata.hit_by_id].myRobotID == null))
        {
            DoFieldChanged();
        }

        if (!allrobots_byid.ContainsKey(indata.hit_by_id) || (allrobots_byid[indata.hit_by_id] == null) || (allrobots_byid[indata.hit_by_id].myRobotID == null))
        {
            return;
        }

        // Sub the score from our robot
        if (player_opr.ContainsKey(GLOBALS.client_names[indata.hit_by_id])) { player_opr[GLOBALS.client_names[indata.hit_by_id]] -= points; }
        else { player_opr[GLOBALS.client_names[indata.hit_by_id]] = -1f * points; }

    }

    private bool endgame_started = false;
    private FirstGameState previous_firstgamestate = FirstGameState.FINISHED;

    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if (timerstate != TimerState.RUNNING)
        { return; }

        // Make sure settings have loaded
        if (!su_settings) { return; }

        // Calculate the total score of things so far (that cvan be used in auto as well as teleop)

        // First lets calculate number of rollers
        rollers_red = 0;
        rollers_blue = 0;

        foreach (RollerDetect currroller in rollers)
        {
            if (currroller.isred)
            {
                rollers_red += 1;
            }

            if (!skills_game && currroller.isblue)
            {
                rollers_blue += 1;
            }
        }

        // Next calculate the high goals
        discs_high_red = red_high_goal.GetGameElementCount();
        discs_high_blue = blue_high_goal.GetGameElementCount();

        // Calculate low goal
        discs_low_red = 0;
        foreach(gameElement curr_disc in red_low_goal.game_elements )
        {
            if( red_low_keepout.IsGameElementInside(curr_disc)) { continue; }

            discs_low_red += 1;
        }

        discs_low_blue = 0;
        foreach (gameElement curr_disc in blue_low_goal.game_elements)
        {
            if (blue_low_keepout.IsGameElementInside(curr_disc)) { continue; }

            discs_low_blue += 1;
        }

        // Now lock in auto

        // Determine auto score
        if (!GLOBALS.SINGLEPLAYER_MODE && (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)))
        {
            discs_high_red_auto = discs_high_red;
            discs_low_red_auto = discs_low_red;
            discs_high_blue_auto = discs_high_blue;
            discs_low_blue_auto = discs_low_blue;
            rollers_red_auto = rollers_red;
            rollers_blue_auto = (skills_game) ? 0 : rollers_blue;

            // Calculate auto score
            score_auto_red = 5f * discs_high_red_auto + 1f * discs_low_red_auto + 10f * rollers_red_auto;
            score_auto_blue = 5f * discs_high_blue_auto + 1f * discs_low_blue_auto + 10f * rollers_blue_auto;

            // Calculate bonus if this isn't skills
            if (!skills_game)
            {
                red_auto_bonus = (score_auto_red > score_auto_blue) ? 1f : 0;
                blue_auto_bonus = (score_auto_blue > score_auto_red) ? 1f : 0;

                if (score_auto_blue == score_auto_red)
                {
                    red_auto_bonus = 0.5f;
                    blue_auto_bonus = 0.5f;
                }

                // Get winpoint
                red_winpoint = (rollers_red_auto >= 2) && (discs_high_red_auto >= 2);
                blue_winpoint = (rollers_blue_auto >= 2) && (discs_high_blue_auto >= 2);
            }
        }

        // Finally do number of tiles
        tiles_red = 0;
        tiles_blue = 0;

        if( time_total.TotalSeconds < 15f) // But only during the last 15 seconds
        {
            foreach (GenericFieldTracker curr_tile in all_tiles)
            {
                bool done_red = false;
                bool done_blue = false;

                foreach (RobotID curr_bot in curr_tile.robots)
                {
                    if (!done_red && curr_bot.is_red)
                    {
                        tiles_red += 1;
                        done_red = true;
                        continue;
                    }

                    if (!done_blue && !curr_bot.is_red)
                    {
                        tiles_blue += 1;
                        done_blue = true;
                        continue;
                    }
                }
            }
        }

        // Return rings if applicable
        ReturnRings();

    }

    private bool red_rings_exhausted = false;
    private bool blue_rings_exhausted = false;
    public void ReturnRings()
    {
        // Check if there's a friendly and that the coast is clear
        if (!blue_rings_exhausted && (detect_blue.AreFriendsColliding() || skills_game && detect_blue.AreEnemiesColliding()) && !bluerampdetector.IsAnyGameElementInside() )
        {
            // return a ring
            // First find one
            int i = 0;
            for (i =0; i < extra_rings_blue.childCount; i++)             
            {
                Transform curr_ring = extra_rings_blue.GetChild(i);
                Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
                if (rb && (rb.isKinematic == true))
                {
                    rb.isKinematic = false;
                    curr_ring.SetPositionAndRotation(ringreturn_blue.position, ringreturn_blue.rotation);
                    break;
                }
            }

            // If there are non left, mark it
            if (i >= extra_rings_blue.childCount)
            {
                detect_blue.GetComponent<MeshRenderer>().enabled = false;
                blue_rings_exhausted = true;
            }
        }

        if (!red_rings_exhausted && (detect_red.AreFriendsColliding() || skills_game && detect_red.AreEnemiesColliding()) && !redrampdetector.IsAnyGameElementInside())
        {
            // return a ring
            // First find one
            int i = 0;
            for (i = 0; i < extra_rings_red.childCount; i++)
            {
                Transform curr_ring = extra_rings_red.GetChild(i);
                Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
                if (rb && (rb.isKinematic == true))
                {
                    rb.isKinematic = false;
                    curr_ring.SetPositionAndRotation(ringreturn_red.position, ringreturn_red.rotation);
                    break;
                }
            }

            // If there are non left, mark it
            if (i >= extra_rings_red.childCount)
            {
                detect_red.GetComponent<MeshRenderer>().enabled = false;
                red_rings_exhausted = true;
            }
        }
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
        score_red = 10f*rollers_red + 5f* discs_high_red + 1f* discs_low_red + score_auto_red + 3f * tiles_red;

        return (int) score_red + score_redadj;
    }

    public override int GetBlueScore()
    {
        if( timerstate == TimerState.FINISHED )
        { return score_bluefinal;  }

        score_blue = 10f * rollers_blue + 5f * discs_high_blue + 1f * discs_low_blue + score_auto_blue + 3f * tiles_blue;
        return (int) (score_blue) + score_blueadj;
    }

   

    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

    }

    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

    }
}