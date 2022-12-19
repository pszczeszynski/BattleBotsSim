using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using System.Linq;

public class Scorekeeper_UltimateGoal : Scorekeeper {

    // Game specific settings
    UltimateGoal_Settings ug_settings = null;

    // all scorers
    public IR_scoringBox goal_high_red;
    public IR_scoringBox goal_mid_red;
    public IR_scoringBox goal_low_red;
    public IR_scoringBox goal_high_blue;
    public IR_scoringBox goal_mid_blue;
    public IR_scoringBox goal_low_blue;

    // Wobble Goal stuff
    public GenericFieldTracker wg_ba;
    public GenericFieldTracker wg_bb;
    public GenericFieldTracker wg_bc;
    public GenericFieldTracker wg_ra;
    public GenericFieldTracker wg_rb;
    public GenericFieldTracker wg_rc;

    public GenericFieldTracker wg_fault_red1;
    public GenericFieldTracker wg_fault_red2;
    public GenericFieldTracker wg_fault_red3;
    public GenericFieldTracker wg_fault_red4;
    public GenericFieldTracker wg_fault_red5;
    public GenericFieldTracker wg_fault_blue1;
    public GenericFieldTracker wg_fault_blue2;
    public GenericFieldTracker wg_fault_blue3;
    public GenericFieldTracker wg_fault_blue4;
    public GenericFieldTracker wg_fault_blue5;
    public GenericFieldTracker wg_outside_box;
    public GenericFieldTracker startline_red1;
    public GenericFieldTracker startline_red2;
    public GenericFieldTracker startline_blue1;
    public GenericFieldTracker startline_blue2;
    public GenericFieldTracker launchline;
    public WobbleGoal wg_red1;
    public WobbleGoal wg_red2;
    public WobbleGoal wg_blue1;
    public WobbleGoal wg_blue2;

    public GenericFieldTracker launch_zone;
    public GenericFieldTracker no_launch_zone;


    // Power shots
    public UG_powershots powershot_red1;
    public UG_powershots powershot_red2;
    public UG_powershots powershot_red3;
    public UG_powershots powershot_blue1;
    public UG_powershots powershot_blue2;
    public UG_powershots powershot_blue3;

    // Score details
    public double penalties_red; // additional points given to ref from opposing teams penalties
    public int rings_red;
    public double score_auto_red;
    public double score_teleop_red;
    public double score_endgame_red;
    public double score_red;

    public double penalties_blue; // additional points given to ref from opposing teams penalties
    public int rings_blue;
    public double score_auto_blue;
    public double score_teleop_blue;
    public double score_endgame_blue;
    public double score_blue;


    // Ball return positions
    public Transform ringshold;
    public Transform red_return_pos;
    public Transform blue_return_pos;
    public Transform blue_starting_pos;
    public Transform red_starting_pos;

    public GenericFieldTracker red_return_box;
    public GenericFieldTracker blue_return_box;

    public List<Transform> ringsheld_red = new List<Transform>();
    public List<Transform> ringsheld_blue = new List<Transform>();
    public List<Transform> allrings = new List<Transform>();

    private int auto_roll = 1; // Which number did autonomouse roll: 1 = no rings, 2 = 1 ring, 3 = 4 rings at auto position

    // My own OPR temporary placeholder
    public Dictionary<string, Dictionary<string, float>> player_detailed_opr = new Dictionary<string, Dictionary<string, float>>();
    public Dictionary<string, float> red_detailed_score = new Dictionary<string, float>();
    public Dictionary<string, float> blue_detailed_score = new Dictionary<string, float>();

    // Safety zones
    public IR_fieldtracker blue_safety;
    public IR_fieldtracker red_safety;

    public SelectiveWall auto_red_blocker;
    public SelectiveWall auto_blue_blocker;

    public GameObject zone_half_field_red;
    public GameObject zone_half_field_blue;
    public GameObject zone_walls_red;
    public GameObject zone_walls_blue;

    private Vector3 old_gravity;

    private bool[] play_goal_animation = new bool[6];

    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 150;
        GLOBALS.TIMER_AUTO = 30;
        GLOBALS.TIMER_ENDGAME = 30;

        // Change force of gravity to suit the FTC game
        old_gravity = Physics.gravity;
        Physics.gravity = new Vector3(0, -9.81f * 2f, 0);

        ResetDetails();
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
        ug_settings = GameObject.Find("GameSettings").GetComponent<UltimateGoal_Settings>();

        // Get all rings
        allrings.Clear();
        GameObject[] allgamelements = GameObject.FindGameObjectsWithTag("GameElement");

        for (int i = 0; i < allgamelements.Length; i++)
        {
            // Get the game element script
            gameElement currelement = allgamelements[i].GetComponent<gameElement>();
            if (currelement == null) { continue; }

            // If it's a ring, then it's ID is <= 30
            if ((currelement.id > 0) && (currelement.id <= 30))
            {
                allrings.Add(currelement.transform);

                // Initialize physics
                Rigidbody rb = currelement.GetComponent<Rigidbody>();
                if (!rb) { continue; }

                // Can't make objects inactive during ScorerInit since it can be run before Client or Server had a chance to find the objects
                // currelement.gameObject.SetActive(false); 
                //                rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
                rb.isKinematic = true;
            }
        }

        // Initialize Rules
        // No rules yet
    }


    public override void ScorerReset()
    {
        base.ScorerReset();

        penalties_red = 0;// additional points given to ref from opposing teams penalties
        rings_red = 0;
        score_auto_red = 0;
        score_teleop_red = 0;
        score_endgame_red = 0;
        score_red = 0;

        penalties_blue = 0; // additional points given to ref from opposing teams penalties
        rings_blue = 0;
        score_auto_blue = 0;
        score_teleop_blue = 0;
        score_endgame_blue = 0;
        score_blue = 0;

        // Clear player opr scores
        player_detailed_opr.Clear();
        foreach (string currbot in allrobots_byname.Keys)
        {
            player_detailed_opr[currbot] = new Dictionary<string, float>();
            player_detailed_opr[currbot]["END"] = 0;
            player_detailed_opr[currbot]["AUTO_N"] = 0;
        }

        ResetRings();


        // Clears our own tracking opr scores
        ResetDetails();
    }

    private void ResetDetails()
    {
        red_detailed_score.Clear();
        blue_detailed_score.Clear();

        red_detailed_score["AUTO"] = 0;
        blue_detailed_score["AUTO"] = 0;
        red_detailed_score["END"] = 0;  // This end-game score is only the wobble goal
        blue_detailed_score["END"] = 0; // This end-game score is only the wobble goal
        red_detailed_score["AUTO_RINGS"] = 0;
        blue_detailed_score["AUTO_RINGS"] = 0;
        red_detailed_score["TELEOP_RINGS"] = 0;
        blue_detailed_score["TELEOP_RINGS"] = 0;
        red_detailed_score["AUTO_PS"] = 0;
        blue_detailed_score["AUTO_PS"] = 0;
        red_detailed_score["AUTO_N"] = 0;
        blue_detailed_score["AUTO_N"] = 0;
        red_detailed_score["END_PS"] = 0;
        blue_detailed_score["END_PS"] = 0;
        red_detailed_score["AUTO_WG"] = 0;
        blue_detailed_score["AUTO_WG"] = 0;

    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        // Add this game specific data

        // Total Scores
        data["R_Score"] = ((int)score_red + score_redadj).ToString();
        data["B_Score"] = ((int)score_blue + score_blueadj).ToString();
        data["B_Pen"] = ((int)penalties_blue).ToString();
        data["R_Pen"] = ((int)penalties_red).ToString();
        data["R_C_RINGS"] = rings_red.ToString();
        data["B_C_RINGS"] = rings_blue.ToString();

        // Auto
        data["R_S_Auto"] = ((int)(score_auto_red + red_detailed_score["AUTO"])).ToString();
        data["B_S_Auto"] = ((int)(score_auto_blue + blue_detailed_score["AUTO"])).ToString();
        data["R_C_Auto_R"] = ((int)red_detailed_score["AUTO_RINGS"]).ToString();
        data["B_C_Auto_R"] = ((int)blue_detailed_score["AUTO_RINGS"]).ToString();
        data["R_C_Auto_PS"] = ((int)red_detailed_score["AUTO_PS"]).ToString();
        data["B_C_Auto_PS"] = ((int)blue_detailed_score["AUTO_PS"]).ToString();
        data["R_C_Auto_WG"] = ((int)red_detailed_score["AUTO_WG"]).ToString();
        data["B_C_Auto_WG"] = ((int)blue_detailed_score["AUTO_WG"]).ToString();
        data["R_C_Auto_N"] = ((int)red_detailed_score["AUTO_N"]).ToString();
        data["B_C_Auto_N"] = ((int)blue_detailed_score["AUTO_N"]).ToString();

        // Teleop
        data["R_S_Tele"] = ((int)score_teleop_red).ToString();
        data["B_S_Tele"] = ((int)score_teleop_blue).ToString();
        data["R_C_Tele_R"] = ((int)red_detailed_score["TELEOP_RINGS"]).ToString();
        data["B_C_Tele_R"] = ((int)blue_detailed_score["TELEOP_RINGS"]).ToString();

        // Endgame
        data["R_S_End"] = ((int)(score_endgame_red + red_detailed_score["END"])).ToString();
        data["B_S_End"] = ((int)(score_endgame_blue + blue_detailed_score["END"])).ToString();
        data["R_C_End_PS"] = ((int)red_detailed_score["END_PS"]).ToString();
        data["B_C_End_PS"] = ((int)blue_detailed_score["END_PS"]).ToString();
        data["R_S_End_WG"] = ((int)red_detailed_score["END"]).ToString();
        data["B_S_End_WG"] = ((int)blue_detailed_score["END"]).ToString();

    }

    // Clears the END GMAE OPR value for everyone
    private void OPR_clearitem(string item)
    {
        foreach (Dictionary<string, float> curropr in player_detailed_opr.Values)
        {
            curropr[item] = 0;
        }
    }

    private void ResetRings()
    {
        // Nothing to do in Client mode
        if (GLOBALS.CLIENT_MODE) { return; }

        // Only inactivate rings once everything is loaded and all game objects had a chance to be found
        //if( GLOBALS.SERVER_MODE && GLOBALS.topserver && !ServerLow.configuration_done ||
        //    GLOBALS.CLIENT_MODE && GLOBALS.topclient && !ClientLow.configuration_done )
        //{
        //    return;
        //}

        // Move all rings to reset positions
        foreach (Transform curring in allrings)
        {
            // Reset velocities
            Rigidbody mybody = curring.GetComponent<Rigidbody>();
//            mybody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            mybody.isKinematic = true;
            mybody.velocity = Vector3.zero;
            mybody.angularVelocity = Vector3.zero;
            curring.position = ringshold.position;
            // mybody.gameObject.SetActive(false);
        }
    }

    public override void Restart()
    {
        base.Restart();

        // Nothing to do in Client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        ResetRings();
        goal_high_red.Reset();
        goal_mid_red.Reset();
        goal_low_red.Reset();
        goal_high_blue.Reset();
        goal_mid_blue.Reset();
        goal_low_blue.Reset();
        red_return_box.Reset();
        blue_return_box.Reset();
        ringsheld_blue.Clear();
        ringsheld_red.Clear();

        // Put half of the rings into red, the other into blue holding boxes
        for (int i = 0; i < allrings.Count - 1;)
        {
            ringsheld_blue.Add(allrings[i++]);
            ringsheld_red.Add(allrings[i++]);
        }

        // Reset power shots 
        ResetPowershots(true);

        // Re-enable any disabled wobble goals
        ResetWobbleGoals();

        // Clear outside box
        wg_outside_box.ClearExitedItem();

        // Enable zone pushbacks
        if( ug_settings.ENABLE_HALFFIELD_ZONES)
        {
            zone_half_field_blue.SetActive(true);
            zone_half_field_red.SetActive(true);
        }
        else
        {
            zone_half_field_blue.SetActive(false);
            zone_half_field_red.SetActive(false);
        }

        // Enable zone pushbacks
        if (ug_settings.ENABLE_WALL_ZONES)
        {
            zone_walls_blue.SetActive(true);
            zone_walls_red.SetActive(true);
        }
        else
        {
            zone_walls_blue.SetActive(false);
            zone_walls_red.SetActive(false);
        }

        ResetDetails();
    }

    void ResetWobbleGoals()
    {
        wg_red1.GetComponent<Rigidbody>().isKinematic = false;
        // wg_red1.gameObject.SetActive(true);
        wg_red2.GetComponent<Rigidbody>().isKinematic = false;
        // wg_red2.gameObject.SetActive(true);
        wg_blue1.GetComponent<Rigidbody>().isKinematic = false;
        // wg_blue1.gameObject.SetActive(true);
        wg_blue2.GetComponent<Rigidbody>().isKinematic = false;
        // wg_blue2.gameObject.SetActive(true);
    }

    // Gets run at the beggining of a match
    // This one is guarantted to run after all player positions have been reset to be the starting position, thus is we're placing objects inside the
    // robots, it needs to happen here.
    public override void OnTimerStart()
    {
        // Force a restart: we need to have all rings divided up in the ringsheld lists
        Restart();

        // Get the game option

        // Roll a random number and initialize 
        //if( )
        System.Random rand = new System.Random();
        

        if (GLOBALS.game_option > 1)
        {
            auto_roll = GLOBALS.game_option - 1;
        }
        else
        {
            auto_roll = rand.Next(1, 4);
        }

        List<Transform> auto_rings = new List<Transform>();

        switch (auto_roll)
        {
            case 3: // Move rings 4,3,2 to position
                auto_rings.Add(PopRing(ringsheld_blue, blue_starting_pos.position + (new Vector3(0, 3f * 0.04f, 0)), blue_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_red, red_starting_pos.position + (new Vector3(0, 3f * 0.04f, 0)), red_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_blue, blue_starting_pos.position + (new Vector3(0, 2f * 0.04f, 0)), blue_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_red, red_starting_pos.position + (new Vector3(0, 2f * 0.04f, 0)), red_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_blue, blue_starting_pos.position + (new Vector3(0, 0.04f, 0)), blue_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_red, red_starting_pos.position + (new Vector3(0, 0.04f, 0)), red_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_blue, blue_starting_pos.position, blue_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_red, red_starting_pos.position, red_starting_pos.rotation));
                break;

            case 2:
                auto_rings.Add(PopRing(ringsheld_blue, blue_starting_pos.position, blue_starting_pos.rotation));
                auto_rings.Add(PopRing(ringsheld_red, red_starting_pos.position, red_starting_pos.rotation));
                break;

            default:
                break;
        }

        // Add 3 rings into robots
        foreach (RobotInterface3D currbot in allrobots)
        {
            // Find a Ring loading position
            Transform ringload = MyUtils.FindHierarchy(currbot.transform, "RingLoad");
            if (!ringload) { continue; }

            RobotID robots_id = currbot.GetComponent<RobotID>();
            if (!robots_id) { continue; }

            // Plase 3 rings 
            List<Transform> rings_held = (robots_id.is_red) ? ringsheld_red : ringsheld_blue;
            PopRing(rings_held, ringload.position + ringload.up * 2f * 0.04f, ringload.rotation);
            PopRing(rings_held, ringload.position + ringload.up * 0.04f, ringload.rotation);
            PopRing(rings_held, ringload.position, ringload.rotation);
        }

        // Reset wobble goals
        wg_blue1.Reset();
        wg_blue2.Reset();
        wg_red1.Reset();
        wg_red2.Reset();

        // Enable auto blockers
        if( ug_settings.ENABLE_AUTO_WALLS)
        {
            auto_blue_blocker.disable = false;
            auto_red_blocker.disable = false;
        }
        else
        {
            auto_blue_blocker.disable = true;
            auto_red_blocker.disable = true;
        }
    }

    // Move rings from a list to a target postion/rotation, removing it from the list and setting the ring to be active
    private Transform PopRing(List<Transform> listofrings, Vector3 tolocation, Quaternion torotation)
    {
        Transform currring = listofrings[0];
        listofrings.RemoveAt(0);

        // Move ring
        currring.transform.position = tolocation;
        currring.transform.rotation = torotation;

        // Re-enable it
        Rigidbody mybody = currring.GetComponent<Rigidbody>();
        mybody.velocity = Vector3.zero;
        mybody.angularVelocity = Vector3.zero;
        mybody.isKinematic = false;
//        mybody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        // mybody.collisionDetectionMode = CollisionDetectionMode.Continuous;
        // mybody.gameObject.SetActive(true);

        // Clear internal owner data
        currring.GetComponent<ball_data>().Clear();

        return currring;

    }

    

    // Returns rings down their slot if some are waiting
    private void DeployWaitingRings()
    {
        // If we're passed auto, then return rings
        if (time_total.TotalSeconds > ((double)(GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)))
        {
            return;
        }

        System.Random rand = new System.Random();

        if ((!red_return_box.IsAnyGameElementInside()) && (ringsheld_red.Count > 0))
        {
            Vector3 newrot = red_return_pos.rotation.eulerAngles;
            newrot.y += ((float)rand.NextDouble()) * 10f - 5f;

            PopRing(ringsheld_red, red_return_pos.position, Quaternion.Euler(newrot));
        }

        if ((!blue_return_box.IsAnyGameElementInside()) && (ringsheld_blue.Count > 0))
        {
            Vector3 newrot = blue_return_pos.rotation.eulerAngles;
            newrot.y += ((float)rand.NextDouble()) * 10f - 5f;

            PopRing(ringsheld_blue, blue_return_pos.position, Quaternion.Euler(newrot));
        }
    }

    public void ResetPowershots(bool force_reset = false)
    {
        if (force_reset || !powershot_blue1.up) { powershot_blue1.Reset(); }
        if (force_reset || !powershot_blue2.up) { powershot_blue2.Reset(); }
        if (force_reset || !powershot_blue3.up) { powershot_blue3.Reset(); }
        if (force_reset || !powershot_red1.up) { powershot_red1.Reset(); }
        if (force_reset || !powershot_red2.up) { powershot_red2.Reset(); }
        if (force_reset || !powershot_red3.up) { powershot_red3.Reset(); }

    }

    public override void ScorerUpdate(bool last_frame = false)
    {
        bool game_running = true;
        if (base.timerstate != TimerState.PAUSED &&
            base.timerstate != TimerState.RUNNING
            )
        {
            game_running = false;
        }

        // Update score
        if( GLOBALS.CLIENT_MODE) { return; }

        // Calculate the score
        CalculateScores();

        // Return Rings if possible
        DeployWaitingRings();

        // Reset powershots if applicable
        if (game_running && (time_total.TotalSeconds > GLOBALS.TIMER_AUTO) && (time_total.TotalSeconds < GLOBALS.TIMER_AUTO + 1)) { ResetPowershots(); }

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

        while (index >= 1)
        {
            // Decrement index
            index--;

            RobotStates currbot = robotstates[index];

            // Skip this bot if it's been destroyed
            if (!currbot.robot) { continue; }


            // ********* Update Progress Bars **********
            // Deal with counting up/down
            if (currbot.counting_down)
            {
                //  Check if timer expired
                if (time - currbot.start_time > ug_settings.BLOCKING_DURATION)
                {
                    currbot.counting_down = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((ug_settings.BLOCKING_DURATION - time + currbot.start_time) / ug_settings.BLOCKING_DURATION);
                }
            }

            if (currbot.counting_up)
            {
                // If blocking is disabled, then count down
                if (!ug_settings.ENABLE_BLOCKING)
                {
                    // Start counting down
                    currbot.counting_up = false;
                    currbot.counting_down = true;
                    currbot.start_time = time - (ug_settings.BLOCKING_DURATION - (time - currbot.start_time));
                }
                //  Check if timer expired
                else if (time - currbot.start_time > ug_settings.BLOCKING_DURATION)
                {
                    currbot.counting_up = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                    currbot.robot.MarkForReset(ug_settings.BLOCKING_RESET_HOLDING_TIME);
                    if (game_running)
                    {
                        // Increment the OPR of the robots colliding
                        List<RobotInterface3D> enemies = currbot.robot.GetAllEnemies();
                        foreach (RobotInterface3D enemybot in enemies)
                        {
                            if (player_opr.ContainsKey(GLOBALS.client_names[enemybot.myRobotID.id]))
                            {
                                player_opr[GLOBALS.client_names[enemybot.myRobotID.id]] += ug_settings.PENALTY_BLOCKING / enemies.Count;
                            }
                            else
                            {
                                player_opr[GLOBALS.client_names[enemybot.myRobotID.id]] = ug_settings.PENALTY_BLOCKING / enemies.Count;
                            }

                        }

                        if (currbot.isRed) { penalties_red += ug_settings.PENALTY_BLOCKING; }
                        else { penalties_blue += ug_settings.PENALTY_BLOCKING; }
                    }
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((time - currbot.start_time) / ug_settings.BLOCKING_DURATION);
                }

            }

            // Check for a valid blocking situation
            foreach (RobotInterface3D enemybot in currbot.robot.GetAllEnemies())
            {
                // Get enemies state
                int enemyindex = GetRobotStateIndex(enemybot);
                if (enemyindex < 0) { continue; }
                RobotStates enemystate = robotstates[enemyindex];

                // If we are inside our safety zone more then the enemy, then count enemy up

                if (ug_settings.ENABLE_BLOCKING && currbot.isRed && red_safety.IsFriendInside(currbot.robot.transform) && (red_safety.GetClosestDistance(currbot.robot.rb_body.transform.position) + 0.01f < blue_safety.GetClosestDistance(enemybot.rb_body.transform.position)) ||
                   ug_settings.ENABLE_BLOCKING && !currbot.isRed && blue_safety.IsFriendInside(currbot.robot.transform) && (blue_safety.GetClosestDistance(currbot.robot.rb_body.transform.position) + 0.01f < red_safety.GetClosestDistance(enemybot.rb_body.transform.position)))
                {
                    enemystate.isblocking = true;

                    if (enemystate.counting_down)
                    {
                        enemystate.counting_down = false;
                        enemystate.counting_up = true;
                        enemystate.start_time = time - (ug_settings.BLOCKING_DURATION - (time - enemystate.start_time));
                    }
                    else if (!enemystate.counting_up)
                    {
                        enemystate.counting_up = true;
                        enemystate.start_time = time;
                    }
                }
            }

        }

        // Now go through bots again and see if a blocking bot is no longer blocking
        index = robotstates.Count;
        while (index >= 1 && ug_settings.ENABLE_BLOCKING)
        {
            index--;

            if (!robotstates[index].isblocking && robotstates[index].counting_up)
            {
                robotstates[index].counting_up = false;
                robotstates[index].counting_down = true;
                robotstates[index].start_time = time - (ug_settings.BLOCKING_DURATION - (time - robotstates[index].start_time));

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
    private bool wobble_goals_teleop_reset = false;
    private FirstGameState previous_firstgamestate = FirstGameState.FINISHED;

    private void CalculateScores()
    {
        // Go through all the high, mid, low goals and get ball count
        // Also add end-game stuff
        if (timerstate == TimerState.RUNNING)
        {
            // Clear AUTO re-callculated data
            if (firstgamestate == FirstGameState.AUTO)
            {
                red_detailed_score["AUTO"] = 0;
                blue_detailed_score["AUTO"] = 0;
                endgame_started = false;
                wobble_goals_teleop_reset = false;
            }

            // Finalize Auto data when auto finishes
            if ((previous_firstgamestate == FirstGameState.AUTO) &&  (firstgamestate != FirstGameState.AUTO))
            {
                close_temp_opr("AUTO_N");
            }
            
            // Clear END-Game re-calculated scores
            if (firstgamestate == FirstGameState.ENDGAME)
            {
                red_detailed_score["END"] = 0;
                blue_detailed_score["END"] = 0;
            }

            // Finalize Auto data when auto finishes
            if ((previous_firstgamestate == FirstGameState.AUTO) && (firstgamestate != FirstGameState.AUTO))
            {
                close_temp_opr("AUTO_N");
            }

            previous_firstgamestate = firstgamestate;

            // If it's not auto, make sure auto wall is off
            if (firstgamestate != FirstGameState.AUTO)
            {
                auto_blue_blocker.disable = true;
                auto_red_blocker.disable = true;

            }

            

            // Increment our ring counter
            rings_red += goal_high_red.number_of_balls + goal_mid_red.number_of_balls + goal_low_red.number_of_balls;
            rings_blue += goal_high_blue.number_of_balls + goal_mid_blue.number_of_balls + goal_low_blue.number_of_balls;

            // Score the balls in high/mid/low
            if (firstgamestate == FirstGameState.AUTO) // Auto score
            {
                // Get Detailed ring count
                red_detailed_score["AUTO_RINGS"] += goal_high_red.number_of_balls + goal_mid_red.number_of_balls + goal_low_red.number_of_balls;
                blue_detailed_score["AUTO_RINGS"] += goal_high_blue.number_of_balls + goal_mid_blue.number_of_balls + goal_low_blue.number_of_balls;

                score_auto_red += 12 * goal_high_red.number_of_balls + 6 * goal_mid_red.number_of_balls + 3 * goal_low_red.number_of_balls;
                score_auto_blue += 12 * goal_high_blue.number_of_balls + 6 * goal_mid_blue.number_of_balls + 3 * goal_low_blue.number_of_balls;
                AddToPlayerOPR(goal_high_red.user_ball_count, 12, true);
                AddToPlayerOPR(goal_mid_red.user_ball_count, 6, true);
                AddToPlayerOPR(goal_low_red.user_ball_count, 3, true);
                AddToPlayerOPR(goal_high_blue.user_ball_count, 12, false);
                AddToPlayerOPR(goal_mid_blue.user_ball_count, 6, false);
                AddToPlayerOPR(goal_low_blue.user_ball_count, 3, false);

                // Calculate penalties penalties
                penalties_red += ug_settings.PENALTY_MAJOR * (goal_high_red.red_bad_elements + goal_high_blue.red_bad_elements + goal_mid_red.red_bad_elements + goal_mid_blue.red_bad_elements);
                penalties_blue += ug_settings.PENALTY_MAJOR * (goal_high_red.blue_bad_elements + goal_high_blue.blue_bad_elements + goal_mid_red.blue_bad_elements + goal_mid_blue.blue_bad_elements);
            }
            else
            {
                // Get Detailed ring count
                red_detailed_score["TELEOP_RINGS"] += goal_high_red.number_of_balls + goal_mid_red.number_of_balls + goal_low_red.number_of_balls;
                blue_detailed_score["TELEOP_RINGS"] += goal_high_blue.number_of_balls + goal_mid_blue.number_of_balls + goal_low_blue.number_of_balls;

                score_teleop_red += 6 * goal_high_red.number_of_balls + 4 * goal_mid_red.number_of_balls + 2 * goal_low_red.number_of_balls;
                score_teleop_blue += 6 * goal_high_blue.number_of_balls + 4 * goal_mid_blue.number_of_balls + 2 * goal_low_blue.number_of_balls;
                AddToPlayerOPR(goal_high_red.user_ball_count, 6, true);
                AddToPlayerOPR(goal_mid_red.user_ball_count, 4, true);
                AddToPlayerOPR(goal_low_red.user_ball_count, 2, true);
                AddToPlayerOPR(goal_high_blue.user_ball_count, 6, false);
                AddToPlayerOPR(goal_mid_blue.user_ball_count, 4, false);
                AddToPlayerOPR(goal_low_blue.user_ball_count, 2, false);

                penalties_red += ug_settings.PENALTY_MAJOR * (goal_high_red.red_bad_elements + goal_high_blue.red_bad_elements + goal_mid_red.red_bad_elements + goal_mid_blue.red_bad_elements);
                penalties_blue += ug_settings.PENALTY_MAJOR * (goal_high_red.blue_bad_elements + goal_high_blue.blue_bad_elements + goal_mid_red.blue_bad_elements + goal_mid_blue.blue_bad_elements);
            }

            // Subtract penalties from OPR readings
            SubFromPlayerOPR(goal_high_red.bad_ball_count, ug_settings.PENALTY_MAJOR);
            SubFromPlayerOPR(goal_mid_red.bad_ball_count, ug_settings.PENALTY_MAJOR);
            SubFromPlayerOPR(goal_high_blue.bad_ball_count, ug_settings.PENALTY_MAJOR);
            SubFromPlayerOPR(goal_mid_blue.bad_ball_count, ug_settings.PENALTY_MAJOR);

            // Score power shots
            if (firstgamestate == FirstGameState.AUTO) // Auto score
            {
                blue_detailed_score["AUTO_PS"] += ((powershot_blue1.scored) ? 1 : 0) + ((powershot_blue2.scored) ? 1 : 0) + ((powershot_blue3.scored) ? 1 : 0);
                red_detailed_score["AUTO_PS"] += ((powershot_red1.scored) ? 1 : 0) + ((powershot_red2.scored) ? 1 : 0) + ((powershot_red3.scored) ? 1 : 0);


                // Add to score if scored
                score_auto_blue += ((powershot_blue1.scored) ? 15 : 0) + ((powershot_blue2.scored) ? 15 : 0) + ((powershot_blue3.scored) ? 15 : 0);
                score_auto_red += ((powershot_red1.scored) ? 15 : 0) + ((powershot_red2.scored) ? 15 : 0) + ((powershot_red3.scored) ? 15 : 0);

                // Subtract from score if unscored
                blue_detailed_score["END_PS"] -= ((powershot_blue1.unscored) ? 1 : 0) + ((powershot_blue2.unscored) ? 1 : 0) + ((powershot_blue3.unscored) ? 1 : 0);
                red_detailed_score["END_PS"] -= ((powershot_red1.unscored) ? 1 : 0) + ((powershot_red2.unscored) ? 1 : 0) + ((powershot_red3.unscored) ? 1 : 0);

                score_auto_blue -= ((powershot_blue1.unscored) ? 15 : 0) + ((powershot_blue2.unscored) ? 15 : 0) + ((powershot_blue3.unscored) ? 15 : 0);
                score_auto_red -= ((powershot_red1.unscored) ? 15 : 0) + ((powershot_red2.unscored) ? 15 : 0) + ((powershot_red3.unscored) ? 15 : 0);

                AddToPlayerOPR(powershot_red1, 15, true);
                AddToPlayerOPR(powershot_red2, 15, true);
                AddToPlayerOPR(powershot_red3, 15, true);
                AddToPlayerOPR(powershot_blue1, 15, false);
                AddToPlayerOPR(powershot_blue2, 15, false);
                AddToPlayerOPR(powershot_blue3, 15, false);

                // Subtract penalties
                penalties_blue += ug_settings.PENALTY_MAJOR * (((powershot_blue1.scored && powershot_blue1.penalty) ? 1 : 0) + ((powershot_blue2.scored && powershot_blue2.penalty) ? 1 : 0) + ((powershot_blue3.scored && powershot_blue3.penalty) ? 1 : 0));
                penalties_red += ug_settings.PENALTY_MAJOR * (((powershot_red1.scored && powershot_red1.penalty) ? 1 : 0) + ((powershot_red2.scored && powershot_red2.penalty) ? 1 : 0) + ((powershot_red3.scored && powershot_red3.penalty) ? 1 : 0));

                // Subtract penalties from OPR readings
                SubFromPlayerOPR(powershot_blue1, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_blue2, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_blue3, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_red1, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_red2, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_red3, ug_settings.PENALTY_MAJOR);

                // Reset scored state of powershots (so that they don't double count)
                powershot_blue1.ClearScore(); powershot_blue2.ClearScore(); powershot_blue3.ClearScore();
                powershot_red1.ClearScore(); powershot_red2.ClearScore(); powershot_red3.ClearScore();
            }


            // Score navigation
            if (firstgamestate == FirstGameState.AUTO) // Auto score
            {
                blue_detailed_score["AUTO_N"] = 0;
                red_detailed_score["AUTO_N"] = 0;


                foreach ( RobotID currbot in allRobotID.Values)
                {
                    // Make sure client exists
                    if( !player_detailed_opr.ContainsKey(GLOBALS.client_names[currbot.id]))
                    {
                        player_detailed_opr[GLOBALS.client_names[currbot.id]] = new Dictionary<string, float>();
                    }

                    // Clear opr
                    player_detailed_opr[GLOBALS.client_names[currbot.id]]["AUTO_N"] = 0;

                    if (currbot.is_red && launchline.IsRobotInside(currbot))
                    { 
                        red_detailed_score["AUTO"] += 5f;
                        red_detailed_score["AUTO_N"] += 1f;
                        player_detailed_opr[GLOBALS.client_names[currbot.id]]["AUTO_N"] = 5f;
                    }

                    if (!currbot.is_red && launchline.IsRobotInside(currbot)) 
                    { 
                        blue_detailed_score["AUTO"] += 5f;
                        blue_detailed_score["AUTO_N"] += 1f;
                        player_detailed_opr[GLOBALS.client_names[currbot.id]]["AUTO_N"] = 5f;
                    }
                }
            }

            // End-game score
            // Score power shots
            // Auto score the balls in high/mid/low
            // Normally we would score
            if (firstgamestate == FirstGameState.ENDGAME) // End-game score
            {
                // Add to score if scored
                blue_detailed_score["END_PS"] += ((powershot_blue1.scored) ? 1 : 0) + ((powershot_blue2.scored) ? 1 : 0) + ((powershot_blue3.scored) ? 1 : 0);
                red_detailed_score["END_PS"] += ((powershot_red1.scored) ? 1 : 0) + ((powershot_red2.scored) ? 1 : 0) + ((powershot_red3.scored) ? 1 : 0);                
                score_endgame_blue += ((powershot_blue1.scored) ? 15 : 0) + ((powershot_blue2.scored) ? 15 : 0) + ((powershot_blue3.scored) ? 15 : 0);
                score_endgame_red += ((powershot_red1.scored) ? 15 : 0) + ((powershot_red2.scored) ? 15 : 0) + ((powershot_red3.scored) ? 15 : 0);

                // Subtract frm score if unscored
                blue_detailed_score["END_PS"] -= ((powershot_blue1.unscored) ? 1 : 0) + ((powershot_blue2.unscored) ? 1 : 0) + ((powershot_blue3.unscored) ? 1 : 0);
                red_detailed_score["END_PS"] -= ((powershot_red1.unscored) ? 1 : 0) + ((powershot_red2.unscored) ? 1 : 0) + ((powershot_red3.unscored) ? 1 : 0);
                score_endgame_blue -= ((powershot_blue1.unscored) ? 15 : 0) + ((powershot_blue2.unscored) ? 15 : 0) + ((powershot_blue3.unscored) ? 15 : 0);
                score_endgame_red -= ((powershot_red1.unscored) ? 15 : 0) + ((powershot_red2.unscored) ? 15 : 0) + ((powershot_red3.unscored) ? 15 : 0);


                AddToPlayerOPR(powershot_red1, 15, true);
                AddToPlayerOPR(powershot_red2, 15, true);
                AddToPlayerOPR(powershot_red3, 15, true);
                AddToPlayerOPR(powershot_blue1, 15, false);
                AddToPlayerOPR(powershot_blue2, 15, false);
                AddToPlayerOPR(powershot_blue3, 15, false);

                // Subtract penalties
                penalties_blue += ug_settings.PENALTY_MAJOR * (((powershot_blue1.scored && powershot_blue1.penalty) ? 1 : 0) + ((powershot_blue2.scored && powershot_blue2.penalty) ? 1 : 0) + ((powershot_blue3.scored && powershot_blue3.penalty) ? 1 : 0));
                penalties_red += ug_settings.PENALTY_MAJOR * (((powershot_red1.scored && powershot_red1.penalty) ? 1 : 0) + ((powershot_red2.scored && powershot_red2.penalty) ? 1 : 0) + ((powershot_red3.scored && powershot_red3.penalty) ? 1 : 0));

                // Subtract penalties from OPR readings
                SubFromPlayerOPR(powershot_blue1, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_blue2, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_blue3, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_red1, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_red2, ug_settings.PENALTY_MAJOR);
                SubFromPlayerOPR(powershot_red3, ug_settings.PENALTY_MAJOR);

                // Reset scored state of powershots (so that they don't double count)
                powershot_blue1.ClearScore(); powershot_blue2.ClearScore(); powershot_blue3.ClearScore();
                powershot_red1.ClearScore(); powershot_red2.ClearScore(); powershot_red3.ClearScore();
            }

            // Initialize wobble goals
            if (!endgame_started && (firstgamestate == FirstGameState.ENDGAME))// End-game score
            {
                endgame_started = true;

                InitWobbleGoalForEndGame(wg_blue1);
                InitWobbleGoalForEndGame(wg_blue2);
                InitWobbleGoalForEndGame(wg_red1);
                InitWobbleGoalForEndGame(wg_red2);
                wg_outside_box.ClearExitedItem();
                wg_outside_box.Reset();

            }

            // Score wobble goals in auto
            if (firstgamestate == FirstGameState.AUTO)
            {
                // Select the target box
                GenericFieldTracker target_box_red;
                GenericFieldTracker target_box_blue;

                if (auto_roll == 1) { target_box_blue = wg_ba; target_box_red = wg_ra; }
                else if (auto_roll == 2) { target_box_blue = wg_bb; target_box_red = wg_rb; }
                else { target_box_blue = wg_bc; target_box_red = wg_rc; }

                // Check the scored wobbgle goals
                bool scored_red1 = false;
                bool scored_red2 = false;
                bool scored_blue1 = false;
                bool scored_blue2 = false;

                // Blue1
                if (target_box_blue.IsGameElementInside(wg_blue1.transform) && !wg_fault_blue1.IsGameElementInside(wg_blue1.transform) && !wg_fault_blue2.IsGameElementInside(wg_blue1.transform)
                        && !wg_fault_blue3.IsGameElementInside(wg_blue1.transform) && !wg_fault_blue4.IsGameElementInside(wg_blue1.transform) && !wg_fault_blue5.IsGameElementInside(wg_blue1.transform))
                { scored_blue1 = true; }
                // Blue2
                if (target_box_blue.IsGameElementInside(wg_blue2.transform) && !wg_fault_blue1.IsGameElementInside(wg_blue2.transform) && !wg_fault_blue2.IsGameElementInside(wg_blue2.transform)
                        && !wg_fault_blue3.IsGameElementInside(wg_blue2.transform) && !wg_fault_blue4.IsGameElementInside(wg_blue2.transform) && !wg_fault_blue5.IsGameElementInside(wg_blue2.transform))
                { scored_blue2 = true; }
                // Red1
                if (target_box_red.IsGameElementInside(wg_red1.transform) && !wg_fault_red1.IsGameElementInside(wg_red1.transform) && !wg_fault_red2.IsGameElementInside(wg_red1.transform)
                        && !wg_fault_red3.IsGameElementInside(wg_red1.transform) && !wg_fault_red4.IsGameElementInside(wg_red1.transform) && !wg_fault_red5.IsGameElementInside(wg_red1.transform))
                { scored_red1 = true; }
                // Red2
                if (target_box_red.IsGameElementInside(wg_red2.transform) && !wg_fault_red1.IsGameElementInside(wg_red2.transform) && !wg_fault_red2.IsGameElementInside(wg_red2.transform)
                        && !wg_fault_red3.IsGameElementInside(wg_red2.transform) && !wg_fault_red4.IsGameElementInside(wg_red2.transform) && !wg_fault_red5.IsGameElementInside(wg_red2.transform))
                { scored_red2 = true; }

                // Add them into the score
                red_detailed_score["AUTO"] += ((scored_red1) ? 15 : 0) + ((scored_red2) ? 15 : 0);
                blue_detailed_score["AUTO"] += ((scored_blue1) ? 15 : 0) + ((scored_blue2) ? 15 : 0);

                red_detailed_score["AUTO_WG"] = ((scored_red1) ? 1 : 0) + ((scored_red2) ? 1 : 0);
                blue_detailed_score["AUTO_WG"] = ((scored_blue1) ? 1 : 0) + ((scored_blue2) ? 1 : 0);

                // Set proper state of wobble goal
                wg_blue1.valid = scored_blue1; wg_blue1.auto_inplay = true;
                wg_blue2.valid = scored_blue2; wg_blue2.auto_inplay = true;
                wg_red1.valid = scored_red1; wg_red1.auto_inplay = true;
                wg_red2.valid = scored_red2; wg_red2.auto_inplay = true;

                // Add to player OPR
                Score_WG_OPR(scored_red1, wg_red1, 15f);
                Score_WG_OPR(scored_red2, wg_red2, 15f);
                Score_WG_OPR(scored_blue1, wg_blue1, 15f);
                Score_WG_OPR(scored_blue2, wg_blue2, 15f);
            }

            // Reset wobble goals after auto
            if (firstgamestate == FirstGameState.TELEOP && !wobble_goals_teleop_reset)
            {
                // Let wobble goals know auto isn't in play
                wg_blue1.Reset();
                wg_blue2.Reset();
                wg_red1.Reset();
                wg_red2.Reset();
                wobble_goals_teleop_reset = true;
            }

            // Score wobble goals endgame
            if (endgame_started)
            {
                // Recalculate new
                red_detailed_score["END"] += Calculate_WG_Endgame(wg_red1, startline_red1, startline_red2);
                red_detailed_score["END"] += Calculate_WG_Endgame(wg_red2, startline_red1, startline_red2);
                blue_detailed_score["END"] += Calculate_WG_Endgame(wg_blue1, startline_blue1, startline_blue2);
                blue_detailed_score["END"] += Calculate_WG_Endgame(wg_blue2, startline_blue1, startline_blue2);
            }


        }

        // Return the rings to either their holding position
        // Also mark if we need to play any animations     
        play_goal_animation[0] |= ReturnRings(goal_high_blue, ringsheld_blue);
        play_goal_animation[1] |= ReturnRings(goal_mid_blue, ringsheld_red);
        play_goal_animation[2] |= ReturnRings(goal_low_blue, ringsheld_blue);
        play_goal_animation[3] |= ReturnRings(goal_high_red, ringsheld_red);
        play_goal_animation[4] |= ReturnRings(goal_mid_red, ringsheld_blue);
        play_goal_animation[5] |= ReturnRings(goal_low_red, ringsheld_red);
    }

    private float Calculate_WG_Endgame(WobbleGoal wg, GenericFieldTracker startline1, GenericFieldTracker startline2)
    {
        // Make sure it's a valid goal in endgame that can score
        if( !wg.valid ) { return 0;  }

        // Check startlines
        if( startline1.IsGameElementInside(wg.transform) || startline2.IsGameElementInside(wg.transform) )
        {
            Score_WG_OPR(true, wg, 5f);
            return 5f;
        }
        else
        {
            Score_WG_OPR(false, wg, 5f);
        }

        if( wg_outside_box.IsGameElementInside(wg.transform))
        {
            Score_WG_OPR(true, wg, 20f);

            // Clear the WG collision info since valid_old is used to track startline in/out
            wg.robot_scored = -1;
            wg.last_contact_robot = null;

            return 20f;
        }

        return 0;
    }

    private void Score_WG_OPR(bool scored, WobbleGoal wg, float points )
    {
        // try to get OPR for the wobble goals
        if (scored && !wg.valid_old && wg.last_contact_robot)
        {
            AddToPlayerOPR(wg.last_contact_robot, points);
        }
        if (!scored && wg.valid_old && wg.last_contact_robot)
        {
            AddToPlayerOPR(wg.last_contact_robot, -1f * points);
        }

        wg.valid_old = scored;
    }

    private void close_temp_opr(string key_to_close)
    {
        foreach(string id in player_detailed_opr.Keys)
        {
            // If this player does have this key then skip
            if(!player_detailed_opr[id].ContainsKey(key_to_close)) { continue; }

            // Add to opr
            if( !player_opr.ContainsKey(id))
            {
                player_opr[id] = player_detailed_opr[id][key_to_close];
            }
            else
            {
                player_opr[id] += player_detailed_opr[id][key_to_close];
            }

            player_detailed_opr[id][key_to_close] = 0f;
        }
    }

    private void InitWobbleGoalForEndGame(WobbleGoal goal)
    {
        // Mark it's endgame
        goal.endgame_started = true;
        goal.valid = false;
        goal.valid_old = false;

        // If it's not in launch zone, then it's valid
        if( ! launch_zone.IsGameElementInside(goal.transform))
        {
            goal.valid = true;
            return;
        }

        // If it's inside a scoring zone, then let it through (only own team color counts - for now)
        if( goal.is_red && (wg_ra.IsGameElementInside(goal.transform) || wg_rb.IsGameElementInside(goal.transform) || wg_rc.IsGameElementInside(goal.transform)))
        {
            goal.valid = true;
            return;
        }
        else if(wg_ba.IsGameElementInside(goal.transform) || wg_bb.IsGameElementInside(goal.transform) || wg_bc.IsGameElementInside(goal.transform))
        {
            goal.valid = true;
            return;
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
        score_red = score_auto_red + red_detailed_score["AUTO"] + score_teleop_red + score_endgame_red + red_detailed_score["END"] - penalties_red; 

        return (int) score_red + score_redadj;
    }

    public override int GetBlueScore()
    {
        if( timerstate == TimerState.FINISHED )
        { return score_bluefinal;  }

        score_blue = score_auto_blue + blue_detailed_score["AUTO"] + score_teleop_blue + score_endgame_blue + blue_detailed_score["END"] - penalties_blue;
        return (int) (score_blue) + score_blueadj;
    }

    // Helper function to go through balls in scoring zones and return them
    private bool ReturnRings(IR_scoringBox goal, List<Transform> ringsheld)
    {
        bool rings_returned = false;

        // Return any rings that scored
        Transform currring = goal.GetNextBall();
        while (currring)
        {
            ReturnRing(currring, ringsheld);
            rings_returned = true;
            currring = goal.GetNextBall();
        }

        goal.Reset();

        return rings_returned;
    }

    private void ReturnRing(Transform currring, List<Transform> ringsheld)
    {
        // Reset velocities
        Rigidbody mybody = currring.GetComponent<Rigidbody>();
 //       mybody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        mybody.isKinematic = true;
        mybody.velocity = Vector3.zero;
        mybody.angularVelocity = Vector3.zero;
        currring.position = ringshold.position;
        ringsheld.Add(currring);
        // mybody.gameObject.SetActive(false);
    }

    // No restrictions on robot choice at this time
    public override string CorrectRobotChoice(string requested_robot)
    {
        //if( requested_robot == "Bot Royale")
        //{
        //    return "FRC shooter";
        //}

        return requested_robot;
    }

    // Corrects the field element if required. By default it just checks if the element
    // flew off the board
    override public bool CorrectFieldElement(GameObject currobj)
    {
        if (!floor_was_found)
        {
            // Extract the field positions used in putting elements back in game-play
            GameObject floor = GameObject.Find("3d field/matts");
            if (floor == null)
            {
                floor = GameObject.Find("3d field/floor");
            }

            if (floor != null)
            {
                floor_max = floor.GetComponent<Renderer>().bounds.max;
                floor_min = floor.GetComponent<Renderer>().bounds.min;
                floor_was_found = true;
            }
        }


        // if object flew off board, correct it
        if (currobj.transform.position.y < floor_min.y - 5)
        {
            // Reset game-elements speed
            Rigidbody rb = currobj.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = new Vector3(0, 0, 0);
                rb.angularVelocity = new Vector3(0, 0, 0);
            }

            // If this is a ring, then deal with it seperetally
            gameElement curr_gameelement = currobj.GetComponent<gameElement>();

            if (curr_gameelement && curr_gameelement.type == ElementType.Jewel)
            {
                int return_slot = UnityEngine.Random.Range(0, 2);
                if (return_slot == 0)
                {
                    ReturnRing(currobj.transform, ringsheld_red);
                }
                else
                {
                    ReturnRing(currobj.transform, ringsheld_blue);
                }
            }

            // If this is a wobble goal, then we need to disable it
            else if (curr_gameelement && (curr_gameelement.type == ElementType.Cube || curr_gameelement.type == ElementType.CubeDark))
            {
                // Move goal tottaly out of the way
                currobj.transform.position = new Vector3(
                    (floor_min.x + floor_max.x)/2f, floor_max.y - 20f, (floor_min.z + floor_max.z)/2f);
                currobj.GetComponent<Rigidbody>().isKinematic = true;
                // currobj.SetActive(false);
            }
            else
            {
                currobj.transform.position = new Vector3(
                        UnityEngine.Random.Range(floor_min.x, floor_max.x), floor_max.y + 20f, UnityEngine.Random.Range(floor_min.z, floor_max.z));
            }
            return true;
        }

        return false;
    }


    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

        // Add if we need to play animations
        serverFlags["ANIM"] =   (play_goal_animation[0] ? "1" : "0") +
                                (play_goal_animation[1] ? "1" : "0") +
                                (play_goal_animation[2] ? "1" : "0") + 
                                (play_goal_animation[3] ? "1" : "0") +
                                (play_goal_animation[4] ? "1" : "0") +
                                (play_goal_animation[5] ? "1" : "0");

        
        for (int i = 0; i < 6; i++) { play_goal_animation[i] = false; }
    }

    // Receiveflags from server
    int animation_played = 0;
    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

        if (!serverFlags.ContainsKey("ANIM") )
        { return; }

        // Play the animations 
        string animations = serverFlags["ANIM"];
        if (animations.Length < 6) { return; }

        if (animations[0] == '1') 
        { goal_high_blue.PlayAnimation(); }
        if (animations[1] == '1') 
        { goal_mid_blue.PlayAnimation(); }
        if (animations[2] == '1') 
        { goal_low_blue.PlayAnimation(); }
        if (animations[3] == '1') 
        {   
            goal_high_red.PlayAnimation();
        }
        if (animations[4] == '1') 
        { goal_mid_red.PlayAnimation(); }
        if (animations[5] == '1') 
        { goal_low_red.PlayAnimation(); }

        // Clear the flags
        serverFlags["ANIM"] = "0";
    }
}