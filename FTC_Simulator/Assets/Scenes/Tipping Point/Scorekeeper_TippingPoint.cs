using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_TippingPoint : Scorekeeper {

    // Game specific settings
    TippingPoint_Settings tp_settings = null;
    ClientLow client = null;

    public GoalCounter red_goal_counter;
    public GoalCounter blue_goal_counter;

    public Transform platform_detector_blue;
    public Transform platform_detector_red;

    private PlatformCounter bd_plat_red;  // Balance detector: platform detector for red
    private PlatformCounter bd_plat_blue;   // Balance detector: platform detector for blue
    private RobotCounter bd_robot_red;      // Balance detector: robot detector for red
    private RobotCounter bd_robot_blue;     // Balance detector: robot detector for blue
    private GoalCounter bd_goal_red;      // Balance detector: goal detector for red
    private GoalCounter bd_goal_blue;     // Balance detector: goal detector for blue

    public GoalCounter Balance_Goal_Counter_Red;
    public RobotCounter Balance_Robot_Counter_Red;
    public GoalCounter Balance_Goal_Counter_Blue;
    public RobotCounter Balance_Robot_Counter_Blue;
    public RobotCollision Balance_Robot_Collision_Red;
    public RobotCollision Balance_Robot_Collision_Blue;

    public Transform preload_redl;
    public Transform preload_redr;
    public Transform preload_bluel;
    public Transform preload_bluer;
    public Transform extra_rings_red;
    public Transform extra_rings_blue;
    public Transform ringreturn_bl;
    public Transform ringreturn_br;
    public Transform ringreturn_rl;
    public Transform ringreturn_rr;
    public FRC_DepotHandler detect_bl;
    public FRC_DepotHandler detect_br;
    public FRC_DepotHandler detect_rl;
    public FRC_DepotHandler detect_rr;
    public bool blue_rings_returned = false;
    public bool red_rings_returned = false;




    // all scorers

    // Blocking push-back
    public GameObject pushback;

    public float score_red = 0;
    public float score_blue = 0;

    public GoalScorer[] all_goals;

    // Rules variables
    private Vector3 old_gravity;
    private void Awake()
    {
        // Set this to be an FTC/VEX game
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 15;
        GLOBALS.TIMER_ENDGAME = 0;

        // Change force of gravity to suit the change-up game
        old_gravity = Physics.gravity;
        Physics.gravity = new Vector3(0, -9.81f * 2f, 0);
    }

    private void OnDestroy()
    {
        // Return gravity back to original
        Physics.gravity = old_gravity;
    }

    public override void ScorerInit()
    {
        ScorerReset();

        GameObject settings = GameObject.Find("GameSettings");
        if( settings ) {  tp_settings = settings.GetComponent<TippingPoint_Settings>(); }

        GameObject theclient = GameObject.Find("Client");
        if( theclient ) { client = theclient.GetComponent<ClientLow>(); }

        // Get all the goals
        all_goals = FindObjectsOfType<GoalScorer>();

        // Extract the balance counters
        bd_plat_red = platform_detector_red.GetComponent<PlatformCounter>();
        bd_plat_blue = platform_detector_blue.GetComponent<PlatformCounter>();
        bd_robot_red = platform_detector_red.GetComponent<RobotCounter>();
        bd_robot_blue = platform_detector_blue.GetComponent<RobotCounter>();
        bd_goal_red = platform_detector_red.GetComponent<GoalCounter>();
        bd_goal_blue = platform_detector_blue.GetComponent<GoalCounter>();


    // Initialize Rules
    // No rules yet
}

    public override void ScorerReset()
    {
        base.ScorerReset();

        red_auto = 0;
        blue_auto = 0;

        blue_winpoint = false;
        red_winpoint = false;
    }

    public override void Restart()
    {
        base.Restart();

        // If this game mode is option 2 in single player, then change timer to 1m
        if( GLOBALS.SINGLEPLAYER_MODE && GLOBALS.topsingleplayer && (GLOBALS.game_option==2))
        {
            GLOBALS.TIMER_TOTAL = 60;
            GLOBALS.TIMER_AUTO = 0;
        }
        else
        {
            GLOBALS.TIMER_TOTAL = 120;
            GLOBALS.TIMER_AUTO = 15;
        }

    }

    private int loadrings_delay = 0;

    private void LoadAllRings()
    {
        // Move rings to preload locations if applicable
        foreach (RobotInterface3D currbot in allrobots)
        {
            switch (currbot.myRobotID.starting_pos)
            {
                case "Red Left":
                    LoadStartingRings(currbot, preload_redl);
                    break;

                case "Red Right":
                    LoadStartingRings(currbot, preload_redr);
                    break;

                case "Blue Left":
                    LoadStartingRings(currbot, preload_bluel);
                    break;

                case "Blue Right":
                    LoadStartingRings(currbot, preload_bluer);
                    break;

            }
        }
    }

    public override void OnTimerStart()
    {
        // Mark to do load rings, except we have to do it on the next update to make sure robot got reset
        loadrings_delay = 2;

        // Initialize Triggers to be visible
        detect_bl.GetComponent<MeshRenderer>().enabled = true;
        detect_br.GetComponent<MeshRenderer>().enabled = true;
        detect_rl.GetComponent<MeshRenderer>().enabled = true;
        detect_rr.GetComponent<MeshRenderer>().enabled = true;
        blue_rings_returned = false;
        red_rings_returned = false;

        // Reset extra rings to be kinematic
        foreach (Transform curr_ring in extra_rings_red)
        {
            Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
            if (rb) { rb.isKinematic = true;  }
        }
        foreach (Transform curr_ring in extra_rings_blue)
        {
            Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
            if (rb) { rb.isKinematic = true; }
        }

        // Clear triggers just in case
        detect_bl.Clear();
        detect_br.Clear();
        detect_rl.Clear();
        detect_rr.Clear();
    }

    // Preload the starting rings
    private void LoadStartingRings( RobotInterface3D bot, Transform rings)
    {
        // Find the preload marker
        PreloadID marker = bot.GetComponentInChildren<PreloadID>();
        if( !marker ) { return; }

        // Marker found, now set rings position
        float i = 0f;
        foreach (Transform curr_ring in rings)
        {   
            curr_ring.SetPositionAndRotation(marker.transform.position + i * delta_ring.magnitude * marker.transform.up, marker.transform.rotation * curr_ring.rotation );
            i += 1f;
        }

        // rings.SetPositionAndRotation(marker.transform.position, marker.transform.rotation);

    }

    private Dictionary<int, gameElement> found_elements = new Dictionary<int, gameElement>();
    public override void ScorerUpdate(bool last_frame = false)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

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

        if( loadrings_delay > 0)
        {
            loadrings_delay -= 1;

            if( loadrings_delay == 0)
            {
                LoadAllRings();
            }
        }

     
    }

    // Function triggered when new robots show up on the field or are updated
    private List<RobotStates> robotstates = new List<RobotStates>();
    public override void FieldChangedTrigger()
    {

    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        data["ScoreR"] = ((int)score_red + score_redadj).ToString();
        data["ScoreB"] = ((int)score_blue + score_blueadj).ToString();

        data["RedWP"] = (red_winpoint) ? "1" : "0";
        data["BlueWP"] = (blue_winpoint) ? "1" : "0";

        data["AutoR"] = ((int)red_auto).ToString();
        data["AutoB"] = ((int)blue_auto).ToString();

        data["r_low_rings"] = ((int)rings_red_low).ToString();
        data["r_mid_rings"] = ((int)rings_red_mid).ToString();
        data["r_high_rings"] = ((int)rings_red_high).ToString();
        data["b_low_rings"] = ((int)rings_blue_low).ToString();
        data["b_mid_rings"] = ((int)rings_blue_mid).ToString();
        data["b_high_rings"] = ((int)rings_blue_high).ToString();

        data["r_goals"] = ((int)goals_red_scored).ToString();
        data["b_goals"] = ((int)goals_blue_scored).ToString();

        data["r_goals_bal"] = ((int)goals_red_balanced).ToString();
        data["b_goals_bal"] = ((int)goals_blue_balanced).ToString();

        data["r_bots_bal"] = ((int)robots_red_balanced).ToString();
        data["b_bots_bal"] = ((int)robots_blue_balanced).ToString();
    }

    public int red_auto = 0;
    public int blue_auto = 0;
    public bool blue_winpoint = false;
    public bool red_winpoint = false;

    public int rings_red_low = 0;
    public int rings_red_mid = 0;
    public int rings_red_high = 0;
    public int rings_blue_low = 0;
    public int rings_blue_mid = 0;
    public int rings_blue_high = 0;
    public int goals_red_balanced = 0;
    public int goals_blue_balanced = 0;
    public int robots_red_balanced = 0;
    public int robots_blue_balanced = 0;
    public int goals_red_scored = 0;
    public int goals_blue_scored = 0;

    public bool blue_balanced = false;
    public bool red_balanced = false;

    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if (timerstate != TimerState.RUNNING)
        { return; }

        // Make sure settings have loaded
        if (!tp_settings) { return; }

        // Calculate the total score of things so far that pertain to autonomouse (no balance yet)
        score_red = 0;
        score_blue = 0;
        rings_red_low = 0;
        rings_red_mid = 0;
        rings_red_high = 0;
        rings_blue_low = 0;
        rings_blue_mid = 0;
        rings_blue_high = 0;


        // Go through all the goals and count the rings
        // Note that the goal counter only stores goals valid in its region (e.g. red or blue)

        foreach (GoalScorer curr_goal in all_goals)
        {
            if (red_goal_counter.IsGoalInside(curr_goal) )
            {
                rings_red_low += curr_goal.GetLowCount();
                rings_red_mid += curr_goal.GetMidCount();
                rings_red_high += curr_goal.GetHighCount();
            }
            else if (blue_goal_counter.IsGoalInside(curr_goal) )
            {
                rings_blue_low += curr_goal.GetLowCount();
                rings_blue_mid += curr_goal.GetMidCount();
                rings_blue_high += curr_goal.GetHighCount();
            }
        }

        score_red += rings_red_low + 3 * rings_red_mid + 10 * rings_red_high;
        score_blue += rings_blue_low + 3 * rings_blue_mid + 10 * rings_blue_high;

        // Add the goals in our zones
        goals_red_scored = red_goal_counter.GetGoalCount();
        score_red += 20 * goals_red_scored;

        goals_blue_scored = blue_goal_counter.GetGoalCount();
        score_blue += 20 * goals_blue_scored;

        // Determine auto score
        if (!GLOBALS.SINGLEPLAYER_MODE && (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)))
        {
            red_auto = 0;
            blue_auto = 0;
            if (score_red > score_blue) { red_auto = 6; }
            else if (score_blue > score_red) { blue_auto = 6; }
            else
            {
                red_auto = 3;
                blue_auto = 3;
            }
        }

        // Add auto back into score
        score_red += red_auto;
        score_blue += blue_auto;

        // ****************************************************
        // Now determine if platform is balanced
        // Only after auto
        // ****************************************************
        // 1) Check platform isn't touching floor
        // 2) Check robot inside platform isn't touching floor
        // 3) Check Goal inside platform isn't touching floor
        // 4) Check robot colliding with platform isn't touching floor
        //    (if a robot outside is touching underneath platform, this is a problem)
        goals_red_balanced = 0;
        goals_blue_balanced = 0;
        robots_red_balanced = 0;
        robots_blue_balanced = 0;

        if (!GLOBALS.SINGLEPLAYER_MODE && (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)))
        {
            return;
        }
            blue_balanced = true;
        red_balanced = true;

        // If platform is contacting floor, invalidate
        if (bd_plat_red.GetPlatformCount() > 0)
        { red_balanced = false; }

        if (bd_plat_blue.GetPlatformCount() > 0)
        { blue_balanced = false; }

        // If robot on platform is also touching floor, invalidate
        if (red_balanced)
        {
            foreach (RobotID currbot in Balance_Robot_Counter_Red.collisions)
            {
                if (bd_robot_red.collisions.Contains(currbot))
                {
                    red_balanced = false;
                    break;
                }
            }
        }

        if (blue_balanced)
        {
            foreach (RobotID currbot in Balance_Robot_Counter_Blue.collisions)
            {
                if (bd_robot_blue.collisions.Contains(currbot))
                {
                    blue_balanced = false;
                    break;
                }
            }
        }

        // If goal on platform is touching floor, invalidate
        if (red_balanced)
        {
            foreach (GoalScorer currgoal in Balance_Goal_Counter_Red.collisions)
            {
                if (bd_goal_red.collisions.Contains(currgoal))
                {
                    red_balanced = false;
                    break;
                }
            }
        }

        if (blue_balanced)
        {
            foreach (GoalScorer currgoal in Balance_Goal_Counter_Blue.collisions)
            {
                if (bd_goal_blue.collisions.Contains(currgoal))
                {
                    blue_balanced = false;
                    break;
                }
            }
        }

        // If robot colliding with platform is touching floor, invalidate
        if (red_balanced)
        {
            foreach (RobotID currbot in Balance_Robot_Counter_Red.collisions)
            {
                if (bd_robot_red.collisions.Contains(currbot))
                {
                    red_balanced = false;
                    break;
                }
            }
        }

        if (blue_balanced)
        {
            foreach (RobotID currbot in Balance_Robot_Counter_Blue.collisions)
            {
                if (bd_robot_blue.collisions.Contains(currbot))
                {
                    red_balanced = false;
                    break;
                }
            }
        }

        // We now know balanced states
        // Add points in

        if( red_balanced )
        {
            goals_red_balanced = Balance_Goal_Counter_Red.GetGoalCount();
            robots_red_balanced = Balance_Robot_Counter_Red.GetRobotCount();
        }

        if (blue_balanced)
        {
            goals_blue_balanced = Balance_Goal_Counter_Blue.GetGoalCount();
            robots_blue_balanced = Balance_Robot_Counter_Blue.GetRobotCount();
        }
        score_red += goals_red_balanced * 20 + robots_red_balanced * 30;
        score_blue += goals_blue_balanced * 20 + robots_blue_balanced * 30;

        // Return rings if applicable
        ReturnRings();

        // Determine win-point
        //if (time_total.TotalSeconds >= (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO))
        //{
        //    blue_winpoint = line_r6.enabled && (tr == 2);
        //    red_winpoint = line_r4.enabled && (tl == 1);
        //}

        // Now correct score for skills challenge
        //if( GLOBALS.SINGLEPLAYER_MODE && GLOBALS.topsingleplayer && GLOBALS.game_option==3)
        //{
        //    score_red = score_red - score_blue + 63;
        //    score_blue = 0;
        //}
    }

    private Vector3 delta_ring = new Vector3(0, 0.12f, 0);
    public void ReturnRings()
    {
        // If blue hasn't been returned, check
        if(!blue_rings_returned && detect_bl.AreFriendsColliding())
        {
            blue_rings_returned = true;
            detect_bl.GetComponent<MeshRenderer>().enabled = false;
            detect_br.GetComponent<MeshRenderer>().enabled = false;
            float i = 0;
            foreach(Transform curr_ring in extra_rings_blue)
            {
                Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
                if( rb ) { 
                    rb.isKinematic = false;
                    curr_ring.SetPositionAndRotation(ringreturn_bl.position + i * delta_ring, ringreturn_bl.rotation * curr_ring.rotation);
                    i += 1f;
                }             
            }
        }

        if (!blue_rings_returned && detect_br.AreFriendsColliding())
        {
            blue_rings_returned = true;
            detect_bl.GetComponent<MeshRenderer>().enabled = false;
            detect_br.GetComponent<MeshRenderer>().enabled = false;
            float i = 0;
            foreach (Transform curr_ring in extra_rings_blue)
            {
                Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
                if (rb)
                {
                    rb.isKinematic = false;
                    curr_ring.SetPositionAndRotation(ringreturn_br.position + i * delta_ring, ringreturn_br.rotation * curr_ring.rotation);
                    i += 1f;
                }
            }
        }

        // Now do red
        if (!red_rings_returned && detect_rl.AreFriendsColliding())
        {
            red_rings_returned = true;
            detect_rl.GetComponent<MeshRenderer>().enabled = false;
            detect_rr.GetComponent<MeshRenderer>().enabled = false;
            float i = 0;
            foreach (Transform curr_ring in extra_rings_red)
            {
                Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
                if (rb)
                {
                    rb.isKinematic = false;
                    curr_ring.SetPositionAndRotation(ringreturn_rl.position + i * delta_ring, ringreturn_rl.rotation * curr_ring.rotation);
                    i += 1f;
                }
            }
        }

        if (!red_rings_returned && detect_rr.AreFriendsColliding())
        {
            red_rings_returned = true;
            detect_rl.GetComponent<MeshRenderer>().enabled = false;
            detect_rr.GetComponent<MeshRenderer>().enabled = false;
            float i = 0;
            foreach (Transform curr_ring in extra_rings_red)
            {
                Rigidbody rb = curr_ring.GetComponent<Rigidbody>();
                if (rb)
                {
                    rb.isKinematic = false;
                    curr_ring.SetPositionAndRotation(ringreturn_rr.position + i * delta_ring, ringreturn_rr.rotation * curr_ring.rotation);
                    i += 1f;
                }
            }
        }

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




    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

        // Blue and Red ringreturn panels
        serverFlags["ST_RR"] = ((blue_rings_returned) ? "1" : "0") + ":" + ((red_rings_returned) ? "1" : "0");
    }

    // Receiveflags from server
    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

        // Get the blue/red rings returned states
        bool old_blue = blue_rings_returned;
        bool old_red = red_rings_returned;
        if (serverFlags.ContainsKey("ST_RR"))
        {
            string instring = serverFlags["ST_RR"];
            if (instring.Length >= 3)
            {
                blue_rings_returned = instring[0] == '1';
                red_rings_returned = instring[2] == '1';
            }
        }

        // Set the state of blue/red indicator
        if( (old_blue != blue_rings_returned) || (old_red != red_rings_returned))
        {
            detect_bl.GetComponent<MeshRenderer>().enabled = !blue_rings_returned;
            detect_br.GetComponent<MeshRenderer>().enabled = !blue_rings_returned;
            detect_rl.GetComponent<MeshRenderer>().enabled = !red_rings_returned;
            detect_rr.GetComponent<MeshRenderer>().enabled = !red_rings_returned;

        }



    }
}