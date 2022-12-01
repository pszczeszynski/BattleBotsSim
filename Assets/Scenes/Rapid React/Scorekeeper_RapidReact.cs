using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_RapidReact : Scorekeeper {

    // Game specific settings
    RapidReact_Settings rr_settings = null;

    // Tarmac links
    public List<TarmacTracker> tarmac_list = new List<TarmacTracker>();

    // Top Goal scorers
    public List<RR_GoalScorer> top_scorers = new List<RR_GoalScorer>();
    public List<RR_GoalScorer> bot_scorers = new List<RR_GoalScorer>();
    public RR_GoalScorer top_hub;
    public RR_GoalScorer bot_hub;

    // Pushback center
    public GameObject pushback;

    // Hanging scorers
    public GenericFieldTracker red_low_bar;
    public GenericFieldTracker red_mid_bar;
    public GenericFieldTracker red_high_bar;
    public GenericFieldTracker red_ultrahigh_bar;
    public GenericFieldTracker red_floor;

    public GenericFieldTracker blue_low_bar;
    public GenericFieldTracker blue_mid_bar;
    public GenericFieldTracker blue_high_bar;
    public GenericFieldTracker blue_ultrahigh_bar;
    public GenericFieldTracker blue_floor;

    // Foul trackers
    public GenericFieldTracker lp_red1;
    public GenericFieldTracker lp_red2;
    public GenericFieldTracker lp_blue1;
    public GenericFieldTracker lp_blue2;
    public GenericFieldTracker hanger_red;
    public GenericFieldTracker hanger_blue;
    public GenericFieldTracker WallPins;

    // Preload blocks
    public Transform preload_redl;
    public Transform preload_redc;
    public Transform preload_redr;
    public Transform preload_bluel;
    public Transform preload_bluec;
    public Transform preload_bluer;

    // Score details
    public double penalties_red; // additional points given to ref from opposing teams penalties
    public int cargo_red;
    public double score_auto_red;
    public double score_teleop_red;
    public double score_endgame_red;
    public double score_red;

    public double penalties_blue; // additional points given to ref from opposing teams penalties

    public int cargo_high_blue = 0;
    public int auto_cargo_high_blue = 0;
    public int cargo_low_blue = 0;
    public int auto_cargo_low_blue = 0;
    public int cargo_high_red = 0;
    public int auto_cargo_high_red = 0;
    public int cargo_low_red = 0;
    public int auto_cargo_low_red = 0;

    public int hang_low_red = 0;
    public int hang_mid_red = 0;
    public int hang_high_red = 0;
    public int hang_traverse_red = 0;

    public int hang_low_blue = 0;
    public int hang_mid_blue = 0;
    public int hang_high_blue = 0;
    public int hang_traverse_blue = 0;

    public int taxi_red = 0;
    public int taxi_blue = 0;

    public double score_auto_blue;
    public double score_teleop_blue;
    public double score_endgame_blue;
    public double score_blue;

    
    public GameObject powerup_overlay;

    [Header("PowerUps")]
    public PowerUpScript powerup_center;
    public List<PowerUpScript> powerup_home = new List<PowerUpScript>();

 
    // My own OPR temporary placeholder
    public Dictionary<string, float> player_opr_endgame = new Dictionary<string, float>();

    private List<int> hang_awarded_foul = new List<int>();

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
        public long LP_StartTime = -1;
        public bool immune_to_foul = false;

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
            LP_StartTime = -1;
            immune_to_foul = false;
            myoffensive_powerups.Clear();
            mydeffensive_powerups.Clear();

            if ( robotID)
            {
                robotID.RemoveData("Taxi");
            }
   
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

        rr_settings = GameObject.Find("GameSettings").GetComponent<RapidReact_Settings>();
        fault_prefab = Resources.Load("Prefabs/FaultAnimation") as GameObject;

        // Initialize powerups
        powerup_center.myscorekeeper = this;
        foreach( PowerUpScript currpu in powerup_home)
        {
            currpu.myscorekeeper = this;
        }
    }

    private string mark_string = "";

    public override void ScorerReset()
    {
        base.ScorerReset();

        penalties_red = 0;// additional points given to ref from opposing teams penalties
        cargo_red = 0;
        score_auto_red = 0;
        score_teleop_red = 0;
        score_endgame_red = 0;
        score_red = 0;
        taxi_red = 0;
        taxi_blue = 0;

        penalties_blue = 0; // additional points given to ref from opposing teams penalties
        score_auto_blue = 0;
        score_teleop_blue = 0;
        score_endgame_blue = 0;
        score_blue = 0;


        cargo_high_blue = 0;
        auto_cargo_high_blue = 0;
        cargo_low_blue = 0;
        auto_cargo_low_blue = 0;
        cargo_high_red = 0;
        auto_cargo_high_red = 0;
        cargo_low_red = 0;
        auto_cargo_low_red = 0;
        hang_awarded_foul.Clear();

        mark_string = "";

        player_opr_endgame.Clear();
        foreach (string currbot in allrobots_byname.Keys)
        {
            player_opr_endgame[currbot] = 0;
        }


        ResetRobotStates();
        ResetPowerUps();
        ResetBalls();

        foreach( RR_GoalScorer currscorer in top_scorers)
        {
            currscorer.Reset();
        }

        top_hub.Reset();

        foreach (RR_GoalScorer currscorer in bot_scorers)
        {
            currscorer.Reset();
        }
        bot_hub.Reset();
    }

    public override void OnTimerStart()
    {
        base.OnTimerStart();
        start_counter = 2;
        preload_delay = 2;
    }

    public void MyTimerStart()
    {
        // Make Tarmacs remember who starts in them
        foreach( TarmacTracker currtarmac in tarmac_list)
        {
            currtarmac.MarkStartingRobots();
        }

        start_counter = -1;
    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        // Add this game specific data
        data["AutoR"] = ((int)score_auto_red).ToString();
        data["AutoB"] = ((int)score_auto_blue).ToString();
        data["TeleR"] = ((int)score_teleop_red).ToString();
        data["TeleB"] = ((int)score_teleop_blue).ToString();
        data["EndR"] = ((int)score_endgame_red).ToString();
        data["EndB"] = ((int)score_endgame_blue).ToString();
        data["ScoreR"] = ((int)score_red + score_redadj).ToString();
        data["ScoreB"] = ((int)score_blue + score_blueadj).ToString();
        data["PenB"] = ((int)penalties_blue).ToString();
        data["PenR"] = ((int)penalties_red).ToString();

        // Additional Data
        data["Auto_Taxi_R"] = taxi_red.ToString();
        data["Auto_Taxi_B"] = taxi_blue.ToString();
        data["Auto_C_H_R"] = auto_cargo_high_red.ToString();
        data["Auto_C_L_R"] = auto_cargo_low_red.ToString();
        data["Auto_C_H_B"] = auto_cargo_high_blue.ToString();
        data["Auto_C_L_B"] = auto_cargo_low_blue.ToString();
        data["C_H_R"] = cargo_high_red.ToString();
        data["C_L_R"] = cargo_low_red.ToString();
        data["C_H_B"] = cargo_high_blue.ToString();
        data["C_L_B"] = cargo_low_blue.ToString();

        data["H_L_B"] = hang_low_blue.ToString();
        data["H_L_R"] = hang_low_red.ToString();
        data["H_M_B"] = hang_mid_blue.ToString();
        data["H_M_R"] = hang_mid_red.ToString();
        data["H_H_B"] = hang_high_blue.ToString();
        data["H_H_R"] = hang_high_red.ToString();
        data["H_T_B"] = hang_traverse_blue.ToString();
        data["H_T_R"] = hang_traverse_red.ToString();

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
            "    # Taxi: " + details["Auto_Taxi_" + team] + "\n" +
            "    # Cargo in Low: " + details["Auto_C_L_" + team] + "\n" +
            "    # Cargo in High: " + details["Auto_C_H_" + team] + "\n" +
            "\n" +
            "<B>TELEOP Score =</B> " + details["Tele" + team] + "\n" +
            "    # Cargo in Low: " + details["C_L_" + team] + "\n" +
            "    # Cargo in High: " + details["C_H_" + team] + "\n" +

            "\n" +
            "<B>ENDGAME Score=</B> " + details["End" + team] + "\n" +
            "    # Hang Low: " + details["H_L_" + team] + "\n" +
            "    # Hang Mid: " + details["H_M_" + team] + "\n" +
            "    # Hang High: " + details["H_H_" + team] + "\n" +
            "    # Hang Traverse: " + details["H_T_" + team] + "\n";
    }

    public override void Restart()
    {
        base.Restart();

    }



    // Deal with power ups

    void UpdatePowerUps()
    {
        // Don't do anything in client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        // Go through robots and release any power ups
        foreach (RobotStates currbot in allrobotstates)
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
            ServicePowerUp(current_pu, rr_settings.PU_HOME, rr_settings.PU_HOME_TYPE);
        }

        // Service center power-up
        ServicePowerUp(powerup_center, rr_settings.PU_CENTER, rr_settings.PU_CENTER_TYPE);

        // Update robots for their powerups
        // Go through robots and release any power ups
        foreach (RobotStates currbot in allrobotstates)
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
                robot.TweakPerformance(1f + (rr_settings.PU_STRENGTH/100f), 1f); // 
                break;
            case PowerUpType.TORQUE:
                robot.TweakPerformance(1f, 1f + 3f* (rr_settings.PU_STRENGTH / 100f)); // Need to reduce full speed a bit since friction/breaking will no longer slow us down as much
                break;
            case PowerUpType.INVISIBILITY:
                robot.TurnOffRenderers(true);
                break;
            case PowerUpType.WEAK:
                robot.TweakPerformance(1f, 1f /(1f + (rr_settings.PU_STRENGTH / 100f)));
                break;
            case PowerUpType.SLOW:
                robot.TweakPerformance(1f / (1f + (rr_settings.PU_STRENGTH / 100f)), 1f);
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
                    if( rr_settings.PU_ONLYONE && (myrobot.myoffensive_powerups.Count >= 1)) { break; }
                    newpu.duration = rr_settings.PU_OFFENSIVE_DURATION * 1000;
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
                    newpu.duration = rr_settings.PU_DEFENSIVE_DURATION * 1000;
                    foreach (RobotStates currbot in allrobotstates)
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
        if (  rr_settings.ENABLE_POWERUPS && isenabled)
        {
            if (current_pu.IsDisabled() && (MyUtils.GetTimeMillisSinceStart() - current_pu.GetTimeStarted()) > rr_settings.PU_RESPAWN*1000)
            {

                // Get a list of allowed poewr-ups
                List<PowerUpType> allowed_types = new List<PowerUpType>();

                // type_allowed: 0 = offensive, 1 = defensive, 2 = both
                if ((type_allowed == 0) || (type_allowed == 2))
                {
                    if (rr_settings.PU_SPEED) { allowed_types.Add(PowerUpType.SPEED); }
                    if (rr_settings.PU_TORQUE) { allowed_types.Add(PowerUpType.TORQUE); }
                    if (rr_settings.PU_INVISIBLE) { allowed_types.Add(PowerUpType.INVISIBILITY); }
                }

                if ((type_allowed == 1) || (type_allowed == 2))
                {
                    if (rr_settings.PU_SLOW) { allowed_types.Add(PowerUpType.SLOW); }
                    if (rr_settings.PU_WEAK) { allowed_types.Add(PowerUpType.WEAK); }
                    if (rr_settings.PU_INVERTED) { allowed_types.Add(PowerUpType.INVERTED); }
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
        if( (thisrobot.myoffensive_powerups.Count > 0) && rr_settings.PU_ONLYONE && ((int) thepu.myPower) < ((int) PowerUpType.SLOW ))
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

    private int preload_delay = 0;
    public override void ScorerUpdate(bool last_frame = false)
    {
        // I think we can ignore everything here in client mode
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
                pushback.SetActive(rr_settings.ENABLE_AUTO_PUSHBACK);
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

       
        // Do fouls
        if (rr_settings.ENABLE_LP_FOUL && game_running)
        {
            Do_LaunchPads();
        }

        Do_HangFoul();

        UpdatePowerUps();
        CalculateScores(last_frame);

        // Nothign left to do if in client mode
        if( GLOBALS.CLIENT_MODE) { return; }

        Do_Blocking();



    }


    public override void RobotCounterExpired(RobotID bot)
    {
        // Do nothing because we do it in the main loop
    }
  
    public void Do_Blocking()
    {
        // See if game is running
        bool game_running = true;
        if (base.timerstate != TimerState.PAUSED &&
            base.timerstate != TimerState.RUNNING
            )
        {
            game_running = false;
        }

        // First reset all robot blocking variable
        int index = allrobotstates.Count;
        while (index >= 1)
        {
            index--;
            allrobotstates[index].isblocking = false;
        }

        // Go through every bot and its enemies and see if a valid blocking or interference situation occured
        index = allrobotstates.Count;
        float time = Time.time;

        while (index >= 1)
        {
            // Decrement index
            index--;

            RobotStates currbot = allrobotstates[index];

            // Skip this bot if it's been destroyed
            if (!currbot.robot) { continue; }

            // If wall pins are disabled, then clear robot counting
            if(!rr_settings.ENABLE_WALLPINS)
            {
                if (currbot.start_time != -1f)
                {
                    currbot.counting_up = false;
                    currbot.counting_down = false;
                    currbot.robot.SetProgressBar(0);
                }

                continue;
            }

            // ********* Update Progress Bars **********
            // Deal with counting down
            if (currbot.counting_down ) 
            {
                //  Check if timer expired
                if (time - currbot.start_time > rr_settings.BLOCKING_DURATION)
                {
                    currbot.counting_down = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((rr_settings.BLOCKING_DURATION - time + currbot.start_time) / rr_settings.BLOCKING_DURATION);
                }
            }

            // *** If counting up, see if it expired and/or update progress bar
            // *** Changing counting-up to down if not blocking done later
            if (currbot.counting_up) 
            {
                //  Check if timer expired
                if (time - currbot.start_time > rr_settings.BLOCKING_DURATION)
                {

                    // restart count up
                    currbot.start_time = time;
                    currbot.robot.SetProgressBar(0);
                    
                    if (game_running)
                    {
                        // Increment the OPR of the robots colliding
                        List<RobotInterface3D> enemies = currbot.robot.GetAllEnemies();
                        foreach (RobotInterface3D enemybot in enemies)
                        {
                            if (player_opr.ContainsKey(GLOBALS.client_names[enemybot.myRobotID.id]))
                            {
                                player_opr[GLOBALS.client_names[enemybot.myRobotID.id]] += rr_settings.PENALTY_BLOCKING / enemies.Count;
                            }
                            else
                            {
                                player_opr[GLOBALS.client_names[enemybot.myRobotID.id]] = rr_settings.PENALTY_BLOCKING / enemies.Count;
                            }

                        }

                        if (currbot.isRed) { penalties_blue += rr_settings.PENALTY_BLOCKING; }
                        else { penalties_red += rr_settings.PENALTY_BLOCKING; }

                        // Create penalty graphics
                        CreateFoul(currbot.robotID.id, false);
                    }
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((time - currbot.start_time) / rr_settings.BLOCKING_DURATION);
                }

            }

            // Now determine if blocking should be enabled for the robot
            // Check for a valid blocking situation
            // Firstly if this robot is not in safe zone, don't do anything
            if( !WallPins.IsRobotInside(currbot.robotID)) { continue; }
            
            foreach (RobotInterface3D enemybot in currbot.robot.GetAllEnemies())
            {
                // Get enemies state
                int enemyindex = GetRobotStateIndex(enemybot);
                if (enemyindex < 0) { continue; }
                RobotStates enemystate = allrobotstates[enemyindex];

                // If we are inside our safety zone more then the enemy, then count enemy up
                if (!WallPins.IsRobotInside(enemystate.robotID) ||
                    (WallPins.GetClosestDistance(currbot.robot.rb_body.transform.position) + 0.1f < WallPins.GetClosestDistance(enemybot.rb_body.transform.position))
                    )
                {
                    enemystate.isblocking = true;

                    if (enemystate.counting_down)
                    {
                        enemystate.counting_down = false;
                        enemystate.counting_up = true;
                        enemystate.start_time = time - (rr_settings.BLOCKING_DURATION - (time - enemystate.start_time));
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
        index = allrobotstates.Count;
        while (index >= 1)
        {
            index--;

            if (!allrobotstates[index].isblocking && allrobotstates[index].counting_up)
            {
                allrobotstates[index].counting_up = false;
                allrobotstates[index].counting_down = true;
                allrobotstates[index].start_time = time - (rr_settings.BLOCKING_DURATION - (time - allrobotstates[index].start_time));
            }
        }

    }




    // Instantiates the foul graphics
    string foul_string = "";

    private void CreateFoul( int robotid, bool change_color = true)
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
        if (change_color) { robot.OverrideColor(0); }

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

    private void MarkRobotForHang(int robotid, bool mark)
    {      
        // Get the robot
        if (!allrobots_byid.ContainsKey(robotid)) { return; }

        RobotInterface3D robot = allrobots_byid[robotid];
        if (!robot.rb_body) { return; }

        // Change the robot's color to foul/holding or return it to normal
        robot.OverrideColor((mark) ? 3 : -1);

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

    

    // Function triggered when new robots show up on the field or are updated
    private List<RobotStates> allrobotstates = new List<RobotStates>();
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
                newstates.Add(allrobotstates[robotstateindex]);
            }
        }

        // Assign robotstates to the list
        allrobotstates = newstates;
    }

    void ResetRobotStates()
    {
        foreach( RobotStates currbot in allrobotstates)
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

    void ResetBalls()
    {
        ball_data[] all_balls = GameObject.FindObjectsOfType<ball_data>();

        foreach( ball_data currball in all_balls)
        {
            currball.Clear();
        }
    }

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

                case "Red Center":
                    PreloadItem(currbot, preload_redc);
                    break;

                case "Red Right":
                    PreloadItem(currbot, preload_redr);
                    break;

                case "Blue Left":
                    PreloadItem(currbot, preload_bluel);
                    break;

                case "Blue Center":
                    PreloadItem(currbot, preload_bluec);
                    break;

                case "Blue Right":
                    PreloadItem(currbot, preload_bluer);
                    break;
            }
        }
    }


    private void PreloadItem(RobotInterface3D bot, Transform preload_items)
    {
        // Find the preload marker
        PreloadID marker = bot.GetComponentInChildren<PreloadID>();
        if (!marker) { return; }

        // Marker found, now set ball positions
        preload_items.SetPositionAndRotation(marker.transform.position, marker.transform.rotation * preload_items.rotation);
    }



    // Returns the robot state index for the passed robot
    // Since # of robots is max 6, we will do sequential search which is comparable in time to a hash lookup.
    private int GetRobotStateIndex( RobotInterface3D robot)
    {
        for (int index = 0; index < allrobotstates.Count; index++)
        {
            if( allrobotstates[index].robot == robot )
            { return index;  }
        }

        return -1;
    }

    private RobotStates GetRobotState(RobotInterface3D robot)
    {
        for (int index = 0; index < allrobotstates.Count; index++)
        {
            if (allrobotstates[index].robot == robot)
            { return allrobotstates[index]; }
        }

        return null;

    }

    private RobotStates GetRobotStateById(int id)
    {
        if( !allrobots_byid.ContainsKey(id)) { return null;  }
        return GetRobotState(allrobots_byid[id]);
    }

    private void AddToPlayerOPR( List<ball_data> allballs, float multiplier, bool is_red)
    {
        foreach( ball_data currball in allballs)
        {
            // If this came from an enemy robot or no bot, don't count it
            if((currball.thrown_robotid==null) || (currball.thrown_robotid.is_red != is_red) ) { continue; }

            int id = currball.thrown_by_id;

            // Add the score to our robot
            if(player_opr.ContainsKey(GLOBALS.client_names[id])) { player_opr[GLOBALS.client_names[id]] += multiplier; }
            else { player_opr[GLOBALS.client_names[id]] = multiplier;  }
        }
    }

    private void AddToPlayerOPR(int id, float amount)
    {
        if (player_opr.ContainsKey(GLOBALS.client_names[id])) { player_opr[GLOBALS.client_names[id]] += amount; }
        else { player_opr[GLOBALS.client_names[id]] = amount; }
    }

    private int start_counter = 0;
    private void CalculateScores(bool last_frame = false)
    {
        // Only calculate while timer is running
        if (timerstate == TimerState.RUNNING)
        {
            // We need to wait an update cycle to allow all callbacks to be processed before letting tarmacs remember who's starting in them.
            if (start_counter >= 0)
            {
                start_counter -= 1;
                if (start_counter == 0)
                {
                    MyTimerStart();
                }
            }

            // Auto tarmac
            if (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO))
            {
                // Go through each robot and check if it exited their tarmac was exited
                foreach (RobotID currbot in allRobotID.Values)
                {
                    if (currbot.GetUserBool("Taxi") && !currbot.GetUserBool("TaxiCounted"))
                    {
                        AddToPlayerOPR(currbot.id, 2);
                        currbot.SetUserBool("TaxiCounted", true);

                        if (currbot.is_red)
                        {
                            taxi_red += 1;
                            score_auto_red += 2f; 
                        }
                        else
                        {
                            taxi_blue += 1;
                            score_auto_blue += 2f; 
                        }
                    }
                }
            }

            // Auto Balls
            if (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO - 5f))
            {

                // Go through each top scorer
                foreach (RR_GoalScorer currscorer in top_scorers)
                {
                    auto_cargo_high_blue += currscorer.GetblueBallCount();
                    score_auto_blue += 4f * currscorer.GetblueBallCount();

                    auto_cargo_high_red += currscorer.GetRedBallCount();
                    score_auto_red += 4f * currscorer.GetRedBallCount();

                    AddToPlayerOPR(currscorer.red_balls, 4f, true);
                    AddToPlayerOPR(currscorer.blue_balls, 4f, false);
                    currscorer.ClearData();
                }

                // Go through each bot scorer
                foreach (RR_GoalScorer currscorer in bot_scorers)
                {
                    auto_cargo_low_blue += currscorer.GetblueBallCount();
                    score_auto_blue += 2f * currscorer.GetblueBallCount();

                    auto_cargo_low_red += currscorer.GetRedBallCount();
                    score_auto_red += 2f * currscorer.GetRedBallCount();

                    AddToPlayerOPR(currscorer.red_balls, 2f, true);
                    AddToPlayerOPR(currscorer.blue_balls, 2f, false);
                    currscorer.ClearData();
                }
            }

            //Teleop balls
            else
            {
                // Add the top hub is last frame
                if (last_frame)
                {
                    top_hub.ResetCountToBallsInside();
                    bot_hub.ResetCountToBallsInside();

                    // **** Top Hub Last Frame
                    List<ball_data> blue_balls_processed = new List<ball_data>();
                    List<ball_data> red_balls_processed = new List<ball_data>();

                    // Remeber balls were processed so we remove it from double consideration
                    foreach (ball_data currball in top_hub.blue_balls)
                    {
                        blue_balls_processed.Add(currball);
                    }
                    foreach (ball_data currball in top_hub.red_balls)
                    {
                        red_balls_processed.Add(currball);
                    }

                    cargo_high_blue += top_hub.GetblueBallCount();
                    score_teleop_blue += 2f * top_hub.GetblueBallCount();
                    cargo_high_red += top_hub.GetRedBallCount();
                    score_teleop_red += 2f * top_hub.GetRedBallCount();

                    AddToPlayerOPR(top_hub.red_balls, 2f, true);
                    AddToPlayerOPR(top_hub.blue_balls, 2f, false);

                    // Go through each top scorer and remove any balls that were counted
                    foreach (RR_GoalScorer currscorer in top_scorers)
                    {
                        // Remove last-frame balls
                        foreach (ball_data currball in blue_balls_processed)
                        {
                            currscorer.blue_balls.Remove(currball);
                        }
                        foreach (ball_data currball in red_balls_processed)
                        {
                            currscorer.red_balls.Remove(currball);
                        }
                    }

                    // **** Bot Hub Last Frame
                    blue_balls_processed.Clear();
                    red_balls_processed.Clear();

                    // Remeber balls were processed so we remove it from double consideration
                    foreach (ball_data currball in bot_hub.blue_balls)
                    {
                        blue_balls_processed.Add(currball);
                    }
                    foreach (ball_data currball in bot_hub.red_balls)
                    {
                        red_balls_processed.Add(currball);
                    }

                    cargo_low_blue += bot_hub.GetblueBallCount();
                    score_teleop_blue += 1f * bot_hub.GetblueBallCount();
                    cargo_low_red += bot_hub.GetRedBallCount();
                    score_teleop_red += 1f * bot_hub.GetRedBallCount();

                    AddToPlayerOPR(bot_hub.red_balls, 1f, true);
                    AddToPlayerOPR(bot_hub.blue_balls, 1f, false);

                    // Go through each top scorer and remove any balls that were counted
                    foreach (RR_GoalScorer currscorer in bot_scorers)
                    {
                        // Remove last-frame balls
                        foreach (ball_data currball in blue_balls_processed)
                        {
                            currscorer.blue_balls.Remove(currball);
                        }
                        foreach (ball_data currball in red_balls_processed)
                        {
                            currscorer.red_balls.Remove(currball);
                        }
                    }
                }

                // Go through each top scorer
                foreach (RR_GoalScorer currscorer in top_scorers)
                {
                    cargo_high_blue += currscorer.GetblueBallCount();
                    score_teleop_blue += 2f * currscorer.GetblueBallCount();
                    cargo_high_red += currscorer.GetRedBallCount();
                    score_teleop_red += 2f * currscorer.GetRedBallCount();

                    AddToPlayerOPR(currscorer.red_balls, 2f, true);
                    AddToPlayerOPR(currscorer.blue_balls, 2f, false);
                    currscorer.ClearData();
                }

                // Go through each bot scorer
                foreach (RR_GoalScorer currscorer in bot_scorers)
                {
                    cargo_low_blue += currscorer.GetblueBallCount();
                    score_teleop_blue += 1f * currscorer.GetblueBallCount();
                    cargo_low_red += currscorer.GetRedBallCount();
                    score_teleop_red += 1f * currscorer.GetRedBallCount();

                    AddToPlayerOPR(currscorer.red_balls, 1f, true);
                    AddToPlayerOPR(currscorer.blue_balls, 1f, false);
                    currscorer.ClearData();
                }
            }


            score_endgame_red = 0;
            score_endgame_blue = 0;

            // End-Game
            player_opr_endgame.Clear();

            // Get endgame hangs
            hang_low_red = 0;
            hang_mid_red = 0;
            hang_high_red = 0;
            hang_traverse_red = 0;


            hang_low_blue = 0;
            hang_mid_blue = 0;
            hang_high_blue = 0;
            hang_traverse_blue = 0;

            // Go through every bot
            foreach( RobotID currbot in allRobotID.Values )
            {
                if( currbot == null) { continue; }
                // do red bot first
                if( currbot.is_red)
                {
                    // If robot is awarded hang, give it to them
                    RobotStates thisbotstate = GetRobotStateById(currbot.id);
                    if( thisbotstate == null ) { continue; }
                    if( hang_awarded_foul.Contains(currbot.id))
                    {
                        hang_traverse_red += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 15f;
                        continue;
                    }

                    // If touching floor, no points
                    if( red_floor.IsRobotInside(currbot)) { continue;  }

                    // Check bars in order
                    if( red_low_bar.IsRobotInside(currbot))
                    {
                        hang_low_red += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 4f;
                        continue;
                    }
                    if (red_mid_bar.IsRobotInside(currbot))
                    {
                        hang_mid_red += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 6f;
                        continue;
                    }
                    if (red_high_bar.IsRobotInside(currbot))
                    {
                        hang_high_red += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 10f;
                        continue;
                    }
                    if (red_ultrahigh_bar.IsRobotInside(currbot))
                    {
                        hang_traverse_red += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 15f;
                        continue;
                    }
                }

                // Do blue bot
                else
                {
                    // If robot is awarded hang, give it to them
                    RobotStates thisbotstate = GetRobotStateById(currbot.id);
                    if (thisbotstate == null) { continue; }
                    if (hang_awarded_foul.Contains(currbot.id))
                    {
                        hang_traverse_blue += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 15f;
                        continue;
                    }

                    // If touching floor, no points
                    if (blue_floor.IsRobotInside(currbot)) { continue; }

                    // Check bars in order
                    if (blue_low_bar.IsRobotInside(currbot))
                    {
                        hang_low_blue += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 4f;
                        continue;
                    }
                    if (blue_mid_bar.IsRobotInside(currbot))
                    {
                        hang_mid_blue += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 6f;
                        continue;
                    }
                    if (blue_high_bar.IsRobotInside(currbot))
                    {
                        hang_high_blue += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 10f;
                        continue;
                    }
                    if (blue_ultrahigh_bar.IsRobotInside(currbot))
                    {
                        hang_traverse_blue += 1;
                        player_opr_endgame[GLOBALS.client_names[currbot.id]] = 15f;
                        continue;
                    }
                }
            }

            // Do possesion limit
            if (rr_settings.ENABLE_POSSESSION_LIMIT)
            {
                foreach (RobotID currbot in allRobotID.Values)
                {
                    if (!currbot) { continue; }

                    // Check if possesion detection is present
                    PossessionDetect2 curr_detector = currbot.GetComponent<PossessionDetect2>();
                    if (!curr_detector) { continue; }

                    // Make sure the limit number is set correctly (robots can appear mid-game, thus well just set the value every frame here)
                    curr_detector.limit_num = 2;

                    // Check if limit exceeded (3s grace period)
                    int num_of_faults = curr_detector.GetFaultCount(3f); // This auto resets the faults
                    if (num_of_faults > 0)
                    {
                        if (currbot.is_red) { penalties_blue += num_of_faults * rr_settings.POSSESSION_PENALTY; }
                        else { penalties_red += num_of_faults * rr_settings.POSSESSION_PENALTY; }

                        CreateFoul(currbot.id, false);
                    }

                }
            }


            score_endgame_red = 4*hang_low_red + 6*hang_mid_red + 10*hang_high_red + 15*hang_traverse_red;
            score_endgame_blue = 4 * hang_low_blue + 6 * hang_mid_blue + 10 * hang_high_blue + 15 * hang_traverse_blue;

        }

    }

    // Launch-Pads Fouls
    private void Do_LaunchPads()
    {
        // Nothing to do here in client mode
        if (GLOBALS.CLIENT_MODE) { return; }

        // G207 rule: If opposing robot touches another who's bumpers contact their launch pads, that's a Foul

        int index = allrobotstates.Count;

        while (index >= 1)
        {
            // Decrement index
            index--;

            RobotStates currbot = allrobotstates[index];

            // Skip this bot if it's been destroyed
            if (!currbot.robot) { continue; }

            // ************ Release immunity if appropriate *******************
            Foul_Release(currbot);

            // See if this robot is inside it's LaunchPad. If not skip
            if (!(currbot.isRed && (lp_red1.IsRobotInside(currbot.robotID) || lp_red2.IsRobotInside(currbot.robotID)) ||
                !currbot.isRed && (lp_blue1.IsRobotInside(currbot.robotID) || lp_blue2.IsRobotInside(currbot.robotID))))
            {
                continue;
            }

            // Give foul to it's enemies
            foreach (RobotInterface3D curr_enemy in currbot.robot.enemies)
            {
                RobotStates enemy_state = GetRobotState(curr_enemy);

                // If immune, skip
                if (enemy_state.immune_to_foul) { continue; }

                // give foul
                enemy_state.immune_to_foul = true;
                enemy_state.LP_StartTime = MyUtils.GetTimeMillis();

                if (currbot.isRed) 
                {
                    penalties_red += rr_settings.LP_FOUL_POINTS;
                }
                else
                {
                    penalties_blue += rr_settings.LP_FOUL_POINTS;
                }

                // give OPR
                player_opr[GLOBALS.client_names[currbot.robotID.id]] += rr_settings.LP_FOUL_POINTS;

                // Instantiate the foul
                CreateFoul(enemy_state.robotID.id);
            }
        }
    }

    // Check if the robot can clear it's foul immunity
    private void Foul_Release(RobotStates currbot)
    {
        if( !currbot.immune_to_foul) { return; }

        if ((int)(MyUtils.GetTimeMillis() - currbot.LP_StartTime) < 1000*rr_settings.FOUL_BLANKING)
        {
            return;
        }

        currbot.immune_to_foul = false;
        currbot.robot.OverrideColor(-1);
    }

    private void Do_HangFoul()
    {
        // Nothing to do here in client mode
        if (GLOBALS.CLIENT_MODE) { return; }

        // If we aren't at end-game, nothing to do
        if (time_total.TotalSeconds > GLOBALS.TIMER_ENDGAME)  { return;  }


        // G207a rule: If opposing robot touches another who's bumpers are in the hang zone, award top hang
        int index = allrobotstates.Count;

        while (index >= 1)
        {
            // Decrement index
            index--;

            RobotStates currbot = allrobotstates[index];

            // Skip this bot if it's been destroyed
            if (!currbot.robot) { continue; }

            // Skip if already awarded hang
            if( hang_awarded_foul.Contains(currbot.robotID.id)) { return; }

            // See if this robot isn't inside it's Hanger, skip
            if (!(currbot.isRed && hanger_red.IsRobotInside(currbot.robotID) ||
                !currbot.isRed && hanger_blue.IsRobotInside(currbot.robotID) ) )
            {
                continue;
            }

            // Give it hang if enemies are inside
            if( currbot.robot.enemies.Count > 0)
            {
                hang_awarded_foul.Add(currbot.robotID.id);

                // Create a foul animation around offending bot
                RobotStates enemy_state = GetRobotState(currbot.robot.enemies[0]);
                CreateFoul(enemy_state.robotID.id);

                // But reset robot's color back to normal
                currbot.robot.enemies[0].OverrideColor(-1);

                // Mark this robot as having valid hang
                MarkRobotForHang(currbot.robotID.id, true);
            }  
        }
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

      
    }

    public override int GetRedScore()
    {
        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }
        score_red = score_auto_red + score_teleop_red + score_endgame_red + penalties_red; 

        return (int) score_red + score_redadj;
    }

    public override int GetBlueScore()
    {
        if( timerstate == TimerState.FINISHED )
        { return score_bluefinal;  }

        score_blue = score_auto_blue + score_teleop_blue + score_endgame_blue + penalties_blue;
        return (int) score_blue + score_blueadj;
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

        // Send foul animation
        serverFlags["FOUL"] = foul_string;
        foul_string = "";

        serverFlags["MARK"] = mark_string;

        // Send Powerup States
        // First send robot powerup states
        string states = "";
        foreach (RobotStates state in allrobotstates)
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
        for (int i = 0; i <= powerup_home.Count; i++)
        {
            PowerUpScript powerup = (i == 0) ? powerup_center : powerup_home[i - 1];
            powerup.ClearOwner();
        }
    }

    // Receiveflags from server
    int animation_played = 0;
    string old_mark_string = "";
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

        // Get fouls
        if (serverFlags.ContainsKey("MARK") && (old_mark_string != serverFlags["MARK"]))
        {
            old_mark_string = serverFlags["MARK"];

            // First unmark all other robots
            foreach ( int currbot in allrobots_byid.Keys)
            {
                MarkRobotForHang(currbot, false);
            }

            // Mark the valid bots
            string[] mark_ids = serverFlags["MARK"].Split(':');
            foreach (string currid in mark_ids)
            {
                if (currid.Length <= 0) { continue; }
                int id = int.Parse(currid);
                MarkRobotForHang(id, true);
            }
        }

        // Get PowerUps
        if (serverFlags.ContainsKey("PU"))
        {
            // First do robot power-ups

            // Clear all existing powerups
            foreach (RobotStates currstate in allrobotstates)
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