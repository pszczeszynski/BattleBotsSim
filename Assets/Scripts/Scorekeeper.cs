using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using TMPro;

public class Scorekeeper : MonoBehaviour {

    public static Scorekeeper instance = null;
    // Object to detect all the different cubes

    // Timer objects
    private Text timer;
    private TextMeshPro field_timer;
    private Text timer_status;
    private GameObject timer_reset;
    public TimeSpan timer_elapsed;
    private GameObject countdown;
    private GameStartAnimation gamestartanimation;
    private GameObject fireworks;
    private bool AUTOFILES = false;
    private long time_last_raw;
    public TimeSpan time_total;

    public bool ALLOW_RESET_POS = true; // Allows a robot position to be reset by user if set to true.
    public bool clean_run = false;
    public string cleancode = "";
    public long OPR_UPDATE_TIME = 500; // ms time between updating OPRs

    // List of all on-field robots
    public List<RobotInterface3D> allrobots = new List<RobotInterface3D>();
    public List<float> allrobots_timecodes = new List<float>();

    // Get all robots by their id
    public Dictionary<int, RobotInterface3D> allrobots_byid = new Dictionary<int, RobotInterface3D>();
    public Dictionary<string, RobotInterface3D> allrobots_byname = new Dictionary<string, RobotInterface3D>();
    public Dictionary<int, RobotID> allRobotID = new Dictionary<int, RobotID>();

    public Dictionary<string, float> player_opr = new Dictionary<string, float>();

    // *** DEBUG 
    // public List<string> player_opr_copy = new List<string>();
    // ***

    public enum TimerState
    {
        READY = 1,
        STOPPED,
        RUNNING,
        PAUSED,
        FINISHED
    };

    public enum FirstGameState
    {
        READY = 1,
        AUTO,
        TELEOP,
        ENDGAME,
        FINISHED
    };

    public TimerState timerstate;
    public FirstGameState firstgamestate = FirstGameState.READY;
    public int score_redfinal;
    public int score_bluefinal;
    public int score_redadj = 0;
    public int score_blueadj = 0;

    // Set shadow mode to true to turn off all auto-update features for multi-player mode
    public bool shadowmode = false;

    // Field element corrections
    protected Vector3 floor_max = new Vector3(1f, 1f, 1f);
    protected Vector3 floor_min = new Vector3(-1f, -1f, -1f);

    public bool floor_was_found = false;


    // Use this for initialization
    // Since this gets objects from other scenes, we want to re-initialize if original initialization fails
    public void Start()
    {
        instance = this;
        ScorerInit();

        // Get timer and scoreboard
        GameObject scoreboard = GameObject.Find("ScoreBoard");
        GameObject timer_obj = GameObject.Find("TIMER");

        // If can't init, abort till later
        if (!timer_obj) { return; }

        CountdownFinder countdownfinder = GameObject.FindObjectOfType<CountdownFinder>();

        if (countdownfinder) // Retrieve the countdown gameobject if countdown was found
        {
            countdown = countdownfinder.countdown;
            countdown.SetActive(false);
        }
        else
        {
            countdown = null;
        }

        GameStartAnimation[] gamestartanimationfinder = Resources.FindObjectsOfTypeAll(typeof(GameStartAnimation)) as GameStartAnimation[];

        if (gamestartanimationfinder.Length >= 1) // Retrieve the countdown gameobject if countdown was found
        {
            gamestartanimation = gamestartanimationfinder[0];
            gamestartanimation.gameObject.SetActive(false);
        }
        else
        {
            gamestartanimation = null;
        }


        // Get fireworks gameobject
        fireworks = GameObject.Find("Fireworks");

        Transform timer_status_obj = timer_obj.transform.Find("Status");
        Transform timer_reset_t = timer_obj.transform.Find("RESET");
        GameObject field_obj = GameObject.Find("FIELD_TIME");

        if (field_obj) { field_timer = field_obj.GetComponent<TMPro.TextMeshPro>(); }


        // Turn on scoreboard if in VR mode
        if ((!MyUtils.XR_isPresent()) || (scoreboard == null))
        {
            if (scoreboard) { scoreboard.SetActive(false); }
            field_obj = null;
        }

        // Start timer in a stopped state
        if (timer_status_obj != null)
        {
            timer = timer_obj.GetComponent<Text>();
            timer_status = timer_status_obj.GetComponent<Text>();
            timer_reset = timer_reset_t.gameObject;
            SetTimerState(TimerState.READY);
        }

        score_redfinal = -1;
        score_bluefinal = -1;
        player_opr.Clear();

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

    private void OnEnable()
    {
        // Re-initialize if required
        if (!floor_was_found) { Start(); }
    }
    // Game specific initialization
    public virtual void ScorerInit()
    {
    }

    private int dofieldchanged = 1;
    public void FieldChanged()
    {
        dofieldchanged = 1;
    }

    // This function doesn't care if players were deleted or added, it just re-creates the internal lists of robots
    public void DoFieldChanged()
    {
        // A change in the field instantiations...
        // Find all robots on the field
        allrobots.Clear();
        allrobots_timecodes.Clear();
        allrobots_byid.Clear();
        allrobots_byname.Clear();
        allRobotID.Clear();

        foreach (RobotInterface3D currbot in GameObject.FindObjectsOfType<RobotInterface3D>())
        {
            // If it is deleted, then ignore
            if (currbot.deleted) { continue; }

            // Get Robot ID
            RobotID currID = currbot.GetComponent<RobotID>();
            if (!currID) { continue; }

            // Add it to our list, this robot may not be new though
            AddPlayer(currbot, false);
        }

        // Call trigger for derived class
        FieldChangedTrigger();

        // Update GameAnimation overlay
        if (gamestartanimation)
        {
            gamestartanimation.OnEnable();
        }
    }

    // This functions populates the robot to our lists, however, if the robot is marked as new it can do additional considerations
    public virtual void AddPlayer(RobotInterface3D currbot, bool robot_is_new = true)
    {
        if ((currbot != null) && (currbot.myRobotID != null))
        {
            // If parent is marked for destruction, it might not be destroyed yet
            GameObject parent = currbot.gameObject;

            if ((parent == null) || (parent.activeSelf == false) || (parent.activeInHierarchy == false))
            {
                return;
            }

            // Add it to the all robots if it doesn't exist alreay
            if (!allrobots.Contains(currbot))
            {
                allrobots.Add(currbot);
                allrobots_timecodes.Add(-1f);
            }

            allrobots_byid[currbot.myRobotID.id] = currbot;
            allRobotID[currbot.myRobotID.id] = currbot.myRobotID;

            if (GLOBALS.client_names.ContainsKey(currbot.myRobotID.id))
            {
                allrobots_byname[GLOBALS.client_names[currbot.myRobotID.id]] = currbot;
            }
        }
    }

    // Virtual function to allow to be overriden
    public virtual void FieldChangedTrigger()
    {
        // At this point the base function has populated all robots
    }

    public bool force_game_end = false;

    public void Update()
    {

        // Even in shadow mode (it just mimicks server's values), we need to do field_timer update here
        if (field_timer) { field_timer.text = timer.text; }

        // Remembr if AUTO was enabled at all
        if (GLOBALS.AUTOMATION_FILES)
        {
            AUTOFILES = true;
        }

        // Calculate new time
        if (!GLOBALS.now_paused && (timerstate == TimerState.RUNNING))
        {

            long currtime = DateTime.Now.Ticks;
            timer_elapsed = TimeSpan.FromTicks(currtime - time_last_raw);
            time_last_raw = currtime;

            // Dont auto increment time if we're in playmode since speeds may be different
            if (!GLOBALS.now_playing)
            { time_total = time_total.Subtract(timer_elapsed); }


            if (force_game_end || (time_total.TotalMilliseconds <= 0))
            {
                // Let scorrer do one more update as the last frame
                ScorerUpdate(true);

                force_game_end = false;
                if (time_total.TotalMilliseconds < 0)
                {
                    time_total = TimeSpan.FromSeconds(0f);
                }
                timer.color = Color.white;
                SetTimerState(TimerState.FINISHED);
                if (firstgamestate == FirstGameState.ENDGAME)
                {
                    GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("GameEnd", 0);
                    firstgamestate = FirstGameState.FINISHED;
                }
            }
            else if (time_total.TotalSeconds <= GLOBALS.TIMER_ENDGAME)
            {
                timer.color = Color.magenta;
                if (firstgamestate == FirstGameState.TELEOP)
                {
                    GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("EndGame", 0);
                    firstgamestate = FirstGameState.ENDGAME;
                }
            }
            else if (time_total.TotalSeconds <= (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO))
            {
                timer.color = Color.white;
                if (firstgamestate == FirstGameState.AUTO)
                {
                    GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("EndAuto", 0);
                    firstgamestate = FirstGameState.TELEOP;
                }
            }
            else
            {
                timer.color = Color.green;
                firstgamestate = FirstGameState.AUTO;
            }

            // Display it: single-player mode will have milliseconds displayed
            if (GLOBALS.SINGLEPLAYER_MODE)
            {
                timer.text = time_total.Minutes.ToString() + ":" + time_total.Seconds.ToString("D2") + "." + ((int)time_total.Milliseconds / 100).ToString("D1");
            }
            else
            {
                timer.text = time_total.Minutes.ToString() + ":" + time_total.Seconds.ToString("D2");
            }

        }
        else
        {
            timer_elapsed = TimeSpan.Zero;
        }

        // Update countdown timer if applicable
        ScorerUpdate();
        UpdateRobotCounter(true);

    }



    public void LateUpdate()
    {
        // Can only search for new robots once old ones have been cleared out, thus need to do it in late update
        // Intentionally wait 1 extra frame to make sure everything clears
        if (dofieldchanged > 0)
        {
            dofieldchanged--;
            if (dofieldchanged == 0) { DoFieldChanged(); }
        }
    }

    // Scorer update is run every update cycle
    public virtual void ScorerUpdate(bool last_frame = false)
    {

    }

    SortedDictionary<string, float> player_opr_red = new SortedDictionary<string, float>();
    SortedDictionary<string, float> player_opr_blue = new SortedDictionary<string, float>();

    // Reset run when timer is set to "ready" state
    public virtual void ScorerReset()
    {
        player_opr.Clear();
        player_opr_red.Clear();
        player_opr_blue.Clear();
        DoFieldChanged();

        foreach (RobotInterface3D currbot in allrobots)
        {
            if (currbot.myRobotID == null) { continue; }

            if (GLOBALS.client_names.ContainsKey(currbot.myRobotID.id))
            {
                player_opr[GLOBALS.client_names[currbot.myRobotID.id]] = 0;
            }
        }

        // Reset score-red adjust and blue-adj
        score_redadj = 0;
        score_blueadj = 0;
        force_game_end = false;
    }

    // Gets called on a level reset after all gameObjects have been reset to their starting pos
    public virtual void Restart()
    {
        // Nothing special
    }

    // Let scorer know that a player did a position reset 
    public virtual void PlayerReset(int id, bool referee = false)
    {
        // Base function does nothing
    }


    // Send OPR's on a reduced rate. Not implemented yet
    // long last_opr_time = 0;

    public virtual void GetScoreDetails(Dictionary<string, string> data)
    {
        data.Clear();
        if (GLOBALS.topserver)
        {
            data["NetFPS"] = GLOBALS.topserver.update_framerate.ToString();
        }

        data["Timer"] = timer.text;
        data["GameState"] = firstgamestate.ToString();

        string oprs = "";

        // DEBUG
        // player_opr_copy.Clear();

        // Go through OPRs and arrange them into red/blue
        foreach (string currid in player_opr.Keys)
        {
            // DEBUG
            //player_opr_copy.Add(currid);
            //player_opr_copy.Add(player_opr[currid].ToString());
            
            // First if this name is not an active player (outdated OPR list), skip this player
            if( !GLOBALS.client_ids.ContainsKey(currid))
            {
                continue;
            }

            // If the currid doesn't exist, we need to re-create our list 
            if (!allrobots_byname.ContainsKey(currid) || (allrobots_byname[currid] == null) || (allrobots_byname[currid].myRobotID == null))
            {
                DoFieldChanged();
            }

            if (!allrobots_byname.ContainsKey(currid) || (allrobots_byname[currid] == null) || (allrobots_byname[currid].myRobotID == null))
            {
                continue;
            }

            if (allrobots_byname[currid].myRobotID.is_red)
            {
                player_opr_red[currid] = player_opr[currid];
            }
            else
            {
                player_opr_blue[currid] = player_opr[currid];
            }
        }

        // Output red first, blue second
        foreach (string currid in player_opr_red.Keys)
        {
            oprs += currid;
            if (player_opr_red[currid] != 0)
            {
                oprs += ": " + player_opr_red[currid].ToString("0");
            }

            oprs += "\n";
        }

        // Add blanks to make sure there are at least 3 lines
        for (int i = 3 - player_opr_red.Count; i >= 1; i--)
        {
            oprs += "\n";
        }

        foreach (string currid in player_opr_blue.Keys)
        {
            oprs += currid;

            if (player_opr_blue[currid] != 0)
            {
                oprs += ": " + player_opr_blue[currid].ToString("0");
            }

            oprs += "\n";
        }

        // Drop the last "\n"
        // if (oprs.Length > 0) { oprs.TrimEnd('\n'); }

        // Client doesn't keep data persistently... yet
        //       if (MyUtils.GetTimeMillis() - last_opr_time > OPR_UPDATE_TIME)
        //       {
        data["OPR"] = oprs;
        //            last_opr_time = MyUtils.GetTimeMillis();
        //        }

        // Get score adjustments
        data["RedADJ"] = score_redadj.ToString();
        data["BlueADJ"] = score_blueadj.ToString();
    }

    public virtual void SetScoreDetails(Dictionary<string, string> data)
    {
        // Timer is re-generated using the time_elapsed values sent so colors can be set. We thus don't use it here
        // if (data.ContainsKey("Timer"))   {  timer.text = data["Timer"];  }

        // This is also re-generated using time_elapsed
        // data["GameState"] = firstgamestate.ToString();

        // Go through OPRs and arrange them into red/blue
        if (data.ContainsKey("OPR"))
        {
            // Clear out old data
            player_opr.Clear();

            string[] oprs = data["OPR"].Split('\n');

            foreach (string curropr in oprs)
            {
                // Split into user:value
                string[] user_data = curropr.Split(':');

                // Make sure only user/value pair was found
                if (user_data.Length != 2) { continue; }

                // Make sure user has a character
                if (user_data[0].Length < 1) { continue; }

                float opr_value = 0;
                if (!float.TryParse(user_data[1], out opr_value)) { continue; }

                player_opr[user_data[0]] = opr_value;
            }

            // Clear out key se we don't keep re-processing this
            data.Remove("OPR");
        }
    }
    public void OnTimerReset()
    {
        SetTimerState(TimerState.READY);
    }

    virtual public void OnTimerStart()
    {
        // Put in things here that need initialization at the moment timer starts
    }

    // Timer related function
    public void OnTimerClick()
    {
        if (shadowmode) { return; }

        if (timer_status == null || timer == null)
        { return; }

        // Deal with the correct timer state
        switch (timerstate)
        {
            case TimerState.READY:
                SetTimerState(TimerState.RUNNING);
                GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("Start", 0);
                OnTimerStart();
                break;

            case TimerState.RUNNING:
                SetTimerState(TimerState.PAUSED);
                clean_run = false;
                cleancode = "";
                break;

            case TimerState.PAUSED:
                SetTimerState(TimerState.RUNNING);
                clean_run = false;
                cleancode = "";
                break;

            case TimerState.FINISHED:
                SetTimerState(TimerState.READY);
                clean_run = false;
                break;

            case TimerState.STOPPED:
                SetTimerState(TimerState.READY);
                clean_run = false;
                cleancode = "";
                break;
        }
    }

    public void SetTimerState(TimerState state, bool server_request = false)
    {
        if (shadowmode && !server_request) { return; }

        switch (state)
        {
            case TimerState.READY:
                DoTimerReady();
                break;

            case TimerState.RUNNING:
                timer_status.text = "RUNNING";
                timer_status.color = Color.gray;
                timer_status.gameObject.SetActive(false);
                timer_reset.SetActive(false);
                time_last_raw = DateTime.Now.Ticks;
                timerstate = TimerState.RUNNING;
                break;

            case TimerState.STOPPED:
                timer_status.text = "STOPPED";
                timer_status.color = Color.red;
                timer_status.gameObject.SetActive(true);
                timer_reset.SetActive(true);
                timerstate = TimerState.STOPPED;
                break;

            case TimerState.PAUSED:
                timer_status.text = "PAUSED";
                timer_status.color = Color.yellow;
                timer_status.gameObject.SetActive(true);
                timer_reset.SetActive(true);
                timerstate = TimerState.PAUSED;
                break;

            case TimerState.FINISHED:
                DoTimerFinished();
                break;
        }
    }

    public virtual void DoTimerReady()
    {
        // Initialize timer and get it ready. 
        score_redfinal = 0;
        score_bluefinal = 0;
        time_total = TimeSpan.FromSeconds(GLOBALS.TIMER_TOTAL);
        timer.text = time_total.Minutes.ToString() + ":" + time_total.Seconds.ToString("D2");
        timer_status.text = "READY";
        timer_status.color = Color.green;
        timer_status.gameObject.SetActive(true);
        timer_reset.SetActive(false);
        timerstate = TimerState.READY;
        ScorerReset();
        firstgamestate = FirstGameState.READY;

        // Record automation state at the beggining match
        AUTOFILES = GLOBALS.AUTOMATION_FILES;
    }


    // Allow DoTimerFinished to be overriden in order to add other finishing touches
    public virtual void DoTimerFinished()
    {
        timer_status.text = "FINISHED" + ((GLOBALS.AUTOMATION_FILES || AUTOFILES) ? " (AUTO)" : "");
        timer_status.color = Color.green;
        timer_status.gameObject.SetActive(true);
        timer_reset.SetActive(false);
        score_redfinal = GetRedScore();
        score_bluefinal = GetBlueScore();
        timerstate = TimerState.FINISHED;
    }


    public String GetTimerText()
    {
        return timer.text;
    }

    public void SetTimerText(String text)
    {
        timer.text = text;
    }

    public String GetTimerState()
    {
        return timer_status.text;
    }

    // String version runs the non-string verison
    // String sent via server-client
    public void SetTimerState(String text)
    {
        TimerState new_state = (TimerState)Enum.Parse(typeof(TimerState), text);

        if (new_state != timerstate)
        {
            SetTimerState(new_state, true);
        }

    }


    public bool IsTimerFinished()
    {
        return (timerstate == TimerState.FINISHED) || (timerstate == TimerState.STOPPED);
    }

    public bool IsTimerRunning()
    {
        return timerstate == TimerState.RUNNING;
    }

    public virtual int GetRedScore()
    {
        int score = 0;

        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }

        return score;
    }

    public virtual int GetBlueScore()
    {
        int score = 0;

        if (timerstate == TimerState.FINISHED)
        { return score_bluefinal; }

        return score;
    }

    public virtual string GetDetails(bool red = true)
    {
        // No details to list by default
        return "";
    }

    // Run countdown animation
    public void StartCountdown()
    {
        if (countdown)
        {
            countdown.SetActive(true);
        }

        GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("Countdown", 0);
    }

    // Shows overlay before a match
    // passivemode=true allows this to be overriden by /SET SHOW_VS screen
    // returns true if a change occured
    private float animation_start_time = 0f;
    public bool ShowGameStartOverlay(bool enable = true, bool passivemode = false)
    {
        bool returnvalue = false;
        // Show the animation
        if (gamestartanimation)
        {
            // IF animation is alreadly enabled or already off, then dont do anything
            if( enable && gamestartanimation.gameObject.activeSelf) { return false; }
            if (!enable && !gamestartanimation.gameObject.activeSelf) { return false; }

            if (enable)
            {
                gamestartanimation.gameObject.SetActive(true);
                animation_start_time = Time.time;
                returnvalue = true;
            }
            else
            {
                if (!passivemode || !animation_override)
                {
                    gamestartanimation.EndAnimation();
                    animation_override = false;
                    animation_start_time = 0f;
                    returnvalue = true;
                }
            }
        }

        // Play the sound
        if (enable)
        {
            GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("Hype1", 0);
        }
        else
        {
            // The music may have been playing because of a no-animation enabled sequence. Hence here lets force it to stop
            GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Stop("Hype1", 0);
        }

        return returnvalue;
    }

    public virtual void UpdateRobotCounter(bool only_if_smaller = false)
    {
        // Go through each bot and update their timer
        foreach( RobotID currbot in allRobotID.Values)
        {
            if( currbot.is_counting)
            {
                RobotInterface3D curr_i3d = allrobots_byid[currbot.id];
                if( !curr_i3d) { continue; }

                float percentage = (currbot.count_duration - currbot.count_start + ((float)time_total.TotalSeconds)) / currbot.count_duration;
                if (!only_if_smaller || (curr_i3d.GetProgressBar() <= 0) || (percentage < curr_i3d.GetProgressBar()))
                {
                    curr_i3d.SetProgressBar(percentage);
                }

                if( percentage <= 0)
                {
                    RobotCounterExpired(currbot);
                }
             }
        }
    }

    // Applies penalties for any countdown that expired
    public virtual void RobotCounterExpired(RobotID bot)
    {
        bot.is_counting = false;
        bot.count_start = 0f;

        // Use the referee reset 
        PlayerReset(bot.id, true);
        if( allrobots_byid[bot.id])
        {
            if (allrobots_byid[bot.id])
            {
                allrobots_byid[bot.id].MarkForReset(1f);
            }
        }
        
    }

    public virtual void ResetRobotCounter(int id)
    {
        RobotID bot = allRobotID[id];
        RobotInterface3D robot_i3d = allrobots_byid[bot.id];
        if( robot_i3d) { robot_i3d.SetProgressBar(0); }
        bot.is_counting = false;
        bot.count_start = 0f;
    }

    public virtual void SetRobotCounter(int id, float duration)
    {
        RobotID bot = allRobotID[id];

        if (duration > 0)
        {
            bot.is_counting = true;
            bot.count_start = (float)time_total.TotalSeconds;
            bot.count_duration = duration;
        }
        else
        {
            ResetRobotCounter(id);
        }
    }

    public virtual bool UseGameStartOverlay()
    {
        return true;
    }

    // Shows overlay before a match
    // Plays overlay music
    // If enabled using this method, it will override the passive turn-off from the server.
    // only active turn-off will disable this
    private bool animation_override = false;
    public void ShowGameStartOverlayNoAnimation(bool enable = true)
    {
        
        if (gamestartanimation)
        {
            if (enable)
            {
                animation_override = true;
                gamestartanimation.ShowNoAnimation(true);
                GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("Hype1", 0, -1, true);
            }
            else
            {
                animation_override = false;
                gamestartanimation.ShowNoAnimation(false);
                GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Stop("Hype1", 0, true);
            }
        }
    }

    // Show red fireworks
    public void StartFireworks(bool red)
    {
        if (fireworks && !GLOBALS.HEADLESS_MODE)
        {
            if (red) { fireworks.GetComponent<FireworksFinder>().fireworksRed.SetActive(true); }
            else     { fireworks.GetComponent<FireworksFinder>().fireworksBlue.SetActive(true); }
        }

        GameObject.Find("AudioManagerTop").GetComponent<AudioManager>().Play("fireworks", 0);
    }


    private Dictionary<string, string> match_details = new Dictionary<string, string>();

    // Add flags to send to clients to sync states
    public virtual void SendServerData(Dictionary<string, string> serverFlags)
    {
        // Get Scores
        serverFlags["REDSCORE"] = GetRedScore().ToString();
        serverFlags["BLUESCORE"] = GetBlueScore().ToString();

        // Get Time
        serverFlags["TIMER"] = GetTimerText();
        serverFlags["TIMERSTATE"] = GetTimerState();
        serverFlags["TIMERAW"] = ((int) time_total.TotalMilliseconds).ToString();

        // Get details of match for output files
        GetScoreDetails(match_details);
        serverFlags["SCORE"] = string.Join(";", match_details.Select(x => x.Key + "=" + x.Value));
    }

    public virtual void ProcessScoreKey(Dictionary<string, string> serverFlags)
    {
        if( ! serverFlags.ContainsKey("SCORE")) { return; }

        // Break the score apart
        string[] all_data = serverFlags["SCORE"].Split(';');
        if (all_data.Length < 1) { return; }

        // Now go through all the data
        for (int i = 0; i < all_data.Length; i++)
        {
            string[] key_value = all_data[i].Split('=');
            if (key_value.Length != 2) { continue; }  // Make sure there are 2 items: Key = Value

            match_details[key_value[0]] = key_value[1];
        }

        SetScoreDetails(match_details);

        // serverFlags.Remove("SCORE");
    }

    // Receiveflags from server
    public virtual void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        // Get timer states for timer color
        string timer;
        string s_timerstate;

        if (serverFlags.TryGetValue("TIMERAW", out timer))
        {
            time_total = TimeSpan.FromMilliseconds(int.Parse(serverFlags["TIMERAW"] ));
            time_last_raw = DateTime.Now.Ticks;

            serverFlags.Remove("TIMERAW");
        }

        if (serverFlags.TryGetValue("TIMERSTATE", out s_timerstate))
        {
            SetTimerState(s_timerstate);
            serverFlags.Remove("TIMERSTATE");

            // Do sanity check on "hype" animation - make sure it's off
            if( (timerstate == TimerState.RUNNING) && gamestartanimation && gamestartanimation.enabled)
            {
                // Force turnoff, but only after a minimum period if it was just started
                if (Time.time - animation_start_time > 1f)
                {
                    ShowGameStartOverlayNoAnimation(false);
                }
            }
        }

        // Get the Score keys
        ProcessScoreKey(serverFlags);

    }

    // Qualify robot name: apply restrictions if applicable
    public virtual string CorrectRobotChoice(string requested_robot)
    {
        return requested_robot;
    }

    // Qualify robot position
    private Dictionary<string, Transform> allPositions = new Dictionary<string, Transform>();
    public virtual Transform CorrectRobotPosition(string requested_position, List<string> used_positions)
    {
        // Find all game positions if not already
        if( allPositions.Count < 1)
        {
            // Find all starting positiong
            GameObject go_positions = GameObject.Find("Positions");
            if( !go_positions ) { return null; }

            Transform field_positions = go_positions.transform;
            for (int i = field_positions.childCount - 1; i >= 0; i--)
            {
                allPositions.Add(field_positions.GetChild(i).name, field_positions.GetChild(i));
            }

            // If none found, exit
            if( allPositions.Count < 1)
            {
                return null;
            }
        }

        // If position is taken, return null
        if (used_positions.Contains(requested_position)) { return null; }

        // If position exists, return it otherwise we will move on to find any available
        if (allPositions.ContainsKey(requested_position)) {  return allPositions[requested_position];  }

        // Figure out if we're looking for red or blue (or something different altogether)
        bool find_blue = requested_position.Contains("Blue");
        bool find_red = requested_position.Contains("Red");

        foreach ( string curr_pos in allPositions.Keys)
        {
            if( find_blue && !curr_pos.Contains("Blue") ) { continue; }
            if (find_red && !curr_pos.Contains("Red")) { continue; }

            if ( !used_positions.Contains(curr_pos) ) { return allPositions[curr_pos]; }
        }

        // No free position found
        return null;
    }

    // Corrects the field element if required. By default it just checks if the element
    // flew off the board
    virtual public bool CorrectFieldElement(GameObject currobj)
    {
        if( !floor_was_found)
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
            currobj.transform.position = new Vector3(
                    UnityEngine.Random.Range(floor_min.x, floor_max.x), floor_max.y + 20f, UnityEngine.Random.Range(floor_min.z, floor_max.z));

            Rigidbody rb = currobj.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = new Vector3(0, 0, 0);
                rb.angularVelocity = new Vector3(0, 0, 0);
            }

            return true;
        }

        return false;
    }

    virtual public bool IsTransformOffField(Transform currobj)
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
        return currobj.position.y < (floor_min.y - 5);
    }

    // Let scorer know a user requested a camera change, and if so, change it accordingly.
    virtual public void OnCameraViewChanged()
    {
        // Do nothing nominally
    }

    // If a server interrupt is sent under the key "SCORER", this function will be called
    virtual public void OnScorerInterrupt(string msg)
    {
        // nothing to do
    }

    virtual public Transform GetOverlays(int id, Transform parent = null)
    {
        return null;
    }

    public virtual String GetOverlaysString(int id)
    {
        return "";
    }

    virtual public bool PU_CheckIfClearToAssign(PowerUpScript thepu, RobotInterface3D robot)
    {
        return true;

    }

    virtual public void RobotDroppedItem(int robotid, gameElement item)
    {

    }

    public void CopyCleanCodeToSystemBuffer()
    {
        GUIUtility.systemCopyBuffer = cleancode;
    }
}
