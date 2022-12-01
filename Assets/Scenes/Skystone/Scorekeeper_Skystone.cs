using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_Skystone : Scorekeeper {

    // Game specific settings
    Skystone_Settings ss_settings = null;

    // all scorers
    public SS_scoringBox scorer_red_tl;
    public SS_scoringBox scorer_blue_tl;
    public SS_scoringBox scorer_red_f;
    public SS_scoringBox scorer_blue_f;
    public SS_scoringBox blue_scoring;
    public SS_scoringBox red_scoring;

    public DepotHandler red_depot;   
    public DepotHandler blue_depot;

    public GameObject center_marker;

    private double score_red;
    private double penalties_red; // additional points given to ref from opposing teams penalties
    private double penalties_blue;  // additional points given to ref from opposing teams penalties
    private double score_blue;

    // Rules variables
    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 0;
        GLOBALS.TIMER_ENDGAME = 30;
    }

    public override void ScorerInit()
    {
        ScorerReset();

        ss_settings = GameObject.Find("GameSettings").GetComponent<Skystone_Settings>();

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

        scorer_red_tl.Reset();
        scorer_blue_tl.Reset();
        scorer_red_f.Reset();
        scorer_blue_f.Reset();


    }

    public override void Restart()
    {
        base.Restart();

        red_depot.Reset();
        blue_depot.Reset();
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
        }

        CalculateScores();
     
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
        while( index >=1 )
        {
            // Decrement index
            index--;

            RobotStates currbot = robotstates[index];

            // Make sure it wasn't destroyed
            if(currbot.robot == null) { continue; }

            // ********* Update Progress Bars **********
            // Deal with counting up/down
            if (currbot.counting_down)
            {
                //  Check if timer expired
                if( time - currbot.start_time > ss_settings.BLOCKING_DURATION)
                {
                    currbot.counting_down = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((ss_settings.BLOCKING_DURATION - time + currbot.start_time) / ss_settings.BLOCKING_DURATION);
                }
            }

            if (currbot.counting_up)
            {
                // If blocking is disabled, then count down
                if( !ss_settings.ENABLE_BLOCKING)
                {
                    // Start counting down
                    currbot.counting_up = false;
                    currbot.counting_down = true;
                    currbot.start_time = time - (ss_settings.BLOCKING_DURATION - (time - currbot.start_time));
                }
                //  Check if timer expired
                else if (time - currbot.start_time > ss_settings.BLOCKING_DURATION)
                {
                    currbot.counting_up = false;
                    currbot.start_time = -1f;
                    currbot.robot.SetProgressBar(0);
                    currbot.robot.MarkForReset(ss_settings.BLOCKING_RESET_HOLDING_TIME);
                    if (game_running)
                    {
                        if (currbot.isRed) { penalties_blue += ss_settings.PENALTY_BLOCKING; }
                        else { penalties_red += ss_settings.PENALTY_BLOCKING; }
                    }
                }
                else
                {
                    // Update progress bar
                    currbot.robot.SetProgressBar((time - currbot.start_time) / ss_settings.BLOCKING_DURATION);
                }

            }

            // Check for a valid blocking situation
            foreach( RobotInterface3D enemybot in currbot.robot.GetAllEnemies() )
            {
                // If we are inside our foundation, then reset enemy robot right away
                if(ss_settings.ENABLE_FOUNDATION_PENALTY && currbot.isRed  && scorer_red_f.IsFriendInside(currbot.robot.transform) ||
                   ss_settings.ENABLE_FOUNDATION_PENALTY && !currbot.isRed && scorer_blue_f.IsFriendInside(currbot.robot.transform)) 
                {
                    // If already marked for reset, then don't double count
                    if( enemybot.GetNeedsReset()) { continue; }
                    enemybot.MarkForReset(ss_settings.BLOCKING_RESET_HOLDING_TIME);
                    
                    if (game_running && ss_settings.ENABLE_FOUNDATION_PENALTY)
                    {
                        if (currbot.isRed) { penalties_red += ss_settings.PENALTY_SCORING; }
                        else { penalties_blue += ss_settings.PENALTY_SCORING; }
                    }
                }

                // Get enemies state
                int enemyindex = GetRobotStateIndex(enemybot);
                if( enemyindex < 0) { continue; }
                RobotStates enemystate = robotstates[enemyindex];
               
                // If we are inside our safety zone, then count enemy up
                if(ss_settings.ENABLE_BLOCKING && currbot.isRed && red_scoring.IsFriendInside(currbot.robot.transform) ||
                   ss_settings.ENABLE_BLOCKING && !currbot.isRed && blue_scoring.IsFriendInside(currbot.robot.transform))
                {
                    enemystate.isblocking = true;

                    if( enemystate.counting_down)
                    {
                        enemystate.counting_down = false;
                        enemystate.counting_up = true;
                        enemystate.start_time = time - (ss_settings.BLOCKING_DURATION - (time - enemystate.start_time));
                    }
                    else if(!enemystate.counting_up)
                    {
                        enemystate.counting_up = true;
                        enemystate.start_time = time;
                    }
                }
            }

            // Check for too many blocks under control
            int total_blocks = 0;
            found_elements.Clear();

            if (ss_settings.ENABLE_POSSESSION_PENALTY)
            {
                foreach (SS_gameidcounter currcounter in currbot.robot.GetComponentsInChildren<SS_gameidcounter>())
                {
                    total_blocks += currcounter.GetElements(found_elements);
                }

                // Now subtract any elements in the foundation
                foreach( gameElement currelement in found_elements.Values)
                {
                    if (currbot.isRed)
                    {
                        if( scorer_red_f.IsGameElementInside(currelement))
                        {
                            total_blocks -= 1;
                        }
                    }
                    else
                    {
                        if (scorer_blue_f.IsGameElementInside(currelement))
                        {
                            total_blocks -= 1;
                        }
                    }
                }

                if (total_blocks > 1)
                {
                    // Oh ohh, under control of more then one elements
                    if (currbot.toomanyblocks_starttime < 0)
                    {
                        currbot.toomanyblocks_starttime = MyUtils.GetTimeMillis() + ss_settings.PENALTY_POSSESSION_GRACE;
                    }
                    else if (MyUtils.GetTimeMillis() - currbot.toomanyblocks_starttime > 0)
                    {
                        if (currbot.isRed) { penalties_blue += ss_settings.PENALTY_POSSESSION; }
                        else { penalties_red += ss_settings.PENALTY_POSSESSION; }

                        currbot.toomanyblocks_starttime = MyUtils.GetTimeMillis() + 5000; // This is a 5s rule
                    }

                }
            }
        }

        // Now go through bots again and see if a blocking bot is no longer blocking
        index = robotstates.Count;
        while (index >= 1 && ss_settings.ENABLE_BLOCKING)
        {
            index--;

            if( !robotstates[index].isblocking && robotstates[index].counting_up)
            {
                robotstates[index].counting_up = false;
                robotstates[index].counting_down = true;
                robotstates[index].start_time = time - (ss_settings.BLOCKING_DURATION - (time - robotstates[index].start_time));

            }
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

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        data["PenB"] = ((int) blue_penalties).ToString();
        data["PenR"] = ((int) red_penalties).ToString();
        data["ScoreR"] = ((int)score_red + score_redadj).ToString();
        data["ScoreB"] = ((int)score_blue + score_blueadj).ToString();

        data["RFound"] = ((int)red_foundation).ToString();
        data["BFound"] = ((int)blue_foundation).ToString();

        data["RSky"] = ((int)red_skybridge).ToString();
        data["BSky"] = ((int)blue_skybridge).ToString();
        
        data["RLevel"] = ((int)red_highest_block).ToString();
        data["BLevel"] = ((int)blue_highest_block).ToString();

    }


    // Returns the robot state index for the passed robot
    // Since # of robots is max 4, we will do sequential search which is comparable in time to a hash lookup.
    private int GetRobotStateIndex( RobotInterface3D robot)
    {
        for (int index = 0; index < robotstates.Count; index++)
        {
            if( robotstates[index].robot == robot )
            { return index;  }
        }

        return -1;
    }

    double red_penalties = 0;
    double red_skybridge = 0;
    double red_foundation = 0;
    double red_highest_block = 0;

    double blue_penalties = 0;
    double blue_skybridge = 0;
    double blue_foundation = 0;
    double blue_highest_block = 0;


    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if(timerstate != TimerState.RUNNING)
        { return;  }

        red_penalties = scorer_red_tl.penalties + scorer_red_f.penalties + penalties_red;
        red_skybridge = scorer_red_tl.score;
        red_foundation = scorer_red_f.GetElementCount();
        red_highest_block = scorer_red_f.GetHighestBlock() * 2;

        blue_penalties = scorer_blue_tl.penalties + scorer_blue_f.penalties + penalties_blue;
        blue_skybridge = scorer_blue_tl.score;
        blue_foundation = scorer_blue_f.GetElementCount();
        blue_highest_block = scorer_blue_f.GetHighestBlock() * 2;

        score_red = red_skybridge + red_foundation + red_highest_block + red_penalties;
        score_blue = blue_skybridge + blue_foundation + blue_highest_block + blue_penalties;
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

}

// Class for robot state/values
class RobotStates
{
    // Timer variables
    public bool counting_down = false;
    public bool counting_up = false;
    public float start_time = -1f;
    public bool isRed = false;
    public bool isblocking = false;

    public RobotInterface3D robot = null;

    public long toomanyblocks_starttime = -1;
}