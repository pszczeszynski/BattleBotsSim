using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_LastManStanding : Scorekeeper
{

    // Game specific settings
    public LastManStanding_Settings lms_settings = null;
    private LSM_BallCollision[] allBalls = null;
    private Transform floor;
    public float floor_max_scale = 3f;
    public int TIME_AUTO = 30;
    private List<Transform> Flames = new List<Transform>();
    public int starting_players_blue = 0;
    public int starting_players_red = 0;
    public int curr_red_count = 0;
    public int curr_blue_count = 0;
    public int red_kills = 0;
    public int blue_kills = 0;
    public TMPro.TextMeshProUGUI textScores;
    public GameObject textScores_panel;
    private string gameovercode = "";
    float center_offset = 0.75f;
    public int mystate = 0; // general my state. So far: 0 = alive, 1 = dead


    // Game Over stuff
    public TMPro.TextMeshProUGUI gameover_text;
    public GameObject gameoverpanel;

    List<int> start_players = new List<int>();
    List<int> dead_players = new List<int>();
    List<int> live_players = new List<int>();
 
    // Red and Blue score objects
    public GameObject redtextobj = null;
    public GameObject bluetextobj = null;

    private void Awake()
    {
        // GLOBALS.PlayerCount = 12; - set by lms_settings
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = TIME_AUTO;
        GLOBALS.TIMER_ENDGAME = -1;
        ALLOW_RESET_POS = false; // Don't allow players to reset their position
                                 
        // Get the red score
        if (redtextobj == null)
        { redtextobj = GameObject.Find("REDSCORE"); }

        // Get the blue score
        if (bluetextobj == null)
        { bluetextobj = GameObject.Find("BLUESCORE"); }

        if(textScores_panel) { textScores_panel.SetActive(false); }

    }

    private bool ConfigureField()
    {
        // Make sure there is a settings menu
        if( lms_settings == null) { return false; }

        // Let clients know to configure their fields
        if( GLOBALS.SERVER_MODE && GLOBALS.topserver)
        {
            GLOBALS.topserver.AddInterrupt("SCORER", "CF");
        }

        // Find all the objects to configure
        List<GameObject> G_list = new List<GameObject>();

        // Add the game objects
        G_list.Add(GameObject.Find("CenterField/G1"));
        G_list.Add(GameObject.Find("CenterField/G2"));
        G_list.Add(GameObject.Find("CenterField/G3"));
        G_list.Add(GameObject.Find("CenterField/G4"));


        // Go Through all the field objects children
        foreach ( GameObject curr_G in G_list)
        {
            // If null exit
            if( curr_G == null) { return false; }

            // Go through all their children and deal with it
            for( int i = 0; i < curr_G.transform.childCount; i++)
            {
                Transform curr = curr_G.transform.GetChild(i);

                // If it's a row item, set position and enable state depending on situation
                if( curr.name.StartsWith("row"))
                {
                    float row_id = float.Parse(curr.name.Substring(3, 1));  // Row ID sets the Z spacing.. just spacing * row_id
                    int pos_id = int.Parse(curr.name.Substring(5, 1)); // pos_id sets the half spacing for x. 0.5*SPACING*pos_id if <5; If pos_id >5, pos_id = -1*(pos_id-5);
                    row_id -= (1 - center_offset);
                    if (pos_id > 5)
                    {
                        pos_id = 5 - pos_id;
                    }

                    // Z position = spacing. X position = ??
                    Vector3 pos = curr.localPosition;
                    pos.z = lms_settings.ROW_SPACING * row_id;
                    pos.x = lms_settings.ROW_SPACING * pos_id;
                    curr.localPosition = pos;

                    // Next disable rows not used. Remember that last row is reserved for left/center/right spawn positions
                    if( row_id >= lms_settings.ROW_COUNT)
                    {
                        curr.gameObject.SetActive(false);
                    }
                    else
                    {
                        curr.gameObject.SetActive(true);
                    }
                   
                }
                else if( curr.name.StartsWith("Protection"))
                {
                    float pos_id = float.Parse(curr.name.Substring(10, 1)); // pos_id sets the spacing for x. 2*SPACING*pos_id if <5; If pos_id >5, pos_id = -1*(pos_id-5);

                    if (pos_id > 5f)
                    {
                        pos_id = 5f - pos_id;
                    }

                    // Z position = spacing. X position = index * ROW_SPACING
                    Vector3 pos = curr.localPosition;
                    pos.z = lms_settings.ROW_SPACING * (((float)lms_settings.ROW_COUNT) - (1 - center_offset));
                    pos.x = lms_settings.ROW_SPACING * (lms_settings.ROW_COUNT - 1) * pos_id / 2f;
                    curr.localPosition = pos;


                    // Enable/disable based on protection setting
                    if (lms_settings.SPAWN_WALLS)
                    {
                        curr.gameObject.SetActive(true);
                    }
                    else
                    {
                        curr.gameObject.SetActive(false);
                    }
                }
                else if (curr.name.StartsWith("spawn"))
                {
                    // Deal with Spawn objects
                    float pos_id = float.Parse(curr.name.Substring(5, 1)); // pos_id sets the spacing for x. 2*SPACING*pos_id if <5; If pos_id >5, pos_id = -1*(pos_id-5);

                    if (pos_id > 5f)
                    {
                        pos_id = 5f - pos_id;
                    }

                    // Z position = spacing. X position = index * ROW_SPACING
                    Vector3 pos = curr.localPosition;
                    pos.z = lms_settings.ROW_SPACING * (((float)lms_settings.ROW_COUNT) - (1 - center_offset));
                    pos.x = lms_settings.ROW_SPACING * (lms_settings.ROW_COUNT-1) * pos_id;
                    curr.localPosition = pos;

                    // enable/disable wall
                    Transform protectionwall = curr.Find("Protection");
                    if(protectionwall != null)
                    {
                        protectionwall.gameObject.SetActive(lms_settings.SPAWN_WALLS);
                    }
                }

            }
        }

        // Calculate floor-max scale
        floor_max_scale = lms_settings.ROW_SPACING * (((float)lms_settings.ROW_COUNT) - (1 - center_offset)) / 5f + 0.75f;

        // Save spawn ball locations for later
        List<Transform> spawn_balls = new List<Transform>();

        // Next, set our spawn locations
        List<Transform> spawn_red = new List<Transform>();
        foreach( Transform currpos in GameObject.Find("CenterField/G2").transform.GetComponentsInChildren<Transform>(true) )
        {
            // If the name is a spawnpoint, add it in
            if( currpos.name.StartsWith("SpawnPoint"))
            {
                spawn_red.Add(currpos);
            }

            if( currpos.name.StartsWith("spawnball"))
            {
                spawn_balls.Add(currpos);
            }
        }

        foreach (Transform currpos in GameObject.Find("CenterField/G1").transform.GetComponentsInChildren<Transform>(true))
        {
            // If the name is a spawnpoint, add it in
            if (currpos.name.StartsWith("SpawnPoint"))
            {
                spawn_red.Add(currpos);
            }

            if (currpos.name.StartsWith("spawnball"))
            {
                spawn_balls.Add(currpos);
            }
        }

        List<Transform> spawn_blue = new List<Transform>();
        foreach (Transform currpos in GameObject.Find("CenterField/G4").transform.GetComponentsInChildren<Transform>(true))
        {
            // If the name is a spawnpoint, add it in
            if (currpos.name.StartsWith("SpawnPoint"))
            {
                spawn_blue.Add(currpos);
            }

            if (currpos.name.StartsWith("spawnball"))
            {
                spawn_balls.Add(currpos);
            }
        }

        foreach (Transform currpos in GameObject.Find("CenterField/G3").transform.GetComponentsInChildren<Transform>(true))
        {
            // If the name is a spawnpoint, add it in
            if (currpos.name.StartsWith("SpawnPoint"))
            {
                spawn_blue.Add(currpos);
            }

            if (currpos.name.StartsWith("spawnball"))
            {
                spawn_balls.Add(currpos);
            }
        }

        // Get all the positions in the field into a red and blue list
        GameObject go_positions = GameObject.Find("Positions");
        if (!go_positions) { return false; }

        List<Transform> red_starting = new List<Transform>();
        List<Transform> blue_starting = new List<Transform>();

        Transform field_positions = go_positions.transform;
        for (int i = field_positions.childCount - 1; i >= 0; i--)
        {
            if(field_positions.GetChild(i).name.StartsWith("Red") )
            {
                red_starting.Add(field_positions.GetChild(i));
            }
            else
            {
                blue_starting.Add(field_positions.GetChild(i));
            }
        }

        // Now set all the spawn point to equal the new field points
        for ( int i = 0; (i < red_starting.Count) && (i < spawn_red.Count); i++)
        {
            red_starting[i].position = spawn_red[i].position;
            red_starting[i].rotation = spawn_red[i].rotation;
        }

        // Now set all the spawn point to equal the new field points
        for (int i = 0; (i < blue_starting.Count) && (i < spawn_blue.Count); i++)
        {
            blue_starting[i].position = spawn_blue[i].position;
            blue_starting[i].rotation = spawn_blue[i].rotation;
        }

        // Set our ball positions
        // First get all our balls
        GameObject go_ball_positions = GameObject.Find("Balls");
        if (!go_ball_positions) { return false; }

        Transform ball_positions = go_ball_positions.transform;

        // Now go through the balls and set their positions to that marked inside our spawn regions.
        // The spawn region positions were saved i nthe code above.
        for (int i = ball_positions.childCount - 1; i >= 0; i--)
        {
            if( i < spawn_balls.Count)
            {
                ball_positions.GetChild(i).position = spawn_balls[i].position;

                // Just in case reset the balls velocities
                Rigidbody its_rb = ball_positions.GetChild(i).GetComponent<Rigidbody>();
                if( its_rb)
                {
                    its_rb.velocity = Vector3.zero;
                    its_rb.angularVelocity = Vector3.zero;
                    its_rb.Sleep();
                }
            }
        }

        // Finally adjust the spectator cam.
        GameObject specCam = GameObject.Find("Spectator Cam");
        if( !specCam) { return true; }

        Transform specCam_Transform = specCam.transform;
        Vector3 cam_position = specCam_Transform.localPosition;
        Vector3 cam_rotation = specCam_Transform.localRotation.eulerAngles;

        cam_position.y = floor_max_scale * 12.25f -4f;
        cam_rotation.x = floor_max_scale * 3.72f + 64.4f;

        specCam_Transform.localPosition = cam_position;
        Quaternion newrot = new Quaternion
        {
            eulerAngles = cam_rotation
        };
        specCam_Transform.localRotation = newrot;

        return true;
    }

    public override void ScorerInit()
    {
        GameObject settings = GameObject.Find("GameSettings");
        if (settings) { 
            lms_settings = settings.GetComponent<LastManStanding_Settings>(); 
        }

        // Get all the field balls
        allBalls = GameObject.FindObjectsOfType<LSM_BallCollision>();

        // Get the floor
        GameObject floor_go = GameObject.Find("3d field/floor");
        if (floor_go)
        {
            // Get the floor
            floor = floor_go.transform;

            // Get all the flames
            for( int i =0; i < floor.childCount; i++)
            {
                Flames.Add(floor.GetChild(i));

                // Pause the flames if in lowest setting
                if( QualitySettings.GetQualityLevel() ==  0)
                {
                    ParticleSystem flame = floor.GetChild(i).GetComponent<ParticleSystem>();
                    if( flame)
                    {
                        flame.Pause();
                    }
                }
            }
        
        }

        ScorerReset();

        // Get the red score
        if (redtextobj == null)
        { redtextobj = GameObject.Find("REDSCORE"); }

        // Get the blue score
        if (bluetextobj == null)
        { bluetextobj = GameObject.Find("BLUESCORE"); }

    }

    override public void OnTimerStart()
    {
        base.OnTimerStart();
        ScorerReset();
        start_players.Clear();
        dead_players.Clear();
        live_players.Clear();

        // Initialize player count and List 
        foreach (RobotID currid in allRobotID.Values)
        {
            // Count players
            if (currid.is_red) { starting_players_red++; }
            else { starting_players_blue++; }

            start_players.Add(currid.id); // Add player id to starting players. Even if they disconnect, this ID is forever retained.
            live_players.Add(currid.id);
        }

        // Set the robot colors
        SetRobotColors();

        // Get the red score
        if (redtextobj == null)
        { redtextobj = GameObject.Find("REDSCORE"); }

        // Get the blue score
        if (bluetextobj == null)
        { bluetextobj = GameObject.Find("BLUESCORE"); }

    }

    private void SetRobotColors()
    {
        // Initialize player count and List 
        foreach (RobotID currid in allRobotID.Values)
        {
            RobotInterface3D currbot = allrobots_byid[currid.id];
            if (currbot)
            {
                if (lms_settings.FREE_FOR_ALL)
                {
                    currbot.SetRobotColor(3);  // Set to LMS robot color
                }
                else
                {
                    currbot.SetColorFromPosition(currid.starting_pos);
                }
            }
        }
    }

    public override void ScorerReset()
    {
        // Configure the field
        ConfigureField();

        base.ScorerReset();

        bool did_field_change = false;
        // Clear any scores
        foreach (RobotID currid in allRobotID.Values)
        {
            // Make sure this robotID still exists
            if (!currid)
            {
                did_field_change = true;
                continue;
            }

            currid.SetUserFloat("KILLS", 0);
            currid.SetUserInt("HITS", 0);
            currid.SetUserBool("DEAD", false);

            if( GLOBALS.SERVER_MODE && GLOBALS.topserver)
            {
                GLOBALS.topserver.ClearFlag("DEAD", currid.id);
            }

            // Make robot physical again
            allrobots_byid[currid.id].EnableTopObjects();
            allrobots_byid[currid.id].SetKinematic(false);
        }

        // If a null RobotID was found (robot got deleted), mark a field change event
        if (did_field_change) { DoFieldChanged(); }
        red_kills = 0;
        blue_kills = 0;
        starting_players_blue = 0;
        starting_players_red = 0;

        ResetFloor();
    }

    void ResetFloor()
    {
        // Resize floor 
        if (!floor) { return; }

        // Reset floor size
        Vector3 localScale = floor.localScale;
        localScale.x = floor_max_scale;
        localScale.z = floor_max_scale;
        floor.localScale = localScale;

        SetFlamesToFloor();
    }

    void SetFlamesToFloor()
    {
        if( !floor) { return; }

        foreach( Transform currflame in Flames)
        {
            currflame.localScale = new Vector3(floor.localScale.x/2f, 2f, 1f);
        }
    }

    public override void Restart()
    {
        base.Restart();
    }

    // Scorer update gets run after base runs it's update, thus base will have process end-of-time already
    private TimerState timerstate_old = TimerState.READY;
    private bool escape_was_pressed = false;
    public override void ScorerUpdate(bool last_frame = false)
    {
        // Turn score panel off/on as appropriate
        if (GLOBALS.CLIENT_MODE && GLOBALS.topclient && (GLOBALS.topclient.connection_state == ClientLow.ConnectionStates.CONNECTED))
        {
            textScores_panel.SetActive(true);
        }
        else if(GLOBALS.SERVER_MODE && GLOBALS.topserver && GLOBALS.topserver.serverReady)
        {
            textScores_panel.SetActive(true);
        }
        else
        {
            textScores_panel.SetActive(false);
        }

        // If escape is pressed, turn off the score display
        if( gameoverpanel.activeSelf && Input.GetKey(KeyCode.Escape))
        {
            gameoverpanel.SetActive(false);
            gameoverpanel.GetComponent<Animator>().enabled = false;
            escape_was_pressed = true;
        }

        // Server side: determine if gameoverpanel needs to be turned on or off
        if (GLOBALS.SERVER_MODE)
        {
            // Start Game-Over animations if applicable
            if ((timerstate == TimerState.FINISHED) && (timerstate_old == TimerState.RUNNING))
            {
                gameoverpanel.SetActive(true);
                gameoverpanel.GetComponent<Animator>().enabled = true;
                CreateGameOverText(gameovercode);

            }
            timerstate_old = timerstate;

            // Get rid of game-over animation if timer-state is not in finished mode
            if ((timerstate != TimerState.FINISHED) && gameoverpanel.activeSelf)
            {
                gameoverpanel.GetComponent<Animator>().enabled = false;
                gameoverpanel.SetActive(false);          
            }
        }

        // Don't update if not running
        if (timerstate != TimerState.RUNNING)
        {
            ResetFloor(); 
            return; 
        }

        // Deal with players that need to be killed
        CheckPlayers();

        //Calculate scores
        CalculateScores();

    }

    public void CheckPlayers()
    {
        // Do nothing if in client mode
        if (GLOBALS.CLIENT_MODE) { return; }

        bool did_field_change = false;

        foreach (RobotID currid in allRobotID.Values)
        {
            // Make sure this robotID still exists
            if (!currid)
            {
                did_field_change = true;
                continue;
            }

            RobotInterface3D currbot = allrobots_byid[currid.id];

            // If this robot was deleted (e.g. disconnected), then skip
            if( !currbot  ) {
                did_field_change = true; 
                continue;  
            }

            // If this robot is dead, then continue
            if( currid.GetUserBool("DEAD")) { continue; }

            // If the robot fell below our threshold, mark is as dead
            if( currbot.rb_body.transform.position.y < -2f)
            {
             
                // Start animation if applicable
                var thebot = currbot as Robot_LastManStanding;

                // Mark as dead
                currid.SetUserBool("DEAD");
                if( GLOBALS.SERVER_MODE && GLOBALS.topserver) { GLOBALS.topserver.AddFlag("DEAD", "1", currid.id);  }
                dead_players.Add(currid.id);
                live_players.Remove(currid.id);

                if (thebot)
                {
                    // Plays death animation, including ending robot in an invisible disabled state
                    thebot.PlayDeathAnimation();
                }
                else
                {
                    // Should never get here since as of this writing only 1 robot allowed to be in this game-mode
                    currbot.SetKinematic(true);
                    currbot.DisableTopObjects();
                }
                continue;
            }

            // Set the progress bar based on their life
            // If the HIT_Count=1, then no progress bar is required
            if (lms_settings.HIT_COUNT <= 1)
            {
                currbot.SetProgressBar(0);
            }
            else
            {
                currbot.SetProgressBar((float)((lms_settings.HIT_COUNT - currid.GetUserInt("HITS")) / ((float)lms_settings.HIT_COUNT)));
            }

            // If this robots hit count meets death threshold, mark it so
            if (currid.GetUserInt("HITS") >= lms_settings.HIT_COUNT)
            {
                if( currid.is_red) { blue_kills++; }
                else { red_kills++;  }

                currid.SetUserBool("DEAD");
                if (GLOBALS.SERVER_MODE && GLOBALS.topserver) { GLOBALS.topserver.AddFlag("DEAD", "1", currid.id); }
                dead_players.Add(currid.id);
                live_players.Remove(currid.id);

                // Send server message about who killed who
                int id_of_killer = currid.GetUserInt("LAST_HIT_ID");
                if( id_of_killer > 0)
                {
                    if( GLOBALS.SERVER_MODE && GLOBALS.topserver)
                    {
                        System.Random rand = new System.Random();
                        int random_index = rand.Next(0, GLOBALS.killingWords.Count - 1);

                        GLOBALS.topserver.AddChat(GLOBALS.client_names[id_of_killer] + " " + GLOBALS.killingWords[random_index] + " " + GLOBALS.client_names[currid.id]);
                    }
                }

                // Start animation if applicable
                var thebot = currbot as Robot_LastManStanding;
                if (thebot)
                {
                    // This marks the robot as a "Spectator" which is used to determine it is dead
                    thebot.PlayDeathAnimation();
                }
                else
                {
                    // Should never get here since as of this writing only 1 robot allowed to be in this game-mode
                    currbot.SetKinematic();
                    currbot.HoldRobot();
                    
                }
            }
        }

        // If a null RobotID was found (robot got deleted), mark a field change event
        if (did_field_change) { DoFieldChanged(); }

        // Now update player count
        curr_red_count = 0;
        curr_blue_count = 0;
        foreach (RobotID currid in allRobotID.Values)
        {
            // Bypass if dead
            if (currid.GetUserBool("DEAD")) { continue; }

            // Add it to the red/blue count
            if (currid.is_red)
            {
                curr_red_count++;
            }
            else
            {
                curr_blue_count++;
            }
        }

    }

    int score_red = 0;
    int score_blue = 0;

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        data["ScoreR"] = ((int)score_red + score_redadj).ToString();
        data["ScoreB"] = ((int)score_blue + score_blueadj).ToString();
    }

    // Need sorting function to list players by their OPR
    // Reverse sort though
    int SortByOPR( int p1, int p2)
    {
        if( ! player_opr.ContainsKey(GLOBALS.client_names[p1]) )
        { player_opr[GLOBALS.client_names[p1]] = 0; }

        if( !player_opr.ContainsKey(GLOBALS.client_names[p2]))
        { player_opr[GLOBALS.client_names[p2]] = 0; }

        return -1*player_opr[GLOBALS.client_names[p1]].CompareTo(player_opr[GLOBALS.client_names[p2]]);
    }
  

    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if (timerstate != TimerState.RUNNING)
        { return; }

        // Make sure settings have loaded
        if (!lms_settings) { return; }
        if( !floor ) { return; }

        // Resize Matt as appropriate
        Vector3 localSize = floor.localScale;
        if (time_total.TotalSeconds >= GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)
        {
            localSize.x = floor_max_scale;
            localSize.z = floor_max_scale;
        }
        else
        {
            float scaler = (float) (time_total.TotalSeconds / ((double) (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)) * floor_max_scale);
            localSize.x = scaler;
            localSize.z = scaler;
        }

        floor.localScale = localSize;
        SetFlamesToFloor();

        // Nothing more to be done
        if (GLOBALS.CLIENT_MODE) { return; }

        // At this point, all invalid robots have been removed and curr player count has been updated
        // Thus we will give 1 point for killing a bot, a point for surviving one more round
        // Calculate score of each bot
        int survival_score_red = starting_players_blue - curr_blue_count;
        int survival_score_blue = starting_players_red - curr_red_count;
        int survival_score = survival_score_red + survival_score_blue;

        // Get player OPR
        foreach ( RobotID currbot in allRobotID.Values)
        {
            // If currbot is dead, then skip
            if( currbot == null) { continue; }

            // If dead, move on
            if(currbot.GetUserBool("DEAD")) { continue; }
           
            // If not present in GLOBALS, there something odd is off... we'll just ignore for now
            if(!GLOBALS.client_names.ContainsKey(currbot.id)) { continue; }

            if (lms_settings.FREE_FOR_ALL) // If free-for-all, get current players score
            {
                player_opr[GLOBALS.client_names[currbot.id]] = survival_score + currbot.GetUserFloat("KILLS");
            }
            else if( currbot.is_red) // If part of red, get the player's red score
            {
                player_opr[GLOBALS.client_names[currbot.id]] = survival_score_red + currbot.GetUserFloat("KILLS");
            }
            else // Otherwise get the player's blue score
            {
                player_opr[GLOBALS.client_names[currbot.id]] = survival_score_blue + currbot.GetUserFloat("KILLS");
            }
        }

        // Display player opr
        start_players.Sort(SortByOPR);

        textScores.text = "<b>Rank:</b>";

        for( int i =0; i < start_players.Count; i++)
        {
            int curr_id = start_players[i];

            // Make sure this player is present in allRobotID
            if ( ! allRobotID.ContainsKey(curr_id)) { continue; }

            // Make sure this is also present in client_names
            if( !GLOBALS.client_names.ContainsKey(curr_id)) { continue; }

            string color_of_id = "<color=white>";
            if(!lms_settings.FREE_FOR_ALL)
            {
                if( allRobotID[start_players[i]].is_red)
                {
                    if (allRobotID[curr_id].GetUserBool("DEAD"))
                    {
                        color_of_id = "<color=#905050>";
                    }
                    else
                    {
                        color_of_id = "<color=red>";
                    }
                }
                else
                {
                    if (allRobotID[curr_id].GetUserBool("DEAD"))
                    {
                        color_of_id = "<color=#505090>";
                    }
                    else
                    {
                        color_of_id = "<color=blue>";
                    }
                }
            }

            textScores.text += "\n<b>" + (i + 1) + "</b>) " + color_of_id + GLOBALS.client_names[curr_id] + "</color>=" + player_opr[GLOBALS.client_names[curr_id]];
        }

        // Generate the game-over code 
        gameovercode = GetGameOverCode();

        // If this is a teams game, show teams score
        if(lms_settings.FREE_FOR_ALL)
        {
            // Turn off score display
            if( redtextobj ) { redtextobj.SetActive(false); }
            if (bluetextobj) { bluetextobj.SetActive(false); }
            score_red = 0;
            score_blue = 0;

            // See if we are down to our last player
            if( (curr_red_count + curr_blue_count <= 1) && !GLOBALS.SINGLEPLAYER_MODE )
            {
                force_game_end = true;
            }
        }
        else
        {
            if (redtextobj) { redtextobj.SetActive(true); }
            if (bluetextobj) { bluetextobj.SetActive(true); }

            // See if end of game should occur
            if( ((curr_red_count <= 0) || (curr_blue_count <= 0)) && !GLOBALS.SINGLEPLAYER_MODE)
            {
                force_game_end = true;
            }
        }
    }


    override public void FieldChangedTrigger()
    {
        // Update robot colors if their positions changed
        SetRobotColors();

        // Move any live players who disconnected into dead players
        for( int i=live_players.Count-1; i >= 0; i--)
        {
            int currid = live_players[i];

            if( !allRobotID.ContainsKey(currid))
            {
                dead_players.Add(currid);
                live_players.RemoveAt(i);
            }
        }
    }


    public override int GetRedScore()
    {
        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }
        score_red = red_kills + (starting_players_blue - curr_blue_count);

        return score_red + score_redadj;
    }

    public override int GetBlueScore()
    {
        if (timerstate == TimerState.FINISHED)
        { return score_bluefinal; }

        score_blue = blue_kills + (starting_players_red - curr_red_count);

        return (int)score_blue + score_blueadj;
    }



    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

        // Add our settings
        serverFlags["SCOREText"] = textScores.text;

        // Get our variable states

        // First send if the score displays are on/off (are we in free for all or teams?)
        string varstates = (redtextobj && redtextobj.activeSelf) ? "1" : "0";
        varstates += ":" + ((bluetextobj && bluetextobj.activeSelf) ? "1" : "0");
        varstates += ":" + ((gameoverpanel && gameoverpanel.activeSelf) ? ("1:" + gameovercode) : "0:");
        serverFlags["LMSState"] = varstates;

        // Note: the "Death" state of the clients is sent via client specific flags
    }

    // Receiveflags from server
    private int mystate_old = -1;
    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

        // Score text (lot of data here)...
        if (serverFlags.ContainsKey("SCOREText"))
        {
            textScores.text = serverFlags["SCOREText"];
        }

        // Decode our states
        if( serverFlags.ContainsKey("LMSState"))
        {
            string[] states = serverFlags["LMSState"].Split(':');
            
            // Make sure the correct amount was found
            if( states.Length < 4) { return; }

            // Get redtextobj
            if (redtextobj)     { redtextobj.SetActive((states[0][0] == '1') ? true : false); }
            if (bluetextobj)    { bluetextobj.SetActive((states[1][0] == '1') ? true : false); }
            bool gameoveractive = (states[2][0] == '1') ? true : false;

            // Clear escape key was pressed flag so that it doesn't override the panel from displaying next time
            if( !gameoveractive)
            {
                escape_was_pressed = false;
            }

            // Turn off game-over panel if it is off
            if( gameoverpanel.activeSelf && !gameoveractive)
            {
                gameoverpanel.GetComponent<Animator>().enabled = false;
                gameoverpanel.SetActive(false);
            }

            // Play game over animation if this just turned on
            if(!escape_was_pressed && !gameoverpanel.activeSelf && gameoveractive)
            {
                gameoverpanel.SetActive(true);
                gameoverpanel.GetComponent<Animator>().enabled = true;
                gameoverpanel.GetComponent<Animator>().Play("GameOver");
                CreateGameOverText(states[3]);
            }
        }

        // Get our "death" state
        if( GLOBALS.CLIENT_MODE & GLOBALS.topclient)
        {
            mystate = (GLOBALS.topclient.serverFlags.ContainsKey("DEAD")) ? int.Parse(GLOBALS.topclient.serverFlags["DEAD"]) : 0;

            // If our state changed, make sure to reset camera
            if( mystate != mystate_old)
            {
                mystate_old = mystate;
                OnCameraViewChanged();
                GLOBALS.topclient.DoCameraViewChanged();
            }
        }
    }

    // Force only last-man-standing robot to be used
    public override string CorrectRobotChoice(string requested_robot)
    {
        return "Bot Royale";
    }

    // Find the appropriate free spot
    public override Transform CorrectRobotPosition(string requested_position, List<string> used_positions)
    {
        // Make sure init happened
        if( lms_settings == null)
        {
            Start();
        }

        // If number of spots exceeded, then exit
        // First free-for-all
        if( lms_settings.FREE_FOR_ALL && (used_positions.Count >= lms_settings.PLAYER_COUNT)) { return null; }
    
        // Next Red or blue max reached?
        if(!lms_settings.FREE_FOR_ALL)
        {
            int max_players = lms_settings.PLAYER_COUNT / 2;

            // Count relevant positions
            string which_side = requested_position.Contains("Red") ? "Red" : "Blue";
            int pos_count = 0;

            foreach( string currpos in used_positions)
            {
                if( currpos.Contains(which_side))
                {
                    pos_count++;
                }
            }

            if( pos_count >= max_players) { return null; }
        }

        // Next if this is free for all, don't be color sensitive
        if (lms_settings.FREE_FOR_ALL) { requested_position = "ALL"; }

        // Finally find a free spot
        return base.CorrectRobotPosition(requested_position, used_positions);
    }

    public override void AddPlayer(RobotInterface3D currbot, bool robot_is_new = true)
    {
        // Add the player
        base.AddPlayer(currbot, robot_is_new);

        // However, if robot is new and game is running, then put him in a dead state
        if( robot_is_new &&  (timerstate == TimerState.RUNNING) )
        {
            // Mark as dead
            currbot.SetKinematic();
            currbot.DisableTopObjects();
            currbot.myRobotID.SetUserBool("DEAD");
        }
    }

    public string GetGameOverCode()
    {
        // NOTE: Player names are restricted to alphanumeric, no punctuations, thus it won't screw up our seperators
        string output = "";

        // Free for all state
        output += (lms_settings.FREE_FOR_ALL) ? "1" : "0";
        output += ".";

        // Top live player (only used by free-for-all)
        output += (live_players.Count == 1) ? GLOBALS.client_names[live_players[0]] : "";
        output += ".";

        // Following top 2 dead players
        output += (dead_players.Count > 0) ? GLOBALS.client_names[dead_players[dead_players.Count - 1]] : "";
        output += ".";
        output += (dead_players.Count > 1) ? GLOBALS.client_names[dead_players[dead_players.Count - 2]] : "";
        output += ".";

        // Next send top 3 OPRs (index 4,5,6)
        output += (start_players.Count > 0) ? GLOBALS.client_names[start_players[0]] : "";
        output += ".";

        output += (start_players.Count > 1) ? GLOBALS.client_names[start_players[1]] : "";
        output += ".";

        output += (start_players.Count > 2) ? GLOBALS.client_names[start_players[2]] : "";

        return output;
    }

    // Set the game text based on the string
    public void CreateGameOverText(string gameovercode)
    {
        // Clear out old text 
        gameover_text.text = "";

        // Split up gamecode
        string[] codesin = gameovercode.Split('.');
        gameover_text.text = "";

        // Make sure correct number was found
        if ( codesin.Length < 7) { return; }

        // Extract data
        bool freeforall = codesin[0][0] == '1';


        // Create message
        if ( freeforall)
        {
            // Update the game-over text (in case the game is ended abruptly)
            gameover_text.text = "<b><size=+10>Last Robot Standing</size></b>\n";
            gameover_text.text += "<size=+10> 1st Place: " + ((codesin[1].Length > 0) ? codesin[1] : "No Survivors?!") + "</size>\n";
            if (codesin[2].Length > 0)
            {
                gameover_text.text += "  2nd Place: " + codesin[2] + "\n";
            }
            if (codesin[3].Length > 0)
            {
                gameover_text.text += "  3rd Place: " + codesin[3] + "\n";
            }

            gameover_text.text += "\n\n<b><size=+10>Overall Score</size></b>\n";
            gameover_text.text += "<size=+10> 1st Place: " + ((codesin[4].Length > 0) ? codesin[4] : "No Survivors?!") + "</size>\n";
            if (codesin[5].Length > 0)
            {
                gameover_text.text += "  2nd Place: " + codesin[5] + "\n";
            }
            if (codesin[6].Length > 0)
            {
                gameover_text.text += "  3rd Place: " + codesin[6] + "\n";
            }
        }
        else
        {
            // Update the game text in case game ends abruptly
            gameover_text.text = "<b><size=+10>Top Individual Scores</size></b>\n";
            gameover_text.text += "<size=+10> 1st Place: " + ((codesin[4].Length > 0) ? codesin[4] : "No Survivors?!") + "</size>\n";
            if (codesin[5].Length > 0)
            {
                gameover_text.text += "  2nd Place: " + codesin[5] + "\n";
            }
            if (codesin[6].Length > 0)
            {
                gameover_text.text += "  3rd Place: " + codesin[6] + "\n";
            }
        }
    }

    public override bool CorrectFieldElement(GameObject currobj)
    {
        if( base.CorrectFieldElement(currobj) )
        {
            // Stop game element from marking it as an active ball
            LSM_BallCollision balldata = currobj.GetComponent<LSM_BallCollision>();

            // If this isn't a ball, ignore
            if( !balldata) { return true; }

            balldata.DeactiveBall();

            return true;
        }

        return false;
    }


    // Let scorer know a user requested a camera change, and if so, change it accordingly.
    override public void OnCameraViewChanged()
    {
        // Don't restrict in single player mode
        if( GLOBALS.SINGLEPLAYER_MODE) { return; }

        // Otherwise in Client mode, make dead people spectators, and living ones follow
        GLOBALS.camera_follows = (mystate == 0);
    }

    // If a server interrupt is sent under the key "SCORER", this function will be called
    override public void OnScorerInterrupt(string msg)
    {
        if( msg == "CF")
        {
            ConfigureField();
        }
    }

    // Don't use the game start overlay - it only supports up to 3 vs 3
    public override bool UseGameStartOverlay()
    {
        return false;
    }
}
