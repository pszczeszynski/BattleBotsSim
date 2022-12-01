using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_ChangeUp : Scorekeeper {

    // Game specific settings
    ChangeUp_Settings cu_settings = null;
    ClientLow client = null;

    // all scorers
    public CU_scoringBox scorer_tl;
    public CU_scoringBox scorer_tm;
    public CU_scoringBox scorer_tr;
    public CU_scoringBox scorer_l;
    public CU_scoringBox scorer_m;
    public CU_scoringBox scorer_r;
    public CU_scoringBox scorer_bl;
    public CU_scoringBox scorer_bm;
    public CU_scoringBox scorer_br;

    // Lines for display
    public MeshRenderer line_r1;
    public MeshRenderer line_r2;
    public MeshRenderer line_r3;
    public MeshRenderer line_r4;
    public MeshRenderer line_r5;
    public MeshRenderer line_r6;
    public MeshRenderer line_r7;
    public MeshRenderer line_r8;


    // Blocking push-back
    public GameObject pushback;

    public Material color_red;
    public Material color_blue;

    public float score_red = 0;
    public float score_blue = 0;

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
        if( settings ) {  cu_settings = settings.GetComponent<ChangeUp_Settings>(); }

        GameObject theclient = GameObject.Find("Client");
        if( theclient ) { client = theclient.GetComponent<ClientLow>(); }

        // Initialize Rules
        // No rules yet
    }

    public override void ScorerReset()
    {
        base.ScorerReset();

        red_balls = 0;
        blue_balls = 0;
        red_rows = 0;
        blue_rows = 0;
        red_auto = 0;
        blue_auto = 0;

        blue_winpoint = false;
        red_winpoint = false;
    }

    public override void Restart()
    {
        base.Restart();

        // If this game mode is option 3 in single player, then change timer to 1m
        if( GLOBALS.SINGLEPLAYER_MODE && GLOBALS.topsingleplayer && (GLOBALS.game_option==3))
        {
            GLOBALS.TIMER_TOTAL = 60;
            GLOBALS.TIMER_AUTO = 0;
        }
        else
        {
            GLOBALS.TIMER_TOTAL = 120;
            GLOBALS.TIMER_AUTO = 15;
        }

        // Deactivate pushback
        pushback.SetActive(false);

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
     
    }

    // Function triggered when new robots show up on the field or are updated
    public override void FieldChangedTrigger()
    {

    }

    public override void GetScoreDetails(Dictionary<string, string> data)
    {
        // Have base add timer and game state
        base.GetScoreDetails(data);

        data["ScoreR"] = ((int)score_red + score_redadj).ToString();
        data["ScoreB"] = ((int)score_blue + score_blueadj).ToString();

        // Add this game specific data
        data["RBalls"] = red_balls.ToString();
        data["BBalls"] = blue_balls.ToString();
        data["AutoR"] = ((int)red_auto).ToString();
        data["AutoB"] = ((int)blue_auto).ToString();
        data["RRows"] = red_rows.ToString();
        data["BRows"] = blue_rows.ToString();
        data["RedWP"] = (red_winpoint) ? "1" : "0";
        data["BlueWP"] = (blue_winpoint) ? "1" : "0";
    }

    public int red_balls = 0;
    public int blue_balls = 0;
    public int red_rows = 0;
    public int blue_rows = 0;
    public int red_auto = 0;
    public int blue_auto = 0;
    public bool blue_winpoint = false;
    public bool red_winpoint = false;

    private void CalculateScores()
    {
        // Don't update score if finished.. if "read"y then clear it..
        if(timerstate != TimerState.RUNNING)
        { return;  }

        // Make sure settings have loaded
        if(!cu_settings) { return;  }

        // Count all the balls first
        red_balls = scorer_tl.GetRedCount() + scorer_tm.GetRedCount() + scorer_tr.GetRedCount() +
                    scorer_l.GetRedCount() + scorer_m.GetRedCount() + scorer_r.GetRedCount() +
                    scorer_bl.GetRedCount() + scorer_bm.GetRedCount() + scorer_br.GetRedCount();

        blue_balls = scorer_tl.GetBlueCount() + scorer_tm.GetBlueCount() + scorer_tr.GetBlueCount() +
            scorer_l.GetBlueCount() + scorer_m.GetBlueCount() + scorer_r.GetBlueCount() +
            scorer_bl.GetBlueCount() + scorer_bm.GetBlueCount() + scorer_br.GetBlueCount();

        // Calculate rows next
        int tl = scorer_tl.GetHighestBall();
        int tm = scorer_tm.GetHighestBall();
        int tr = scorer_tr.GetHighestBall();
        int l = scorer_l.GetHighestBall();
        int m = scorer_m.GetHighestBall();
        int r = scorer_r.GetHighestBall();
        int bl = scorer_bl.GetHighestBall();
        int bm = scorer_bm.GetHighestBall();
        int br = scorer_br.GetHighestBall();

        red_rows = 0;
        blue_rows = 0;

        line_r1.enabled = false;
        line_r2.enabled = false;
        line_r3.enabled = false;
        line_r4.enabled = false;
        line_r5.enabled = false;
        line_r6.enabled = false;
        line_r7.enabled = false;
        line_r8.enabled = false;
        bool enable_lines = cu_settings.ENABLE_LINES || cu_settings.ENABLE_LINES_SPEC;

        // Row 1
        if ((tl == tm) && (tl == tr) && (tl == 1))  
        { 
            red_rows++;
            Destroy(line_r1.material);
            line_r1.material = color_red;
            line_r1.enabled = enable_lines;
        }
        if ((tl == tm) && (tl == tr) && (tl == 2)) 
        { 
            blue_rows++;
            Destroy(line_r1.material);
            line_r1.material = color_blue;
            line_r1.enabled = enable_lines;
        }

        // Row 2
        if ((l == m) && (l == r) && (l == 1)) 
        { 
            red_rows++;
            Destroy(line_r2.material);
            line_r2.material = color_red;
            line_r2.enabled = enable_lines;
        }
        if ((l == m) && (l == r) && (l == 2)) 
        { 
            blue_rows++;
            Destroy(line_r2.material);
            line_r2.material = color_blue;
            line_r2.enabled = enable_lines;
        }

        // Row 3
        if ((bl == bm) && (bl == br) && (bl == 1)) 
        { 
            red_rows++;
            Destroy(line_r3.material);
            line_r3.material = color_red;
            line_r3.enabled = enable_lines;
        }
        if ((bl == bm) && (bl == br) && (bl == 2)) 
        { 
            blue_rows++;
            Destroy(line_r3.material);
            line_r3.material = color_blue;
            line_r3.enabled = enable_lines;
        }

        // Row 4
        if ((tl == l) && (tl == bl) && (tl == 1)) 
        { 
            red_rows++;
            Destroy(line_r4.material);
            line_r4.material = color_red;
            line_r4.enabled = enable_lines;
        }
        if ((tl == l) && (tl == bl) && (tl == 2)) 
        { 
            blue_rows++;
            Destroy(line_r4.material);
            line_r4.material = color_blue;
            line_r4.enabled = enable_lines;
        }

        // Row 5
        if ((tm == m) && (tm == bm) && (tm == 1)) 
        { 
            red_rows++;
            Destroy(line_r5.material);
            line_r5.material = color_red;
            line_r5.enabled = enable_lines;
        }
        if ((tm == m) && (tm == bm) && (tm == 2)) 
        { 
            blue_rows++;
            Destroy(line_r5.material);
            line_r5.material = color_blue;
            line_r5.enabled = enable_lines;
        }

        // Row 6
        if ((tr == r) && (tr == br) && (tr == 1)) 
        { 
            red_rows++;
            Destroy(line_r6.material);
            line_r6.material = color_red;
            line_r6.enabled = enable_lines;
        }
        if ((tr == r) && (tr == br) && (tr == 2)) 
        { 
            blue_rows++;
            Destroy(line_r6.material);
            line_r6.material = color_blue;
            line_r6.enabled = enable_lines;
        }

        // Row 7
        if ((tl == m) && (tl == br) && (tl == 1)) 
        { 
            red_rows++;
            Destroy(line_r7.material);
            line_r7.material = color_red;
            line_r7.enabled = enable_lines;
        }
        if ((tl == m) && (tl == br) && (tl == 2)) 
        { 
            blue_rows++;
            Destroy(line_r7.material);
            line_r7.material = color_blue;
            line_r7.enabled = enable_lines;
        }

        // Row 8
        if ((tr == m) && (tr == bl) && (tr == 1)) 
        { 
            red_rows++;
            Destroy(line_r8.material);
            line_r8.material = color_red;
            line_r8.enabled = enable_lines;
        }
        if ((tr == m) && (tr == bl) && (tr == 2)) 
        { 
            blue_rows++;
            Destroy(line_r8.material);
            line_r8.material = color_blue;
            line_r8.enabled = enable_lines;
        }

        // Score before auto
        score_red = red_balls + 6 * red_rows;
        score_blue = blue_balls + 6 * blue_rows;

        // Determine auto score
        if(!GLOBALS.SINGLEPLAYER_MODE && (time_total.TotalSeconds > (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO)))
        {
            red_auto = 0;
            blue_auto = 0;
            if( score_red > score_blue) { red_auto = 6; }
            else if( score_blue > score_red) { blue_auto = 6; }
            else
            {
                red_auto = 3;
                blue_auto = 3;
            }
        }

        // Add auto back into score
        score_red += red_auto;
        score_blue += blue_auto;

        // Determine win-point
        if (time_total.TotalSeconds >= (GLOBALS.TIMER_TOTAL - GLOBALS.TIMER_AUTO))
        {
            blue_winpoint = line_r6.enabled && (tr == 2);
            red_winpoint = line_r4.enabled && (tl == 1);
        }

        // Now correct score for skills challenge
        if( GLOBALS.SINGLEPLAYER_MODE && GLOBALS.topsingleplayer && GLOBALS.game_option==3)
        {
            score_red = score_red - score_blue + 63;
            score_blue = 0;
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

    // Returns 0 if line is off, 1 if red, 2 if blue
    // 3 and 4 if the lines are disabled for single player
    private string LightToC(MeshRenderer light) 
    {
        if (!cu_settings || (!cu_settings.ENABLE_LINES && !cu_settings.ENABLE_LINES_SPEC)) { return "0"; }
        if (!light.enabled) { return "0"; }
        if( light.material.GetColor("_EmissionColor") == color_red.GetColor("_EmissionColor")) 
        {
            if (cu_settings.ENABLE_LINES) { return "1"; }
            return "3";
        }

        if (cu_settings.ENABLE_LINES) { return "2"; }
        return "4";
    }

    // Set the light to the propper value
    private void CToLight(MeshRenderer light, char state)
    {
        if (state == '0') { light.enabled = false; return; }

        light.enabled = true;
        if (state == '1') 
        {
            Destroy(light.material);
            light.material = color_red;
        }
        else if(state == '2')
        {
            Destroy(light.material);
            light.material = color_blue;
        }
        else if(state == '3' && client.isSpectator())
        {
            Destroy(light.material);
            light.material = color_red;
        }
        else if(state == '4' && client.isSpectator())
        {
            Destroy(light.material);
            light.material = color_blue;
        }
        else
        {
            light.enabled = false;
        }
    }

    // Add flags to send to clients to sync states
    public override void SendServerData(Dictionary<string, string> serverFlags)
    {
        base.SendServerData(serverFlags);

        // Add states of lines
        serverFlags["SK_LINES"] = LightToC(line_r1) +
                                    LightToC(line_r2) +
                                    LightToC(line_r3) +
                                    LightToC(line_r4) +
                                    LightToC(line_r5) +
                                    LightToC(line_r6) +
                                    LightToC(line_r7) +
                                    LightToC(line_r8);

        // add states of lights
        if (cu_settings && !cu_settings.ENABLE_LIGHTS) { serverFlags["SK_LIGHTS"] = "000000000"; }
        else
        {
            serverFlags["SK_LIGHTS"] = scorer_tl.GetHighestBall().ToString() +
                                       scorer_tm.GetHighestBall().ToString() +
                                       scorer_tr.GetHighestBall().ToString() +
                                       scorer_l.GetHighestBall().ToString() +
                                       scorer_m.GetHighestBall().ToString() +
                                       scorer_r.GetHighestBall().ToString() +
                                       scorer_bl.GetHighestBall().ToString() +
                                       scorer_bm.GetHighestBall().ToString() +
                                       scorer_br.GetHighestBall().ToString();
        }
    }

    // Receiveflags from server
    public override void ReceiveServerData(Dictionary<string, string> serverFlags)
    {
        base.ReceiveServerData(serverFlags);

        if( !serverFlags.ContainsKey("SK_LINES") || !serverFlags.ContainsKey("SK_LIGHTS"))
        { return; }

        string lines = serverFlags["SK_LINES"];
        if(lines.Length < 8) { return; }

        // Decode lines
        CToLight(line_r1, lines[0]);
        CToLight(line_r2, lines[1]);
        CToLight(line_r3, lines[2]);
        CToLight(line_r4, lines[3]);
        CToLight(line_r5, lines[4]);
        CToLight(line_r6, lines[5]);
        CToLight(line_r7, lines[6]);
        CToLight(line_r8, lines[7]);


        // Decode lights
        string lights = serverFlags["SK_LIGHTS"];
        if (lights.Length < 9) { return; }

        scorer_tl.SetHighestBall(lights[0] - '0');
        scorer_tm.SetHighestBall(lights[1] - '0');
        scorer_tr.SetHighestBall(lights[2] - '0');
        scorer_l.SetHighestBall(lights[3] - '0');
        scorer_m.SetHighestBall(lights[4] - '0');
        scorer_r.SetHighestBall(lights[5] - '0');
        scorer_bl.SetHighestBall(lights[6] - '0');
        scorer_bm.SetHighestBall(lights[7] - '0');
        scorer_br.SetHighestBall(lights[8] - '0');

    }
}