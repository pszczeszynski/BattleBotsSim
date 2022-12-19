using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_RoverRockus : Scorekeeper {

    // all scorers
    public RR_scoringBox scorer_red_silver;
    public RR_scoringBox scorer_red_gold;
    public RR_scoringBox scorer_red_box;
    public RR_scoringBox scorer_blue_silver;
    public RR_scoringBox scorer_blue_gold;
    public RR_scoringBox scorer_blue_box;

    // Rule 1: no blocking access to Lander
    public GameObject rule1_r1;
    private RR_Rule1 r1_r1;
    public GameObject rule1_r2;
    private RR_Rule1 r1_r2;
    public GameObject rule1_b1;
    private RR_Rule1 r1_b1;
    public GameObject rule1_b2;
    private RR_Rule1 r1_b2;

    public GameObject redcrater;
    public GameObject bluecrater;
    public GameObject center_marker;

    private double score_red;
    private double score_blue;

    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 0;
        GLOBALS.TIMER_ENDGAME = 30;
    }


    // Rules variables



    public override void ScorerInit()
    {
        score_red = 0;
        score_blue = 0;

        // Initialize rules
        // Field planes are parents to the RR_rule objects. This is so their collision
        // component can be triggered and call the Rule collision callback.
        r1_r1 = rule1_r1.GetComponent<RR_Rule1>();
        r1_r1.robotid = null;
        r1_r1.goal = center_marker.transform;
        r1_r2 = rule1_r2.GetComponent<RR_Rule1>();
        r1_r2.robotid = null;
        r1_r2.goal = center_marker.transform;
        r1_b1 = rule1_b1.GetComponent<RR_Rule1>();
        r1_b1.robotid = null;
        r1_b1.goal = center_marker.transform;
        r1_b2 = rule1_b2.GetComponent<RR_Rule1>();
        r1_b2.robotid = null;
        r1_b2.goal = center_marker.transform;

        // Go through all the RobotID objects and assign them to the correct rule
        GameObject[] allrobots = GameObject.FindGameObjectsWithTag("Robot");

        foreach (GameObject current in allrobots )
        {
            RobotID currid = current.GetComponent<RobotID>();
            if( currid == null )
            { continue; }

            switch (currid.starting_pos)
            {
                case "Red Front":
                    r1_r1.robotid = currid;
                    break;

                case "Red Back":
                    r1_r2.robotid = currid;
                    break;

                case "Blue Front":
                    r1_b1.robotid = currid;
                    break;

                case "Blue Back":
                    r1_b2.robotid = currid;
                    break;
            }
        }
    }

    public override void ScorerReset()
    {
        base.ScorerReset();
        score_red = 0;
        score_blue = 0;
    }

    public override void ScorerUpdate(bool last_frame = false)
    {
        if (base.timerstate != TimerState.PAUSED &&
            base.timerstate != TimerState.RUNNING
            )
        {
            ScorerReset();
            return;
        }

        CalculateScores();
    }


    private void CalculateScores()
    {
        score_red = scorer_red_gold.GetElementCount() * 5f + scorer_red_silver.GetElementCount() * 5f + scorer_red_box.GetElementCount() * 2f;
        score_blue = scorer_blue_gold.GetElementCount() * 5f + scorer_blue_silver.GetElementCount() * 5f + scorer_blue_box.GetElementCount() * 2f;
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
