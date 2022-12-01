using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_RelicRecovery : Scorekeeper {

    // Red Box1 [row,column]
    CubeScoring[,] RedBox1 = new CubeScoring[4, 3];
    CubeScoring[,] RedBox2 = new CubeScoring[4, 3];
    CubeScoring[,] BlueBox1 = new CubeScoring[4, 3];
    CubeScoring[,] BlueBox2 = new CubeScoring[4, 3];

    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 0;
        GLOBALS.TIMER_ENDGAME = 30;
    }

    override public void OnTimerStart()
    {
        base.OnTimerStart();
        ScorerReset();
    }

    public override void ScorerReset()
    {
        base.ScorerReset();


    }

    // Rules variables

    public override void ScorerInit()
    {
        // Populate all the boxes
        FindScoringElements("r1", RedBox1);
        FindScoringElements("r2", RedBox2);
        FindScoringElements("b1", BlueBox1);
        FindScoringElements("b2", BlueBox2);
    }

   
    private void FindScoringElements(string prefix, CubeScoring[,] Box)
    {
        // Go through rows
        for (int i = 0; i < 4; i++)
        {
            // Go through columns
            for (int j = 0; j < 3; j++)
            {
                GameObject curr = GameObject.Find(prefix + "_r" + (i + 1) + "_" + (j + 1));
                if (curr == null)
                {
                    Debug.LogError("Unable to find box scorring element for " + prefix + "_r" + (i + 1) + "_" + (j + 1));
                    continue;
                }

                Box[i, j] = curr.GetComponent<CubeScoring>();
            }
        }
    }

    private int FindBoxScore(CubeScoring[,] Box)
    {
        int score = 0;

        // Count cubes
        for (int i = 0; i < 4; i++)
        {
            int curr_row_count = 0;

            gameElement previous_cube = null;
            // Go through columns
            for (int j = 0; j < 3; j++)
            {
                if (Box[i, j].IsCube(previous_cube))
                {
                    score += 2;
                    curr_row_count++;
                }
            }

            // Add row bonus
            if (curr_row_count >= 3)
            {
                score += 10;
            }
        }

        // Count columns
        // One cube can spawn columns, but can only count it once per columns
        // Will ASSUME most of the time, only 1 cube can occupy the lowest row, thus will
        // count up ignoring the cube in the lower row
        int num_of_cols = 0;
        String cypher = "";

        // Column number
        for (int j = 0; j < 3; j++)
        {
            int curr_col_count = 0;

            gameElement previous_cube = null;
            // Row number
            for (int i = 0; i < 4; i++)
            {
                if (previous_cube = Box[i, j].IsCube(previous_cube))
                {
                    curr_col_count++;
                    if (previous_cube.type == ElementType.Cube)
                    { cypher += "W"; }
                    else
                    { cypher += "B"; }

                }
            }

            if (curr_col_count >= 4)
            {
                score += 20;
                num_of_cols++;
            }
        }

        // Check Cipher
        if (num_of_cols >= 3)
        {
            switch (cypher)
            {
                // FROG
                case "WBWBBWBWWBWB":
                case "BWBWWBWBBWBW":
                // Bird
                case "WBBWBWWBWBBW":
                case "BWWBWBBWBWWB":
                //Snake
                case "BBWWBWWBWWBB":
                case "WWBBWBBWBBWW":
                    score += 30;
                    break;
                default:
                    // no cypher
                    break;
            }
        }

        return score;
    }

    public override int GetRedScore()
    {
        int score = 0;

        if (timerstate == TimerState.FINISHED)
        { return score_redfinal; }

        score += FindBoxScore(RedBox1);
        score += FindBoxScore(RedBox2);

        return score + score_redadj;
    }

    public override int GetBlueScore()
    {
        int score = 0;

        if (timerstate == TimerState.FINISHED)
        { return score_bluefinal; }

        score += FindBoxScore(BlueBox1);
        score += FindBoxScore(BlueBox2);

        return score + score_blueadj;
    }
}
