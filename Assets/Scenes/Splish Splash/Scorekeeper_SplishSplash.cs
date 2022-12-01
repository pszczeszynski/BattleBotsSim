using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;

public class Scorekeeper_SplishSplash : Scorekeeper {

    // all field elements
    private GameObject[] fieldElements;

    private double score_red;
    private double score_blue;

    private void Awake()
    {
        GLOBALS.PlayerCount = 4;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 0;
        GLOBALS.TIMER_ENDGAME = 0;
    }


    public override void ScorerInit()
    {
        // Find all the field element balls
        fieldElements = GameObject.FindGameObjectsWithTag("GameElement");
    }
 
    public override void ScorerUpdate(bool last_frame = false)
    {
        CalculateScores(timer_elapsed);
    }

    public override void ScorerReset()
    {
        base.ScorerReset();

        score_red = 0;
        score_blue = 0;
    }

    override public void OnTimerStart()
    {
        base.OnTimerStart();
        ScorerReset();
    }


    private void CalculateScores(TimeSpan elapsed_time)
    {
        foreach (GameObject currobj in fieldElements)
        {
            if (currobj.transform.localPosition.x < 0)
            {
                if (currobj.GetComponent<gameElement>().type == ElementType.Cube)
                {
                    score_red += 0.5f * elapsed_time.TotalMilliseconds / 1000f;
                }
                else if (currobj.GetComponent<gameElement>().type == ElementType.Jewel)
                {
                    score_red += 1.5f * elapsed_time.TotalMilliseconds / 1000f;
                }
            }
            else
            {
                if (currobj.GetComponent<gameElement>().type == ElementType.Cube)
                {
                    score_blue += 0.5f * elapsed_time.TotalMilliseconds / 1000f;
                }
                else if (currobj.GetComponent<gameElement>().type == ElementType.Jewel)
                {
                    score_blue += 1.5f * elapsed_time.TotalMilliseconds / 1000f;
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

}
