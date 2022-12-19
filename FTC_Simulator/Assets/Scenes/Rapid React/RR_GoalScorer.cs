using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RR_GoalScorer : GenericFieldTracker
{
    public bool mark_balls_invalid = true;

    // Track balls that have entered
    public List<ball_data> red_balls = new List<ball_data>();
    public List<ball_data> blue_balls = new List<ball_data>();

    // Reset count to balls contained inside
    public void ResetCountToBallsInside()
    {
        red_balls.Clear();
        blue_balls.Clear();

        foreach(gameElement ge in game_elements)
        {
            OnGameElementTriggerEnter(ge);
        }
    }

    // Check if a ball data element is inside
    public bool ContainsBall( ball_data ball)
    {
        return red_balls.Contains(ball) || blue_balls.Contains(ball);
    }

    public override void OnGameElementTriggerEnter(gameElement ge)
    {
        // Get the ball data
        ball_data curr_ball = ge.GetComponent<ball_data>();

        if( curr_ball == null ) { return; }

        // If ball is invalid, ignore it
        if( curr_ball.IsFlagSet("Invalid"))     { return;}

        // Invalidate the ball for the future
        if (mark_balls_invalid) { curr_ball.SetFlag("Invalid"); }
        
        // If this ball didn't pass through the funnel, then don't add it 
        if (!curr_ball.IsFlagSet("Funnel")) { return; }

        // If this is a ball add it to our list
        if (ge.type == ElementType.Blue1)
        {
            blue_balls.Add(curr_ball);
        }
        else if (ge.type == ElementType.Red1)
        {
            red_balls.Add(curr_ball);
        }


    }

    public override void Reset()
    {
        base.Reset();
        ClearData();
    }

    public void ClearData()
    {
        red_balls.Clear();
        blue_balls.Clear();
    }

    // Get number of red/blue balls
    public int GetRedBallCount()
    {
        return red_balls.Count;
    }

    public int GetblueBallCount()
    {
        return blue_balls.Count;
    }
    


}
