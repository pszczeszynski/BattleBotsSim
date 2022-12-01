using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TerminalManager : MonoBehaviour
{
    public List<GenericFieldTracker> robot_detectors = new List<GenericFieldTracker>();
    public List<GenericFieldTracker> ball_outs = new List<GenericFieldTracker>();
    public GenericFieldTracker inside_detector;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // If no balls inside, exit
        if( inside_detector.GetGameElementCount() < 1) { return; }

        // Check if any of the robot_detectors have a robot touching it
        for( int i =0; i < robot_detectors.Count; i++)
        {
            if (robot_detectors[i].IsAnyRobotInside())
            {              
                DeployBall(i);
            }
        }
    }

    // Deploys a ball based on first robot detected. If multiple robots are touching it, it ingores the others
    private void DeployBall(int index)
    {
        // Make sure there is some robot inside
        if( robot_detectors[index].robots.Count <=0 ) { return; }

        // Make sure detector has no ball inside it
        if( ball_outs[index].IsAnyGameElementInside()) { return; }

        RobotID curr_bot = robot_detectors[index].robots[0];

        // Get a ball
        gameElement out_ball = null;
        foreach( gameElement curr_ball in inside_detector.game_elements)
        {
            if( (curr_ball.type == ElementType.Blue1) && !curr_bot.is_red ||
                (curr_ball.type == ElementType.Red1) && curr_bot.is_red )
            {
                out_ball = curr_ball;
                break;
            }
        }

        // If no ball found, exit
        if( out_ball == null ) { return; }

        // Now move ball to it's starting location
        out_ball.transform.position = ball_outs[index].transform.position;

        // Reset the speed
        if (out_ball.GetComponent<Rigidbody>())
        {
            // Reset velocities
            Rigidbody mybody = out_ball.GetComponent<Rigidbody>();

            mybody.velocity = Vector3.zero;
            mybody.angularVelocity = Vector3.zero;
        }
    }
}
