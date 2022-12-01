using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TarmacTracker : GenericFieldTracker
{
    public List<RobotID> starting_robots = new List<RobotID>();

    // Record all robots inside 
    public void MarkStartingRobots()
    {
        starting_robots.Clear();

        foreach( RobotID currbot in robots)
        {
            starting_robots.Add(currbot);
        }
    }

    // Mark robots that exited (the scorekeeper will only poll this during auto)
    public override void RobotExited(RobotID robotid)
    {
        // Make sure this robotid wasn't destroyed
        if( robotid == null ) { return; }

        if( starting_robots.Contains(robotid))
        {
            robotid.SetUserBool("Taxi", true);
        }
    }
}
