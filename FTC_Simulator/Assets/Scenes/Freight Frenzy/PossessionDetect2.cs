using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Original PossessionDetect creates a foul every 5s you exceed limit, and doesn't scale it with the number of items.
// This one creates only a 1 time foul, and creates a foul for every ball held longer than ~3s that exceeds capacity.
public class PossessionDetect2 : MonoBehaviour
{
    // List of GenericFieldTrackers inside bot to keep sum
    public PossessionTracker[] all_trackers;
    public int limit_num = 1; // if number of GE exceeds this, it will start counting how long
    public int curr_count = 0;  // The number of GE currently
    private float time_of_fault = 0;
    Scorekeeper scorer;
    public RobotID self_ri;
    private Dictionary<gameElement, float> master_list = new Dictionary<gameElement, float>(); // List of unique balls and the time they entered


    // Start is called before the first frame update
    void Start()
    {
        // Find scorer if not present
        if (!scorer)
        {
            // Find our scorekeeper
            scorer = GameObject.FindObjectOfType<Scorekeeper>();
        }

        // Initialize trackers with my ID
        foreach ( PossessionTracker currtracker in all_trackers)
        {
            if (currtracker)
            {
                currtracker.master2 = this;
            }
        }

        // Initialize our self_id
        self_ri = GetComponentInParent<RobotID>();
    }

    // Update is called once per frame
    void Update()
    {
        // Update count
        curr_count = GetGameElementCount();

        // Ensure that there are at least limit_num of balls that are ok
        while ((GetBallCountThatCanFault() > 0) && (curr_count - GetBallCountThatCanFault()) < limit_num)
        {
            ResetOldestBall();
        }
    }
 


    public int GetGameElementCount()
    {
        // Get all the unique gamelements in all the trackers
        List<gameElement> allballs = new List<gameElement>();

        foreach (GenericFieldTracker curr_tracker in all_trackers)
        {
            // Iterate through every element in current tracker
            foreach (gameElement curr_element in curr_tracker.game_elements)
            {
                if( allballs.Contains(curr_element)) { continue; }

                allballs.Add(curr_element);
            }
        }

        // Remove any elements in the tracker that don't exist, add elements into the tracker that are new
        // If they exceed the limit, set their time to current time
        // If the are below limit, make them not able to get a foul

        // First remove balls
        List<gameElement> toremove = new List<gameElement>();

        foreach (gameElement curr_ball in master_list.Keys)
        {
            if( !allballs.Contains(curr_ball) )
            {
                toremove.Add(curr_ball);
            }
        }

        foreach ( gameElement currelement in toremove)
        {
            master_list.Remove(currelement);
        }

        // Next add new ones
        foreach( gameElement curr_ball in allballs)
        {
            if( !master_list.ContainsKey(curr_ball))
            {
                // If limit hasn't been reached, add it but dont allow it to fault
                if( master_list.Count < limit_num )
                {
                    master_list.Add(curr_ball, Time.time + 999f);
                }
                else
                {
                    master_list.Add(curr_ball, Time.time);
                }
            }
        }

        return master_list.Count;
    }

    // Returns the number of balls that are inside for >= the time_fault, and resets them
    public int GetFaultCount( float time_fault)
    {
        int faults = 0;
        // First remove balls
        List<gameElement> keyList = new List<gameElement>(master_list.Keys);

        foreach (gameElement curr_ball in keyList)
        {
            if(Time.time - master_list[curr_ball] >= time_fault )
            {
                faults += 1;
                master_list[curr_ball] = Time.time + 999f;
            }
        }

        return faults;
    }

    // Returns the number of balls that are inside for >= the time_fault, and resets them
    public int GetBallCountThatCanFault()
    {
        int faults = 0;
        // First remove balls
        List<gameElement> keyList = new List<gameElement>(master_list.Keys);

        foreach (gameElement curr_ball in keyList)
        {
            if (Time.time - master_list[curr_ball] >= 0f)
            {
                faults += 1;
            }
        }

        return faults;
    }

    // Resets counting on the oldest ball
    public void ResetOldestBall()
    {
        float oldest_time = -0.001f;
        gameElement oldest_ball = null;

        // First remove balls
        List<gameElement> keyList = new List<gameElement>(master_list.Keys);

        foreach (gameElement curr_ball in keyList)
        {
            if (Time.time - master_list[curr_ball] >= oldest_time)
            {
                oldest_time = Time.time - master_list[curr_ball];
                oldest_ball = curr_ball;
            }
        }

        if( oldest_ball)
        {
            master_list[oldest_ball] = Time.time + 999f;
        }
    }

    // Gets called when a game element has exited any of our trackers
    public void OnGameElementTriggerExit(gameElement ge)
    {
      // Nothing to do here.

    }
}
