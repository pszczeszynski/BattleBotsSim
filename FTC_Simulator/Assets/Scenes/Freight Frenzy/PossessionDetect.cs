using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PossessionDetect : MonoBehaviour
{
    // List of GenericFieldTrackers inside bot to keep sum
    public PossessionTracker[] all_trackers;
    public int limit_num = 1; // if number of GE exceeds this, it will start counting how long
    public int curr_count = 0;  // The number of GE currently
    private float time_of_fault = 0;
    Scorekeeper scorer;
    public RobotID self_ri;

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
                currtracker.master = this;
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

        // If we exceed limit, count for how long
        if( curr_count > limit_num)
        {
            if (time_of_fault <= 0)
            {
                time_of_fault = Time.time;
            }
        }
        else
        {
            time_of_fault = 0;
        }
    }


    // Retrieve fault duration. 0 = no fault
    public float GetFaultDuration()
    {
        if (time_of_fault <= 0) { return 0; }

        return Time.time - time_of_fault;
    }

    // Resets fault timer to current start time (if in fault)
    // delta_time: offset time_of_fault by this amount. A positive number moves the start of fault into the future
    public void ResetFault(float delta_time = 0)
    {
        if (curr_count > limit_num)
        {
            time_of_fault = Time.time + delta_time;           
        }
        else
        {
            time_of_fault = 0;
        }
    }

    
    // Returns true if possesion limit exceeded 
    public bool IsFault()
    {
        return curr_count > limit_num;
    }

    private Dictionary<gameElement, bool> master_list = new Dictionary<gameElement, bool>();

    public int GetGameElementCount()
    {
        int total_elements = 0;
        int trackers_with_items = 0;

        // First get the raw number of trackers, not accounting for duplicates
        foreach( GenericFieldTracker curr_tracker in all_trackers)
        {
            if( !curr_tracker) { continue; }
            int count = curr_tracker.GetGameElementCount();
            total_elements += count;

            if( count > 0 )
            {
                trackers_with_items += 1;
            }
        }

        // If there are <2 elements or there is only 1 tracker or only 1 tracker has items, return the number
        // This covers 99% of the cases
        if( (total_elements <= 1) || (all_trackers.Length <=1) || (trackers_with_items <= 1)) { return total_elements;  }

        // In this case we need to deal with duplicates. 
        // Create a master list of game items, ignoring duplicates
        master_list.Clear();

        foreach( GenericFieldTracker curr_tracker in all_trackers)
        {
            // Iterate through every element in current tracker
            foreach( gameElement curr_element in curr_tracker.game_elements)
            {
                master_list[curr_element] = true;   
            }
        }

        return master_list.Count;
    }

    // Gets called when a game element has exited any of our trackers
    public void OnGameElementTriggerExit(gameElement ge)
    {
        foreach( PossessionTracker curr_tracker in all_trackers)
        {
            if( !curr_tracker) { continue; }

            if(  curr_tracker.IsGameElementInside(ge) )
            {
                return;
            }
        }

        ge.held_by_robot = 0;

        // If  the ge isn't inside it's own tracker, then call scorer
        if (!ge.tracker || ge.tracker.IsGameElementInside(ge))
        {
            return;
        }

        // Let scorer know the cube exited
        if (scorer && self_ri) // Find scorer if not already found
        {
            scorer.RobotDroppedItem(self_ri.id, ge);
        }
        

    }
}
