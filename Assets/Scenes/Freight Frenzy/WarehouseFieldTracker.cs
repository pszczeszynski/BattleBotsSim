using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WarehouseFieldTracker : GenericFieldTracker
{
    Scorekeeper_FreightFrenzy ff_scorer;

    protected override void Start()
    {
        base.Start();

        // Find our scorekeeper
        ff_scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper_FreightFrenzy>();
    }


    // Gets called when a game element is exited
    public override void OnGameElementTriggerExit(gameElement ge)
    {
        // Only deal with GE marked as "in"
        if (ge.note2 != "in")
        {
            return;
        }

        //  Mark we no longer hold on to the GE is all of it's parts have exited
        if (!IsGameElementInside(ge) )
        {
            // If GE is not held by a robot, let scorer know the GE left 
            if (ff_scorer && (ge.held_by_robot==0)) 
            {
                ff_scorer.GameElementLeftWarehouse(ge, this);
            }
        }
    }

    // When a part of a GE enters our field, mark it that it's inside our control
    public override void OnGameElementTriggerEnter(gameElement ge)
    {
        // Any gameElements marked as "in" need to be tracked by this
        if (ge.note2 == "in")
        {
            ge.tracker = this;
        }
    }
}
