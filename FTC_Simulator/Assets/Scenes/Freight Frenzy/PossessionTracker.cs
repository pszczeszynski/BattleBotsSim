using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PossessionTracker : GenericFieldTracker
{
    public PossessionDetect master;
    public PossessionDetect2 master2;


    // Gets called when a game element is exited
    public override void OnGameElementTriggerExit(gameElement ge)
    {
        if (master)
        {
            master.OnGameElementTriggerExit(ge);
        }
        else if (master2)
        {
            master2.OnGameElementTriggerExit(ge);
        }
    }

    // When a part of a GE enters our cube, mark it as owned by us
    public override void OnGameElementTriggerEnter(gameElement ge)
    {
        // Mark game-element as carried by us
        if (self_ri)
        {
            ge.held_by_robot = self_ri.id;
        }

    }

}
