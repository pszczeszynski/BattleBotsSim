using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// Uses triggers to count how many gameElements are inside it.
// DuckChecker advances duck state machine as requested

public class DuckChecker : MonoBehaviour
{

    // list of game elements colliding  
    public ElementType filter_element = ElementType.Off; // When off, counts all gameElements

    public DuckStates.DuckPositions entry_state = DuckStates.DuckPositions.Placed;
    public DuckStates.DuckPositions exit_state = DuckStates.DuckPositions.Touched;
    public int processed_counter = 0;

    private void Start()
    {
    }

    public void Reset()
    {
        processed_counter = 0;

    }

    private void Update()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

    }

    private gameElement FindGameElement(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        gameElement collision_element = topparent.GetComponent<gameElement>();

        // While we haven't found a gameElement and there is more parents, keep searching for it
        while (collision_element == null && topparent.parent != null)
        {
            topparent = topparent.parent;
            collision_element = topparent.GetComponent<gameElement>();
        }

        // If this isn't an element of the proper type, return null
        if (collision_element == null) { return null; }
        if ((filter_element == ElementType.Off) || (collision_element.type == filter_element)) { return collision_element; }  
        return null;
    }

    void OnTriggerEnter(Collider collision)
    {
        if( GLOBALS.CLIENT_MODE) { return; }
 
        // Get the associated game element (duck)
        gameElement collision_element = FindGameElement(collision);
        if( !collision_element) { return; }

        // Make sure it has a Duck state machine
        DuckStates duck_sm = collision_element.GetComponent<DuckStates>();
        if( !duck_sm ) { return;  }

        // Next make sure it's the correct type 
        if( duck_sm.mystate != entry_state) { return; }

        // Advance it to the next level
        duck_sm.mystate = exit_state;

        // Increase counter 
        processed_counter += 1;
  
    }
  

}

