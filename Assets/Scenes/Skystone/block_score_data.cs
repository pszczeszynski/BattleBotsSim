using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum PositionState
{
    Loading = 1, // Fully inside loading
    ToBuilding,  // Transition started from loading side
    Building,   // Fully in Building
    ToLoading,   // Transition started from Building Side
    off         // not to be considered for scoring
}
public class block_score_data : gameElement
{
    // Contains the history of where the block has been to determine scoring
    public PositionState currposition = PositionState.Loading;
    public int pieces_in_crossing = 0;
    public bool has_scored = false;

    // Mark that we entered a transition point
    public void StartTransition(float center_x)
    {
        // If this is the first piece entering the transition zone, mark where we are coming from
        if (pieces_in_crossing == 0)
        {
            // Mark which side we entered the transition point from
            if (transform.position.x < center_x)
            {
                currposition = PositionState.ToBuilding;
            }
            else
            {
                currposition = PositionState.ToLoading;
            }
        }

        pieces_in_crossing += 1;
    }

    public int EndTransition(float center_x)
    {
        pieces_in_crossing -= 1;

        if( pieces_in_crossing > 0) { return 0; }

        // If we came from the opposite side, then return -1 if we should be subtracting points or +1 if we should be adding.
        // Otherwise return 0
        if (currposition == PositionState.ToLoading)
        {
            if (transform.position.x < center_x)
            {
                has_scored = false;
                currposition = PositionState.Loading;
                return -1;
            }
            else
            {
                currposition = PositionState.Building;
            }
        }
        if (currposition == PositionState.ToBuilding)
        {
            if (transform.position.x > center_x)
            {
                currposition = PositionState.Building;
                if(has_scored) { return 0; }
                has_scored = true;
                return 1;
            }
            else
            {
                currposition = PositionState.Loading;
            }
        }
        return 0;
    }

    public bool OutOfRegion()
    {
        return pieces_in_crossing == 0;
    }

    // Over-ride ResetPosition - need to also reset our settings
    public override void ResetPosition(int option = 1)
    {
        currposition = PositionState.Loading;
        pieces_in_crossing = 0;
        has_scored = false;

        base.ResetPosition(option);
    }

}
