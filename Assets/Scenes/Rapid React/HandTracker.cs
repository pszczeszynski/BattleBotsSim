using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTracker : MonoBehaviour
{
    HingeJoint myhinge;
    HingeJoint otherhinge;

    public bool FreshHold = false;
    public List<GameObject> collisions = new List<GameObject>();
    public int min_collisions = 2;

    // Returns true if this is a new hold occured without clearing
    public bool IsFreshHold()
    {
        return FreshHold;
    }

    public void ClearHold()
    { 
        FreshHold = false;        
    }

    public void ClearInvalid()
    {

        // Remove items without a transform OR who's collision count reached <=0
        int item = collisions.Count - 1;
        while (item >= 0)
        {
            if ((collisions[item] == null) || (!collisions[item].activeSelf))
            {
                collisions.RemoveAt(item);
            }

            item -= 1;
        }

    }

    // Returns true if there are any collisions
    public bool IsHolding()
    {
        ClearInvalid();

        return collisions.Count != 0;
    }

    void OnTriggerEnter(Collider collision)
    {
        if ( GLOBALS.CLIENT_MODE) { return; }

        if( collision.gameObject.GetComponent<CapsuleCollider>() )
        {
            collisions.Add(collision.gameObject);

            if( collisions.Count >= min_collisions)
            {
                FreshHold = true;
            }
        }      
    }
   
    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        if (collision.gameObject.GetComponent<CapsuleCollider>())
        {
            if (collisions.Contains(collision.gameObject))
            {
                collisions.Remove(collision.gameObject);
            }

            ClearInvalid();

            if( collisions.Count == 0)
            {
                FreshHold = false;
            }
        }
    }


}
