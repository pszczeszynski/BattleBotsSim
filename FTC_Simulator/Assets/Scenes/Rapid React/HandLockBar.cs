using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandLockBar : MonoBehaviour
{
    public GameObject lockbar;

    public bool lockwhenable = false;

    public List<GameObject> collisions = new List<GameObject>();

    public void ForceLock(bool state)
    {
        // Can't disable object or components because it prevents a OnTriggerExit from being called.
        // Instead we will teleport the object inside the arm beam so it dissapears.
        float y = 0f;
        if( !state ) { y = 0.15f; }

        lockbar.transform.localPosition = new Vector3(0, y, 0);
       
    }

    // Locks or unlocks the lockbar
    public void Lock(bool lockstate)
    {
        // Clear bad entries
        ClearInvalid();

        lockwhenable = false;

        // If collisions are 0, then just go ahead and lock the bar
        if(collisions.Count == 0)
        {
            ForceLock(lockstate);
        }
        else
        {
            if( lockstate)
            {
                lockwhenable = true;
            }
            else
            {
                ForceLock(false);
            }
        }

    }

    public bool IsLocked()
    {
        return lockbar.transform.localPosition.y <= 0.1f;
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

    void OnTriggerEnter(Collider collision)
    {
        if ( GLOBALS.CLIENT_MODE) { return; }

        // If this is a collision with the lockbar, ignore
        foreach( Transform curr in lockbar.transform)
        {
            if( curr.gameObject == collision.gameObject) { return; }
        }

        // Only remember objects with Colliders that aren't triggers
        if (!collision.isTrigger)
        {
            collisions.Add(collision.gameObject);
        }

    }
   
    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        if (collision.gameObject == lockbar) { return; }

        if( collisions.Contains(collision.gameObject) )
        {
            collisions.Remove(collision.gameObject);
        }

        ClearInvalid();


        if ( collisions.Count <= 0)
        {
            if (lockwhenable)
            {
                ForceLock(true);   
            }
        }
    }

}
