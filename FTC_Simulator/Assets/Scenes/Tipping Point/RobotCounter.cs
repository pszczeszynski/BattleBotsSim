using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// Keeps track of robots inside this trigger box
public class RobotCounter : MonoBehaviour
{

    // list of game elements colliding
    public List<RobotID> collisions = new List<RobotID>();
    public List<int> collisions_count = new List<int>();

    private void Start()
    {
    }

    public void Reset()
    {

    }

    private void Update()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

    }

    private RobotID FindRobotID(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        RobotID collision_element = topparent.GetComponent<RobotID>();

        // While we haven't found a RobotID and there is more parents, keep searching for it
        while (collision_element == null && topparent.parent != null)
        {
            topparent = topparent.parent;
            collision_element = topparent.GetComponent<RobotID>();
        }
        return collision_element;
    }


    void OnTriggerEnter(Collider collision)
    {
        if( GLOBALS.CLIENT_MODE) { return; }
        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element (ring)
        RobotID collision_element = FindRobotID(collision);

        // Make sure we found something
        if (collision_element)
        {
            // Add it to our list
            if (collisions.Contains(collision_element))
            {
                collisions_count[collisions.IndexOf(collision_element)] += 1;
            }
            else
            {
                collisions.Add(collision_element);
                collisions_count.Add(1);
            }
        }      

    }
   

    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element
        RobotID collision_element = FindRobotID(collision);

        // Make sure we found something
        if (collision_element)
        {
            // Remove it from our list if it's ready
            if (collisions.Contains(collision_element))
            {
                int index = collisions.IndexOf(collision_element);
                collisions_count[index] -= 1;

                if (collisions_count[index] <= 0)
                {
                    collisions_count.RemoveAt(index);
                    collisions.RemoveAt(index);
                }
            }
        }     
    }

    public int GetRobotCount()
    {
        return collisions.Count;
    }

    public int GetBlueRobotCount()
    {
        int count = 0;

        foreach(RobotID curr in collisions )
        {
            if( !curr.is_red ) { count += 1; }
        }
        return count;
    }

    public int GetRedRobotCount()
    {
        int count = 0;

        foreach (RobotID curr in collisions)
        {
            if (curr.is_red) { count += 1; }
        }
        return count;
    }

    private void RemoveInvalidItems()
    {
        int item = collisions.Count - 1;
        while (item >= 0)
        {
            if (collisions[item] == null)
            {
                collisions.RemoveAt(item);
                collisions_count.RemoveAt(item);
            }

            item -= 1;
        }    
    }

}

