using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// Keeps track of platforms inside it
public class PlatformCounter : MonoBehaviour
{

    // list of game elements colliding
    public List<PlatformID> collisions = new List<PlatformID>();
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

    private PlatformID FindPlatformID(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        PlatformID collision_element = topparent.GetComponent<PlatformID>();

        // While we haven't found a RobotID and there is more parents, keep searching for it
        while (collision_element == null && topparent.parent != null)
        {
            topparent = topparent.parent;
            collision_element = topparent.GetComponent<PlatformID>();
        }
        return collision_element;
    }


    void OnTriggerEnter(Collider collision)
    {
        if( GLOBALS.CLIENT_MODE) { return; }
        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element (ring)
        PlatformID collision_element = FindPlatformID(collision);

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
        PlatformID collision_element = FindPlatformID(collision);

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

    public int GetPlatformCount()
    {
        return collisions.Count;
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

