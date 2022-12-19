using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// counts how many rings are inside it
public class GoalCounter : MonoBehaviour
{

    // list of game elements colliding
    public List<GoalScorer> collisions = new List<GoalScorer>();
    public List<int> collisions_count = new List<int>();
    public bool isRed = false;
    public bool isBlue = false;


    private void Update()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

    }

    private GoalScorer FindGameElement(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        GoalScorer collision_element = topparent.GetComponent<GoalScorer>();

        // While we haven't found a gameElement and there is more parents, keep searching for it
        while (collision_element == null && topparent.parent != null)
        {
            topparent = topparent.parent;
            collision_element = topparent.GetComponent<GoalScorer>();
        }
        return collision_element;
    }



    void OnTriggerEnter(Collider collision)
    {
        if( GLOBALS.CLIENT_MODE) { return; }
        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element (ring)
        GoalScorer collision_element = FindGameElement(collision);

        // Make sure we found something
        if (collision_element)
        {
            // Drop if this goal is an alliance goal in the wrong zone
            if( collision_element.isRed && isBlue ||
                collision_element.isBlue && isRed)
            {
                return;
            }

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
        GoalScorer collision_element = FindGameElement(collision);

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

    public int GetGoalCount()
    {
        return collisions.Count;
    }

   
    public bool IsGoalInside(GoalScorer item)
    {
        return collisions.Contains(item);
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

