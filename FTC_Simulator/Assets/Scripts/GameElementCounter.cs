using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// Uses triggers to count how many gameElements are inside it.

public class GameElementCounter : MonoBehaviour
{

    // list of game elements colliding
    public List<gameElement> collisions = new List<gameElement>();
    public List<int> collisions_count = new List<int>();

    public ElementType filter_element = ElementType.Off; // When off, counts all gameElements

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
        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element (ring)
        gameElement collision_element = FindGameElement(collision);

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
        gameElement collision_element = FindGameElement(collision);

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

    public int GetElementCount()
    {
        return collisions.Count;
    }

   
    public bool IsGameElementInside(gameElement item)
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

