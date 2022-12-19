using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// counts how many rings are inside it
public class RingCounter : MonoBehaviour
{

    // list of game elements colliding
    public List<gameElement> collisions = new List<gameElement>();
    public List<int> collisions_count = new List<int>();

    public List<gameElement> collisions_ir = new List<gameElement>();
    public List<int> collisions_ir_count = new List<int>();

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
        if (collision_element.type == ElementType.Jewel) { return collision_element; }  // Return if it's of type Jewel which designates a ring
        return null;
    }

    private gameElement FindGameElementFromInnerRingID(Collider collision)
    {
        // Find parent
        Transform topparent = collision.transform;

        if(topparent.GetComponent<InnerRingID>() || topparent.GetComponentInParent<InnerRingID>())
        {
            return FindGameElement(collision);
        }

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

        // Also find the innerRing elements
        collision_element = FindGameElementFromInnerRingID(collision);
        if( !collision_element) { return;  }

        // Add it to our list
        if (collisions_ir.Contains(collision_element))
        {
            collisions_ir_count[collisions_ir.IndexOf(collision_element)] += 1;
        }
        else
        {
            collisions_ir.Add(collision_element);
            collisions_ir_count.Add(1);
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

        // Also find the innerRing elements
        collision_element = FindGameElementFromInnerRingID(collision);
        if (!collision_element) { return; }

        // Add it to our list
        if (collisions_ir.Contains(collision_element))
        {
            int index = collisions_ir.IndexOf(collision_element);
            collisions_ir_count[index] -= 1;

            if (collisions_ir_count[index] <= 0)
            {
                collisions_ir_count.RemoveAt(index);
                collisions_ir.RemoveAt(index);
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

        item = collisions_ir.Count - 1;
        while (item >= 0)
        {
            if (collisions_ir[item] == null)
            {
                collisions_ir.RemoveAt(item);
                collisions_ir_count.RemoveAt(item);
            }

            item -= 1;
        }
    }

}

