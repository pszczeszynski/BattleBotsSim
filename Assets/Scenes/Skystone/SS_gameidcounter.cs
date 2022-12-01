using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// This scoring box is for the power line transitions
public class SS_gameidcounter : MonoBehaviour
{
    // list of elements colliding
    public Dictionary<int, gameElement> collisions = new Dictionary<int, gameElement>();
    public Dictionary<int, int> collisions_count = new Dictionary<int, int>();
    public int count = 0;

    private void Update()
    {
        count = collisions.Count;
    }

    private void OnDisable()
    {
        // Reset the collistions
        Reset();
    }

    private void OnDestroy()
    {
        Reset();
    }

    public void Reset()
    {
        foreach( gameElement currelement in collisions.Values)
        {
            if( currelement && currelement.enabled && currelement.held_by_robot > 0)
            {
                currelement.held_by_robot -= 1;
            }
        }
        collisions.Clear();
        collisions_count.Clear();
    }

    void OnTriggerEnter(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        if (collision_element == null) { return; }

        // Make sure it's a block
        if(collision_element.type != ElementType.Cube) { return; }

        // Add it to our list
        if (collisions.ContainsKey(collision_element.id)) {
            collisions_count[collision_element.id] += 1;
        }
        else { 
            collisions[collision_element.id] = collision_element;
            collisions_count[collision_element.id] = 1;
            collision_element.held_by_robot += 1;
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
        if (collision_element == null) { return; }
        
        // Remove it from our list if it's ready
        if (collisions.ContainsKey(collision_element.id) )
        {
            collisions_count[collision_element.id] -= 1;

            if(collisions_count[collision_element.id] <= 0)
            {
                collisions.Remove(collision_element.id);
                collisions_count.Remove(collision_element.id);
                collision_element.held_by_robot -= 1;
            }
        }
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

        return collision_element;
    }

    public int GetElementCount()
    {
        return collisions.Count;
    }

    // Adds elements to list and returns how many were added
    public int GetElements(Dictionary<int,gameElement> elements)
    {
        int added = 0;

        foreach( int key in collisions.Keys)
        {
            if( !elements.ContainsKey(key))
            {
                added++;
                elements[key] = collisions[key];
            }
        }

        return added;
    }

    private void RemoveInvalidItems()
    {
        // Remove any objects that have been destroyed
        foreach(int key in collisions.Keys)
        {
            if ((collisions[key] == null) || (collisions[key].gameObject == null) || (collisions[key].enabled == false))
            {
                collisions.Remove(key);
                collisions_count.Remove(key);
            }
        }
    }

}

