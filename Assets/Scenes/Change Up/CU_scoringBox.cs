using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// This scoring box is for the power line transitions
public class CU_scoringBox : MonoBehaviour
{
    ChangeUp_Settings cu_settings = null;
    public VLB.VolumetricLightBeam our_light;

    // list of elements colliding
    public List<gameElement> collisions = new List<gameElement>();
    public List<int> collisions_count = new List<int>();

    private void Start()
    {
        if (GameObject.Find("GameSettings"))
        {
            cu_settings = GameObject.Find("GameSettings").GetComponent<ChangeUp_Settings>();
        }
    }

    public void Reset()
    {

    }

    private void Update()
    {
        // Don't do anything if this is a client (change our mind later?)
        if (GLOBALS.CLIENT_MODE) { return; }

        SetHighestBall(GetHighestBall());

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
        if ((collision_element.type == ElementType.Cube) || (collision_element.type == ElementType.CubeDark)) { return collision_element; }  // Return if it's of type cube/Block
        return null;
    }

    void OnTriggerEnter(Collider collision)
    {
        if( GLOBALS.CLIENT_MODE) { return; }
        
        // Clean up list
        RemoveInvalidItems();

        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        if (collision_element == null) { return; }

        // Add it to our list
        if (collisions.Contains(collision_element)) {
            collisions_count[collisions.IndexOf(collision_element)] += 1;
        }
        else { 
            collisions.Add(collision_element);
            collisions_count.Add(1);
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
        if (collisions.Contains(collision_element) )
        {
            int index = collisions.IndexOf(collision_element);
            collisions_count[index] -= 1;
                
            if(collisions_count[index] <= 0 )
            {
                collisions_count.RemoveAt(index);
                collisions.RemoveAt(index);
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

    // Highest ball returns the highest game element that is below the threshold
    // 2 = blue, 0 = none, 1 = red
    public int GetHighestBall()
    {
        gameElement highest_element = null;

        foreach (gameElement curr_element in collisions)
        {
            // Make sure it's actually inside
            if(!curr_element.transform || (curr_element.transform.position.y > 1.56f)) { continue; }

            if (!highest_element)
            {
                highest_element = curr_element;
                continue;
            }

            if(highest_element.transform && (curr_element.transform.position.y > highest_element.transform.position.y))
            {
                highest_element = curr_element;
            }
        }

        if( !highest_element) { return 0; }
        if( highest_element.type == ElementType.CubeDark) { return 1;  }
        return 2;
    }

    public void SetHighestBall(int top_element)
    {
        // Exit if a settings window wasn't found
        if (cu_settings == null)
        {
            cu_settings = GameObject.Find("GameSettings").GetComponent<ChangeUp_Settings>();
            if (cu_settings == null) { return; }
        }

        // If our light isn't found, don't do light stuff
        if (!our_light) { return; }

        if (!cu_settings.ENABLE_LIGHTS)
        {
            our_light.enabled = false;
        }
        else
        {
            our_light.enabled = true;

            if (top_element == 1)
            {
                our_light.color = Color.red;
            }
            else if (top_element == 2)
            {
                our_light.color = Color.blue;
            }
            else
            {
                our_light.color = Color.grey;
            }
        }
    }

    public bool IsTopElementBlue()
    {
        // If top element is blue, return true
        return GetHighestBall() == 2;
    }

    public bool IsTopElementRed()
    {
        // If top element is blue, return true
        return GetHighestBall() == 1;
    }

    public int GetBlueCount()
    {
        int total = 0;

        foreach (gameElement curr_element in collisions)
        {
            if( curr_element.type == ElementType.Cube )
            {
                total++;
            }
        }

        return total;
    }

    public int GetRedCount()
    {
        int total = 0;

        foreach (gameElement curr_element in collisions)
        {
            if (curr_element.type == ElementType.CubeDark)
            {
                total++;
            }
        }

        return total;
    }
}

