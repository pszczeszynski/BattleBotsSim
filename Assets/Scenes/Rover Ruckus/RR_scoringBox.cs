using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class RR_scoringBox : MonoBehaviour {

    public bool score_cubes = true;
    public bool score_balls = true;

    // list of elements colliding
    public List<gameElement> collisions = new List<gameElement>();

    private void Update()
    {
        // Vary the color of the element if it is in box    
        // Since the cubes have 0 blue, this will update the blue based on time
        // Lets do a 2s glow
        float color_index = (float) ((DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond) % 2000);
        if( color_index > 1000f )
        {
            color_index = 2000f - color_index;
        }

        foreach (gameElement currelement in collisions)
        {
            Renderer cuberender = currelement.gameObject.GetComponent<Renderer>();
            Color newcolor = cuberender.material.color;
            if( currelement.type == ElementType.Cube)
            { newcolor.b = 0.5f + 0.5f * color_index / 1000f; }
            else
            { newcolor.b = 0.3f * color_index / 1000f; }

            cuberender.material.color = newcolor;
        }
    }

    private gameElement FindGameElement( Collider collision )
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
        if( collision_element == null ) { return null; }
        if(score_cubes && collision_element.type == ElementType.Cube ||
           score_balls && collision_element.type == ElementType.Jewel ) { return collision_element; }
        return null;
    }

    void OnTriggerEnter( Collider collision)
    {
        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        if( collision_element == null ) { return; }

        // Add it to our list
        if ( collisions.Contains(collision_element) ) { return;  }
        else { collisions.Add(collision_element);  }
    }

    void OnTriggerExit(Collider collision )
    {
        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        if (collision_element == null) { return; }

        // Remove it from our list
        if (collisions.Contains(collision_element))
        { collisions.Remove(collision_element); }

        // Fix the color
        collision_element.ResetColor();
    }

    public int GetElementCount()
    {
        return collisions.Count;
    }
}

