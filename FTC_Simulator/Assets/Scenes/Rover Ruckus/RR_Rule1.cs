using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class RR_Rule1 : MonoBehaviour {
    // Enable/Disable this tule
    public static bool enable_rules = false;

    // list of elements colliding
    public List<gameElement> collisions = new List<gameElement>();
    public RobotID robotid;
    private MeshRenderer mesh;
    public Transform goal;
    bool redpos = true;


    private void Update()
    {
        if (!enable_rules) { return; }

        if (mesh == null)
        {
            mesh = GetComponent<MeshRenderer>();
        }
            
        // Only run myself if assigned to a robot
        if (robotid == null)
        {
            mesh.enabled = false;
            return;
        }

        mesh.enabled = true;

        // Determine our position
        if( robotid.starting_pos == "Blue Front" || robotid.starting_pos == "Blue Back")
        {
            redpos = false;
        }
        else
        {
            redpos = true;
        }

        // Set size of rectangle
        // Changed algorithm to be a fixed size
        Vector3 size = transform.localScale;
        size.x = 0.15f;  // double length of average robot
        size.z = 0.075f; // Width of average robot
        transform.localScale = size;

        // Transform plane to mark our path to lander
        // First get center position
        Vector3 center = (goal.position + robotid.position) / 2f;
        center.y = robotid.position.y;
        transform.position = center;
        transform.position = robotid.position;

        // Next get angle and size
        Vector3 delta = goal.position - robotid.position;
        delta.y = 0;

        float targetangle = Vector3.SignedAngle(new Vector3(1f, 0, 0), delta, new Vector3(0,1,0));
        Vector3 myangles = transform.eulerAngles;
        myangles.y = targetangle;
        transform.eulerAngles = myangles;

  

        // Vary the color of the element if it is in box    
        // Since the cubes have 0 blue, this will update the blue based on time
        // Lets do a 2s glow
        float color_index = (float) ((DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond) % 2000);
        if( color_index > 1000f )
        {
            color_index = 2000f - color_index;
        }

        //foreach (gameElement currelement in collisions)
        //{
        //    Renderer cuberender = currelement.gameObject.GetComponent<Renderer>();
        //    Color newcolor = cuberender.material.color;
        //    if( currelement.type == ElementType.Cube)
        //    { newcolor.b = 0.5f + 0.5f * color_index / 1000f; }
        //    else
        //    { newcolor.b = 0.3f * color_index / 1000f; }
        //
        //    cuberender.material.color = newcolor;
        //}
    }

    private gameElement FindGameElement( Collider collision )
    {
        // Find the top parent
        Transform topparent = collision.transform;

        // gameElement collision_element = topparent.GetComponent<gameElement>();

        // While we haven't found a gameElement and there is more parents, keep searching for it
        //while (collision_element == null && topparent.parent != null)
        //{
        //    topparent = topparent.parent;
        //    collision_element = topparent.GetComponent<gameElement>();
        //}

        // If this isn't an element of the proper type, return null
        //if( collision_element == null ) { return null; }
        //if(score_cubes && collision_element.type == ElementType.Cube ||
        //   score_balls && collision_element.type == ElementType.Jewel ) { return collision_element; }
        return null;
    }

    void OnTriggerEnter( Collider collision)
    {
        // Only run myself if assigned to a robot
        if (robotid == null)
        { return; }

        // Get the associated game element
        // gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        // if( collision_element == null ) { return; }

        // Add it to our list
        //if ( collisions.Contains(collision_element) ) { return;  }
        // else { collisions.Add(collision_element);  }
    }

    void OnTriggerExit(Collider collision )
    {
        // Only run myself if assigned to a robot
        if (robotid == null)
        { return; }

        // Get the associated game element
        // gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        // if (collision_element == null) { return; }

        // Remove it from our list
        // if (collisions.Contains(collision_element))
        //{ collisions.Remove(collision_element); }

        // Fix the color
        // collision_element.ResetColor();
    }

    public int GetElementCount()
    {
        return collisions.Count;
    }
}

