using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// This scoring box is for the ball finder
// Modified to also keep track of gameelements=jewels passing through
// Also added code to ignore gamelements with "Bad" keyword in their data
public class UG_powershots : MonoBehaviour
{
    public bool penalty = false;
    public int hit_by_id = -1;
    public bool scored = false;
    public bool unscored = false;
    public HingeJoint myjoint;
    public bool up = false;

    private Vector3 starting_pos;
    private Quaternion starting_rot;

    private AudioManager my_top_audiomanager = null;

    private void Start()
    {
        starting_pos = transform.position;
        starting_rot = transform.rotation;
        myjoint = GetComponent<HingeJoint>();


        // Find the top audio manager gameobject
        GameObject go_audio = GameObject.Find("AudioManager");
        if (!go_audio) { return; }

        // Extract it's AudioManager
        my_top_audiomanager = go_audio.GetComponent<AudioManager>();
        if (my_top_audiomanager == null) { return; }
        
    }

    public void Reset()
    {
        // Make sure we are being reset after initialization
        if( !myjoint ) { return;  }
        penalty = false;
        hit_by_id = -1;
        transform.position = starting_pos;
        transform.rotation = starting_rot;
        scored = false;
        unscored = false;
        up = true;
        previous_up = true;
    }


    public void ClearScore()
    {
        if( scored )
        {
            penalty = false;
            hit_by_id = -1;
        }

        scored = false;
        unscored = false;
    }

    private bool previous_up = false;
    private void Update()
    {
        // Initialize for the first time
        if( !myjoint) {
            return;
        }

        // Update our positions
        up = myjoint.angle <= 45f;

        // Trigger scored true if applicable
        if (!scored)
        {
            scored = !up && previous_up;
            if (scored) { 
                unscored = false;
                my_top_audiomanager.Play("popsound", 0);
            }
        }

        // Same with unscored
        if (!unscored)
        {
            unscored = up && !previous_up;
            if (unscored) { scored = false; }
        }

        previous_up = up;
    }

    private void OnCollisionEnter(Collision collision)
    {
        // If we are triggered already, then ignore any collisions
        if(!myjoint || (myjoint.angle > 45f)) { return; }

        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);
        if(collision_element == null) { return; }

        ball_data myballdata = collision_element.GetComponent<ball_data>();
        if (myballdata == null) { return; }

        // Make sure we found something
        if ((collision_element == null) || (myballdata == null) ) { return; }

        // Mark if penalty is supposed to be assessed
        if (myballdata.thrown_by_id > 0)
        {
            if (myballdata.thrown_by_id == hit_by_id)
            {
                penalty |= myballdata.flags.ContainsKey("Bad");
            }
            else
            {
                penalty = myballdata.flags.ContainsKey("Bad");
            }

            // Record who hit us
            hit_by_id = myballdata.thrown_by_id;
        }
    }


    private gameElement FindGameElement(Collision collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        gameElement collision_element = topparent.GetComponent<gameElement>();

        // While we haven't found a gameElement and there is more parents, keep searching for it
        if (collision_element == null)
        {
            collision_element = topparent.GetComponentInParent<gameElement>();
        }

        // If this isn't an element of the proper type, return null
        if (collision_element == null) { return null; }
        if (collision_element.type == ElementType.Jewel) { return collision_element; }  // Return if it's of type Jewel
        return null;
    }

}

