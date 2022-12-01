using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// This scoring box is for the ball finder
// Modified to also keep track of gameelements=jewels passing through
// Also added code to ignore gamelements with "Bad" keyword in their data
public class IR_scoringBox : MonoBehaviour
{
    public int number_of_items = 0;
    public int number_of_balls = 0;
    public bool is_red = true;  // Mark what side it's on
    public int sound_to_play = 1; // which sound to play
    public Dictionary<int, int> user_ball_count = new Dictionary<int, int>();
    public bool ignore_Bad_elements = false;
    public bool penalize_Bad_elements = false;
    public Dictionary<int, int> bad_ball_count = new Dictionary<int, int>();
    public int red_bad_elements = 0;
    public int blue_bad_elements = 0;
    public bool play_animation = false;
    public bool disable_in_client = true;

    private Animator my_animation;

    private void OnEnable()
    {
        my_animation = GetComponent<Animator>();
    }

    public void Reset()
    {
        number_of_balls = 0;
        balls.Clear();
        user_ball_count.Clear();
        bad_ball_count.Clear();
        number_of_items = 0;
        red_bad_elements = 0;
        blue_bad_elements = 0;
    }

    public bool IsSomethingInside()
    {
        return number_of_items > 0;
    }
    
    private void Update()
    {

    }

    private gameElement FindGameElement(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        gameElement collision_element = topparent.GetComponent<gameElement>();

        // While we haven't found a gameElement and there is more parents, keep searching for it
        if (collision_element == null )
        {
            collision_element = topparent.GetComponentInParent<gameElement>();
        }

        // If this isn't an element of the proper type, return null
        if (collision_element == null) { return null; }
        if (collision_element.type == ElementType.Jewel) { return collision_element; }  // Return if it's of type Jewel
        return null;
    }

    public List<Transform> balls = new List<Transform>();
    private AudioManager my_top_audiomanager = null;

    void OnTriggerEnter(Collider collision)
    {
        // Don't do anything if this is a client
        if( disable_in_client && GLOBALS.CLIENT_MODE) { return;  }

        // Mark something entered
        number_of_items++;

        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);
        if(collision_element == null) { return; }

        ball_data myballdata = collision_element.GetComponent<ball_data>();
        if (myballdata == null) { return; }

        // Ignore if already in our list
        if( balls.Contains(collision_element.transform)) { return; }

        // Make sure we found something
        if ((collision_element == null) || (myballdata == null) ) { return; }
       
        // Ignore if this gameElement is marked as invalid
        if(ignore_Bad_elements &&  myballdata.flags.ContainsKey("Bad"))
        {
            return;
        }

        // Penalize if this gameElement is marked as bad
        // Penalties are delt with on top of the regular scoring the item gives
        if (penalize_Bad_elements && myballdata.flags.ContainsKey("Bad"))
        {
            if (bad_ball_count.ContainsKey(myballdata.thrown_by_id))
            {
                bad_ball_count[myballdata.thrown_by_id] += 1;
            }
            else
            {
                bad_ball_count[myballdata.thrown_by_id] = 1;
            }

            // Add the bad elements count
            if( myballdata.thrown_robotid.is_red) { red_bad_elements++; }
            else { blue_bad_elements++; }

        }

        // Increment our count
        number_of_balls += 1;

        if(user_ball_count.ContainsKey(myballdata.thrown_by_id))
        {
            user_ball_count[myballdata.thrown_by_id] += 1;
        }
        else
        {
            user_ball_count[myballdata.thrown_by_id] = 1;
        }

        // Mark ball for location reset
        if (is_red)
        {
            myballdata.scored_red = true;
        }
        else
        {
            myballdata.scored_blue = true;
        }

        // Add it to our list
        balls.Add(collision_element.transform);

        if(my_top_audiomanager == null)
        {
            // Find the top audio manager gameobject
            GameObject go_audio = GameObject.Find("AudioManager");
            if( !go_audio) { return; }

            // Extract it's AudioManager
            my_top_audiomanager = go_audio.GetComponent<AudioManager>();
            if(my_top_audiomanager == null) { return; }
        }

        PlayAnimation();

        if (sound_to_play == 1)
        {
            // my_top_audiomanager.Stop("ballscored", 0f);
            my_top_audiomanager.Play("ballscored", 0f);
        }
        else if(sound_to_play==2)
        {
            // my_top_audiomanager.Stop("ballscored_high", 0f);
            my_top_audiomanager.Play("ballscored_high", 0f);
        }
    }

    public void PlayAnimation()
    {
        if (play_animation && my_animation)
        {
            my_animation.Play("Base Layer.flash");
        }
    }

    void OnTriggerExit(Collider collision)
    {
        // Don't do anything if this is a client
        if (disable_in_client && GLOBALS.CLIENT_MODE) { return; }

        number_of_items--;
    }

    // Return one ball if it exists
    public Transform GetNextBall()
    {
        // If empty, return null
        if( balls.Count <= 0) { return null;  }

        // Take the first one out
        Transform return_val = balls[0];
        balls.RemoveAt(0);
        
        return return_val;
    }

}

