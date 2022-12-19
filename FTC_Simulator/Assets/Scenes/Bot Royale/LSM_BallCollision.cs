using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LSM_BallCollision : BHelperData 
{
    // This class requires the parent to also have Ball_data
    public ball_data balldata;
    public long highlite_speed = 175; // ms for each cycle
    Material my_material;
    LastManStanding_Settings lms_settings;

    // Start is called before the first frame update
    void Start()
    {
        // Retrieve the ball_data for this ball
        balldata = GetComponent<ball_data>();
        my_material = GetComponent<MeshRenderer>().material;

        // Add a random number to the highlite_speed
        highlite_speed = (long)(((float)highlite_speed) * ((Random.value * 0.2f - 0.1f) + 1f));

        // Find the settings for our game
        GameObject settings = GameObject.Find("GameSettings");
        if (settings) { lms_settings = settings.GetComponent<LastManStanding_Settings>(); }
    }

    // Update is called once per frame
    bool old_ball_highlite = false;
    public int highlite_color = 3; // 1 = red, 2 = blue, 3 = white

    // Send our highlite state
    public override string GetString()
    {
        return highlite_color.ToString();
    }

    public override void SetString(string indata)
    {
        // Set highlite color
        int.TryParse(indata, out highlite_color);
    }

    void Update()
    {
        if (!my_material) { return; }
        
        // Determine highlite color only if not in client mode
        if (!GLOBALS.CLIENT_MODE) { 

            bool ball_highlite = balldata.thrown_by_id > 0;

            // If the ball highlite is engaged, check which color it should be
            if ( ball_highlite && !old_ball_highlite)
            {
                // Make sure we found settings
                if (!lms_settings) { lms_settings = GameObject.Find("GameSettings").GetComponent<LastManStanding_Settings>(); }

                if( lms_settings.FREE_FOR_ALL)              { highlite_color = 3; }
                else if( balldata.thrown_robotid.is_red)    { highlite_color = 1; }
                else                                        { highlite_color = 2; }
            }
            else if( !ball_highlite)
            {
                highlite_color = 0;
            }

            old_ball_highlite = ball_highlite;
        }

        // Highlite our ball
        float scalar = (float) (MyUtils.GetTimeMillis()  % highlite_speed) / ((float) highlite_speed);

        if (highlite_color == 1)
        {
            my_material.color = new Color(2, 0.5f*scalar+0.5f, 0.5f * scalar + 0.5f, 1);
        }
        else if (highlite_color == 2)
        {
            my_material.color = new Color(0.5f * scalar + 0.5f, 0.5f * scalar + 0.5f, 2, 1);
        }
        else if (highlite_color == 3)
        {
            my_material.color = new Color(scalar + 0.5f, scalar + 0.5f, scalar + 0.5f, 1);
        }
        else
        {
            my_material.color = Color.yellow;
        }

    }

    // Check if a collission occured
    void OnCollisionEnter(Collision collision)
    {
        // Make sure we are a projectile/weapon
        if( balldata.thrown_by_id < 1) { return; }

        // See if we collided with a robot
        RobotID therobot = collision.transform.GetComponentInParent<RobotID>();
        if( !therobot) {
            // Turn off projectile mode
            balldata.thrown_by_id = -1;
            balldata.thrown_robotid = null;
            return; 
        }

        // If this is the same person that threw the ball, ignore this collision
        if (balldata.thrown_by_id == therobot.id) { return; }

        // If it's another robot, determine what to do with it: free-for-all mark oponent for reset, otherwise do nothing
        if (lms_settings.FREE_FOR_ALL || (balldata.thrown_robotid.is_red != therobot.is_red) )
        {
            // Add the hit to the robot
            therobot.SetUserInt("HITS", therobot.GetUserInt("HITS") + 1);

            // Make robot remember who hit it last
            therobot.SetUserInt("LAST_HIT_ID", balldata.thrown_robotid.id);
            balldata.thrown_robotid.SetUserFloat("KILLS", balldata.thrown_robotid.GetUserFloat("KILLS") + 1f / lms_settings.HIT_COUNT);

            // Play hit sound
            GetComponent<AudioManager>().Play("HitOpponent", 0, 1f);
        }
        else
        {
            GetComponent<AudioManager>().Play("ballhit1", 0, 1f);
        }

        // De-activate ball
        balldata.thrown_by_id = -1;
        balldata.thrown_robotid = null;

        return;
    }

    public void DeactiveBall()
    {
        balldata.thrown_by_id = -1;
        balldata.thrown_robotid = null;
        return;
    }
}
