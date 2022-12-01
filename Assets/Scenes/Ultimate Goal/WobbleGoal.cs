using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WobbleGoal : BHelperData
{
    public bool is_red = false;     // Red or Blue color
    public bool endgame_started = false; // Used to apply indicator if it's valid for scoring
    public bool auto_inplay = false; // coloring for auto
    public bool valid = false; // True if it's o.k. to be scored in endgame
    public bool valid_old = false; // Scorekeeper uses this to track if it accounted for the change of state
    public int robot_scored = -1;

    // Who touched us last
    public RobotID last_contact_robot;


    Renderer[] renderers;

    // Start is called before the first frame update   
    void Start()
    {
        // Find all renderers
        renderers = gameObject.GetComponentsInChildren<Renderer>();
    }

    // Update is called once per frame
    private bool valid_color = true;
    private bool last_color = false;

    public void Reset()
    {
        endgame_started = false;
        auto_inplay = false;
        valid = false;
        robot_scored = -1;
    }

    private void OnCollisionEnter(Collision collision)
    {
        // Remember the last robot that touched us
        // Get the RobotID
        RobotID curr_robot = collision.gameObject.GetComponentInParent<RobotID>();
        if( !curr_robot) { return;  }

        last_contact_robot = curr_robot;
    }

    void Update()
    {
        // calculate the color the game element should be if we need to calculate it
        if (!GLOBALS.CLIENT_MODE)
        { valid_color = valid || !(endgame_started || auto_inplay); }

        // Adjust color to calculated
        if (valid_color != last_color)
        {
            // Go through renderers and adjust colors
            foreach( Renderer curr_renderer in renderers)
            {
                Color new_color = curr_renderer.material.color;
                if(valid_color)
                {
                    new_color.r = (new_color.r > 0.45f) ? 1f : 0;
                    new_color.g = (new_color.g > 0.45f) ? 1f : 0;
                    new_color.b = (new_color.b > 0.45f) ? 1f : 0;
                }
                else
                {
                    new_color.r = (new_color.r > 0.45f) ? 0.5f : 0.2f;
                    new_color.g = (new_color.g > 0.45f) ? 0.5f : 0.2f;
                    new_color.b = (new_color.b > 0.45f) ? 0.5f : 0.2f;
                }

                curr_renderer.material.color = new_color;

                last_color = valid_color;
            }
        }      
    }

    override public string GetString()
    {
        return (valid_color ? "1" : "0");
    }

    override public void SetString(string indata)
    {
        valid_color = indata[0] == '1';
    }
}
