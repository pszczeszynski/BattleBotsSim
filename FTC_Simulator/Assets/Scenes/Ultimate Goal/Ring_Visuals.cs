using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ring_Visuals : BHelperData 
{
    // This class requires the parent to also have Ball_data
    public ball_data balldata;
    public long highlite_speed = 175; // ms for each cycle
    MeshRenderer[] my_renderers;

    // Start is called before the first frame update
    void Start()
    {
        // Retrieve the ball_data for this ball
        balldata = GetComponent<ball_data>();
        my_renderers = GetComponentsInChildren<MeshRenderer>();

        // Add a random number to the highlite_speed
        highlite_speed = (long)(((float)highlite_speed) * ((Random.value * 0.2f - 0.1f) + 1f));

    }

    // Update is called once per frame
    public int old_ball_highlite = 0;
    public int highlite_color = 0; // 0=off, 1 = white (invalid throw)
    
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
        if (my_renderers.Length <= 0) { return; }

        if (!GLOBALS.CLIENT_MODE)
        {
            // Determine highlite color
            if ((balldata.thrown_by_id > 0) && balldata.flags.ContainsKey("Bad")) { highlite_color = 1; }
            else { highlite_color = 0; }
        }

        // If nothing changed, do nothing
        if( (old_ball_highlite == 0) && (highlite_color == 0) ) { return; }

        // Otherwise update highlite
        old_ball_highlite = highlite_color;

        // Highlite our ball
        float scalar = (float) (MyUtils.GetTimeMillis()  % highlite_speed) / ((float) highlite_speed);
        Color new_color = new Color(1f,0.254f, 0);  // Set the orange color of a normal ring

        // If we are in a highlite mode, highlite it.
        if (highlite_color == 1)
        {
            new_color.r = scalar + 0.5f;
            new_color.g = scalar + 0.5f;
            new_color.b = scalar + 0.5f;
        }
        
        // Assign new colors
        foreach( MeshRenderer curr_renderer in my_renderers)
        {
            if( curr_renderer.material)
            {
                curr_renderer.material.color = new_color;
            }
        }

    }

    // Check if a collission occured
    void OnCollisionEnter(Collision collision)
    {
        // Make sure we are a projectile/weapon
        if( balldata.thrown_by_id < 1) { return; }

        // If this is a field structure, ignore it
        if( collision.transform.tag == "FieldStructure") { return; }

        // Otherwise this should no longer 
        // See if we collided with a robot: if it's ourselves, ignore it
        RobotID therobot = collision.transform.GetComponentInParent<RobotID>();
        // If this is the same person that threw the ball, ignore this collision
        if ((therobot != null) && (balldata.thrown_by_id == therobot.id)) { return; }

        // De-activate ball
        balldata.thrown_by_id = -1;
        balldata.thrown_robotid = null;
        balldata.flags.Clear();

        GetComponent<AudioManager>().Play("ballhit1", 0, 1f);

        return;
    }

    public void DeactiveBall()
    {
        balldata.thrown_by_id = -1;
        balldata.thrown_robotid = null;
        balldata.flags.Clear();
        return;
    }
}
