using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ball_Visuals : BHelperData 
{
    // This class requires the parent to also have Ball_data
    public ball_data balldata;
    MeshRenderer[] my_renderers;
    Color starting_color;
 

    // Start is called before the first frame update
    void Start()
    {
        // Retrieve the ball_data for this ball
        balldata = GetComponent<ball_data>();
        my_renderers = GetComponentsInChildren<MeshRenderer>();
        starting_color = my_renderers[0].material.color;

    }

    // Update is called once per frame
    public int old_ball_highlite = 0;
    public int brightness_setting = 100; // 100=normal, 40  = dark (not able to score
    
    // Send our highlite state
    public override string GetString()
    {
        return brightness_setting.ToString();
    }

    public override void SetString(string indata)
    {
        // Set highlite color
        int.TryParse(indata, out brightness_setting);
    }

    void Update()
    {
        if (my_renderers.Length <= 0) { return; }

        if (!GLOBALS.CLIENT_MODE)
        {
            // Determine highlite color
            if (balldata.IsFlagSet("Invalid")) { brightness_setting = 30; }
            else { brightness_setting = 100; }
        }

        // If nothing changed, do nothing
        if( old_ball_highlite == brightness_setting ) { return; }

        // Otherwise update highlite
        old_ball_highlite = brightness_setting;

        // Highlite our ball
        Color new_color = starting_color * ((float) brightness_setting)/100f;
        
        // Assign new colors
        foreach( MeshRenderer curr_renderer in my_renderers)
        {
            if( curr_renderer.material)
            {
                curr_renderer.material.color = new_color;
            }
        }

    }  
}
