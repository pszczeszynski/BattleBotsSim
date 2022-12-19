using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MarkBall : MonoBehaviour
{
    public bool disable_in_client = true;
    public bool clear_flags = false;
    public string add_key = "";
    public string add_value = "";

    void OnTriggerEnter(Collider collision)
    {
        if (disable_in_client && GLOBALS.CLIENT_MODE) { return; }

        // Get ball data
        ball_data balldata = collision.GetComponentInParent<ball_data>();

        if (balldata == null) { return; }

        // Mark the ball data as requested
        if (clear_flags)
        {
            balldata.flags.Clear();
        }

        if (add_key.Length > 0)
        {
            balldata.flags[add_key] = add_value;
        }

    }
}

