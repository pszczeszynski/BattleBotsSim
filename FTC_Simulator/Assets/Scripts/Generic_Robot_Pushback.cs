using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Generic_Robot_Pushback : MonoBehaviour
{
    public bool enable = true;  // Easy way to enable/disable. If enabled, still keeps track of robots entering/leaving, ready to push when necessary
    public bool push_only_body = true; // Pushes only body if true
    public bool push_red = false;  // Pushes red if true
    public bool push_blue = false; // pushes blue if true

    public float force_scale = 200f;



    // Start is called before the first frame update 
    void Start()
    { 

    }

    void FixedUpdate()
    {
        // Apply a pushing force to opposite players
        PushRobots(); 

    }


    void Update()
    {
 

    }

    // Gets called on a field reset after all objects are re-initialized
    public void Reset()
    {
        
        // Clean up list
        RemoveInvalidItems();

    }
  
    // List of robots in and out
    public List<RobotID> bots_red = new List<RobotID>();
    private List<int> bots_red_count = new List<int>();

    public List<RobotID> bots_blue = new List<RobotID>();
    private List<int> bots_blue_count = new List<int>();

    public void OnTriggerEnter(Collider collision)
    {
        // Clean up list
        RemoveInvalidItems();

        // Record Collision
        AddBot(collision);
    }

    public void OnTriggerExit(Collider collision)
    {
        // Clean up list
        RemoveInvalidItems();

        // Remove the bot
        RemoveBot(collision);
    }

    private void PushRobots()
    {
        // If pushing disabled, don't push
        if( !enable) { return; }

        // Push red
        if (push_red)
        {
            foreach (RobotID curr_bot in bots_red)
            {
                PushBot(curr_bot);
            }
        }

        // Push blue
        if (push_blue)
        {
            foreach (RobotID curr_bot in bots_blue)
            {
                PushBot(curr_bot);
            }
        }
    }

    private void PushBot(RobotID bot)
    {
        // Push body if body is enabled
        if( push_only_body )
        {
            Transform body = bot.transform.Find("Body");
            if( !body ) { return; } // Should never happen

            // Ok, now time to push the bot out of the box
            Rigidbody bots_rb = body.GetComponent<Rigidbody>();
            if (bots_rb == null) { return; } // Should never happen

            bots_rb.velocity = Vector3.zero;
            bots_rb.angularVelocity = Vector3.zero;

            // And finally we add force in opposite direction to use
            bots_rb.AddForce((bots_rb.transform.position - transform.position).normalized * force_scale, ForceMode.Acceleration);
            return;
        }

        // Otherwise go through every rigid body and push back
        foreach( Rigidbody curr_rb in bot.GetComponentsInChildren<Rigidbody>())
        {
            curr_rb.velocity = Vector3.zero;
            curr_rb.angularVelocity = Vector3.zero;

            // And finally we add force in opposite direction to use
            curr_rb.AddForce((curr_rb.transform.position - transform.position).normalized * force_scale, ForceMode.Acceleration);
        }
    }

    // Returns true if the collision is a bot
    // Searches for isRed or isBlue or both (if both are true)
    private RobotID GetRobotID(Collider collision, bool isRed, bool isBlue)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return null; }
        
        if (robotElement.is_red  && isRed ||
            !robotElement.is_red && isBlue)
        {
            return robotElement;
        }

        // This is not valid
        return null;
    }



    // Adds an enemy robot to the list and/or increment its collision counter
    private bool AddBot(Collider collision)
    {
        // Check if this is either red or blue bot
        RobotID robotElement = GetRobotID(collision, true, true);
        if ( !robotElement) { return false;  }

        // Get the correct color counter
        List<RobotID> bots = (robotElement.is_red) ? bots_red : bots_blue;
        List<int> bots_count = (robotElement.is_red) ? bots_red_count : bots_blue_count;

        // Increment the collision count
        int index = bots.IndexOf(robotElement);

        // Increment collision counter
        if (index >= 0)
        {
            bots_count[index] += 1;
        }
        else // First time this is added
        {
            bots.Add(robotElement);
            bots_count.Add(1);
        }

        return true;
    }


    // Remove a enemies from the list and/or remove its collision counter
    private bool RemoveBot(Collider collision)
    {
        // Check if this is either red or blue bot
        RobotID robotElement = GetRobotID(collision, true, true);
        if (!robotElement) { return false; }

        // Get the correct color counter
        List<RobotID> bots = (robotElement.is_red) ? bots_red : bots_blue;
        List<int> bots_count = (robotElement.is_red) ? bots_red_count : bots_blue_count;

        // Increment the collision count
        int index = bots.IndexOf(robotElement);

        // This is an enemy
        if (index >= 0)
        {
            bots_count[index] -= 1;
            if (bots_count[index] <= 0)
            {
                bots.RemoveAt(index);
                bots_count.RemoveAt(index);
            }
        }

        return true;
    }



    private void RemoveInvalidItems()
    {
        // Remove items without a transform
        int item = bots_red.Count - 1;
        while (item >= 0)
        {
            if (bots_red[item] == null)
            {
                bots_red.RemoveAt(item);
                bots_red_count.RemoveAt(item);
            }

            item -= 1;
        }

        // Remove items without a transform
        item = bots_blue.Count - 1;
        while (item >= 0)
        {
            if (bots_blue[item] == null)
            {
                bots_blue.RemoveAt(item);
                bots_blue_count.RemoveAt(item);
            }

            item -= 1;
        }
    }

    public bool IsBotInsdide( RobotID bot)
    {
        return bots_blue.Contains(bot) || bots_red.Contains(bot);
    }

    // empties lists
    public void Clear()
    {
        bots_red.Clear();
        bots_red_count.Clear();
        bots_blue.Clear();
        bots_blue_count.Clear();
        RemoveInvalidItems();
    }
}
