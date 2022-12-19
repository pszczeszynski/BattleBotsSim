using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// counts how many rings are inside it
public class GoalScorer : MonoBehaviour
{
    // Low, Mid and High Ring Counters
    public List<RingCounter> low_counter = new List<RingCounter>();  // Checks for rings present
    public List<RingCounter> mid_counter = new List<RingCounter>();  // Checks for InnerRingID present
    public List<RingCounter> high_counter = new List<RingCounter>(); // Check for InnerRingID present

    public float time_last_updated = 0;
    int low_count = 0;
    int mid_count = 0;
    int high_count = 0;

    public bool isRed = false;
    public bool isBlue = false;
    public bool isYellow = false;

    public void UpdateCounts()
    {
        // If already updated this cycle, don't do it twice
        if (time_last_updated == Time.time)
        {
            return;
        }

        time_last_updated = Time.time;

        low_count = 0;
        mid_count = 0;
        high_count = 0;

        // Go from high to low hit points.
        // Make list of rings counted, and eliminate them from consideration
        List<gameElement> found_rings = new List<gameElement>();

        // Start with high counter
        foreach( RingCounter curr_counter in high_counter)
        {
            int index = curr_counter.collisions_ir_count.Count - 1;

            for(; index >=0; index-- )
            {
                if( curr_counter.collisions_ir_count[index] < 2)
                { continue; }
               
                found_rings.Add(curr_counter.collisions_ir[index]);

                // Only add ring in if it isn't in contact with a robot
                RobotCollision robot_collision = curr_counter.collisions_ir[index].GetComponent<RobotCollision>();

                if( robot_collision && (robot_collision.GetRobotCount() > 0) )
                {
                    continue;
                }

                high_count++;
            }
        }

        // Next do mid
        foreach (RingCounter curr_counter in mid_counter)
        {
            int index = curr_counter.collisions_ir_count.Count - 1;

            for (; index >= 0; index--)
            {
                if (curr_counter.collisions_ir_count[index] < 2)
                { continue; }

                found_rings.Add(curr_counter.collisions_ir[index]);

                // Only add ring in if it isn't in contact with a robot
                RobotCollision robot_collision = curr_counter.collisions_ir[index].GetComponent<RobotCollision>();

                if (robot_collision && (robot_collision.GetRobotCount() > 0))
                {
                    continue;
                }

                mid_count++;
            }
        }

        // Finally do low
        foreach (RingCounter curr_counter in low_counter)
        {
            foreach (gameElement curr_element in curr_counter.collisions)
            {
                if (found_rings.Contains(curr_element)) { continue; }

                found_rings.Add(curr_element);

                // Only add ring in if it isn't in contact with a robot
                RobotCollision robot_collision = curr_element.GetComponent<RobotCollision>();

                if (robot_collision && (robot_collision.GetRobotCount() > 0))
                {
                    continue;
                }

                low_count++;
            }
        }

    }

    public int GetLowCount()
    {
        UpdateCounts();

        return low_count;
    }

    public int GetMidCount()
    {
        UpdateCounts();

        return mid_count;
    }

    public int GetHighCount()
    {
        UpdateCounts();

        return high_count;
    }

}

