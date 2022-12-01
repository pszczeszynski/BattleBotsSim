using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JunctionStuff : MonoBehaviour
{
    public enum JunctionType
    {
        terminal = 1,
        ground = 2,
        low = 3,
        medium = 4,
        high = 5
    };

    public JunctionType myType = JunctionType.low;

    public List<ConeStuff> all_cones = new List<ConeStuff>();

    // public List<ConeStuff> blue_cones = new List<ConeStuff>();
    // public List<ConeStuff> red_cones = new List<ConeStuff>();

    public int red_count = 0;
    public int blue_count = 0;
    public bool blue_owned = false;
    public bool red_owned = false;
    public bool blue_capped = false;
    public bool red_capped = false;

    // Circuit information
    public List<Wire> connected_wires = new List<Wire>();
    public int weight = 9999;

    public class SorterByHeight : IComparer<ConeStuff>
    {
        public int Compare(ConeStuff x, ConeStuff y)
        {
            return x.transform.position.y.CompareTo( y.transform.position.y);
        }
    }
    SorterByHeight mysorter = new SorterByHeight();

    public void Update()
    {
        // Update our counts
        UpdateCounts();
    }

    public virtual void Reset()
    {
        all_cones.Clear();
        red_count = 0;
        blue_count = 0;
        blue_owned = false;
        red_owned = false;
        blue_capped = false;
        red_capped = false;
    }

    public void UpdateCounts()
    {
        // If this is a terminal, don't do anything
        if( myType == JunctionType.terminal)
        { return;  }

        // Sort list by height
        all_cones.Sort(mysorter);

        red_count = 0;
        blue_count = 0;
        blue_owned = false;
        red_owned = false;
        blue_capped = false;
        red_capped = false;

        for( int i =0; i < all_cones.Count; i++)
        {
            if( all_cones[i].isRed)
            {
                // Make sure it's upright
                if (all_cones[i].isConeUpright(transform.up, (myType == JunctionType.ground) ? 8f : 45f))
                {
                    red_count++;
                    red_owned = true;
                    blue_owned = false;
                }

                if( all_cones[i].isBeacon)
                {
                    red_capped = true;
                    red_owned = true;
                    blue_owned = false;
                    return;
                }
            }
            else
            {
                // Make sure it's upright
                if (all_cones[i].isConeUpright(transform.up, (myType == JunctionType.ground) ? 8f : 45f))
                {
                    blue_count++;
                    blue_owned = true;
                    red_owned = false;
                }

                if( all_cones[i].isBeacon)
                {
                    blue_capped = true;
                    blue_owned = true;
                    red_owned = false;
                    return;
                }
            }          
        }
    }

    public int GetBlueCount()
    {
        return blue_count;
    }

    public int GetRedCount()
    {
        return red_count;
    }

   

    void OnTriggerEnter(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // If this is a terminal, don't do anything
        if (myType == JunctionType.terminal)
        { return; }


        // Clean up list
        RemoveInvalidItems();

        // See if we have a cone in the hierarchy
        ConeStuff cone = collision.GetComponentInParent<ConeStuff>();

        if (!all_cones.Contains(cone))
        {
            all_cones.Add(cone);
        }   
       
    }

 

    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }
        // If this is a terminal, don't do anything
        if (myType == JunctionType.terminal)
        { return; }

        // Clean up list
        RemoveInvalidItems();

        // See if we have a cone in the hierarchy
        ConeStuff cone = collision.GetComponentInParent<ConeStuff>();

        if (all_cones.Contains(cone))
        {
            all_cones.Remove(cone);
        }
    }

  
    private void RemoveInvalidItems()
    {
        // Remove items without a transform OR who's collision count reached <=0
        int item = all_cones.Count - 1;
        while (item >= 0)
        {
            if (all_cones[item] == null) 
            {
                all_cones.RemoveAt(item);
            }

            item -= 1;
        }   
    }

}
