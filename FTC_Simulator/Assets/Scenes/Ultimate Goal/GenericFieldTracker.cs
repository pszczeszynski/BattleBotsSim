using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GenericFieldTracker : MonoBehaviour
{
    // List of robots inside the region
    public List<RobotID> robots = new List<RobotID>();
    public List<int> robot_collisions = new List<int>();

    public List<gameElement> game_elements = new List<gameElement>();
    public List<int> game_elements_collisions = new List<int>();

    public bool dont_clear_exited_items = false;  // If object exits box, don't clear it from our list. User can call clear exited items functions to clear them.
    public bool disable_in_client = true;
    public bool only_robot_body = false; // If true, only tracks robot body, not arms and such
    public bool track_gameelements = true;
    public bool track_robots = true;
    public bool ignore_self = false;
    public bool ignore_ge_off = false;

    public string note = "";  // If filled in, the note in the gameelement must match this note

    public gameElement self_ge = null;
    public RobotID self_ri = null;

    protected virtual void Start()
    {
        // Identify ourselves
        self_ge = GetComponentInChildren<gameElement>();
        if( !self_ge ) { self_ge = GetComponentInParent<gameElement>(); }

        self_ri = GetComponentInChildren<RobotID>();
        if (!self_ri) { self_ri = GetComponentInParent<RobotID>(); }
    }
   


    public virtual void Reset()
    {
        robots.Clear();
        robot_collisions.Clear();
        game_elements.Clear();
        game_elements_collisions.Clear();
    }
   

    /*   private void Update()
       {

       }
   */

    public bool IsGameElementInside(gameElement element)
    {
        return game_elements.Contains(element);
    }

    public bool IsAnyGameElementInside()
    {
        return game_elements.Count > 0;
    }

    public int GetGameElementCount()
    {
        return game_elements.Count;
    }

    public int GetGameElementCount(ElementType type)
    {
        int count = 0;

        foreach( gameElement currelement in game_elements)
        {
            if( currelement.type == type)
            {
                count += 1;
            }
        }

        return count;
    }

    public bool IsGameElementInside(Transform element)
    {
        return game_elements.Contains(element.GetComponentInParent<gameElement>());
    }

    public bool IsRobotInside(RobotID robot)
    {
        return robots.Contains(robot);
    }

    public bool IsRobotInside(Transform robot)
    {
        return robots.Contains(robot.GetComponentInParent<RobotID>());
    }
    public bool IsAnyRobotInside()
    {
        // Make sure to clear bad bots 
        if (robots.Count > 0)
        {
            RemoveInvalidItems();
        }
        return robots.Count > 0;
    }

    public int GetRedRobotCount()
    {
        int count = 0;

        foreach( RobotID curr in robots)
        {
            if( curr.is_red)
            { count += 1; }
        }

        return count;
    }

    public int GetBlueRobotCount()
    {
        int count = 0;

        foreach (RobotID curr in robots)
        {
            if (!curr.is_red)
            { count += 1; }
        }

        return count;
    }

    // Gets called when a game element is added
    public virtual void OnGameElementTriggerEnter( gameElement ge )
    {

    }

    // Gets called when a robot is added
    public virtual void OnRobotTriggerEnter(RobotID robot)
    {

    }

    // Gets called when a game element is added
    public virtual void OnGameElementTriggerExit(gameElement ge)
    {

    }

    // Gets called when a robot is added
    public virtual void OnRobotTriggerExit(RobotID robot)
    {

    }

    void OnTriggerEnter(Collider collision)
    {
        if (disable_in_client && GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        if (track_gameelements)
        {
            // See if we have a gameElement in the hierarchy
            gameElement game_element = collision.GetComponentInParent<gameElement>();

            if (game_element && ((game_element.type != ElementType.Off) || !ignore_ge_off))
            {
                // Ignore ourselves
                if (ignore_self && self_ge)
                {
                    if (game_element == self_ge) { return; }
                }

                // Skip if note doesn't match (and is defined)
                if( (note.Length > 0) && (note != game_element.note) ) { return; }

                AddGameElement(game_element);
                OnGameElementTriggerEnter(game_element);

                return;
            }
        }

        if (track_robots)
        {
            // See if this is a robot instead
            RobotID robot_id = collision.GetComponentInParent<RobotID>();

            if (robot_id)
            {
                // Only look at body colliders if that is the desire
                if (only_robot_body && !(collision.attachedRigidbody.transform.name.StartsWith( "Body")))
                {
                    return;
                }

                // Ignore collision boundry item
                if( collision.transform.name.StartsWith("collisionBoundry"))
                {
                    return;
                }

                // Ignore ourselves
                if (ignore_self && self_ri)
                {
                    if (robot_id == self_ri) { return; }
                }

                AddRobot(robot_id);
            }
        }
    }

   
    // Adds a robot to the lists
    private bool AddRobot(RobotID robotElement)
    {

        // Increment the collision count
        int index = robots.IndexOf(robotElement);

        // Increment collision counter
        if (index >= 0)
        {
            robot_collisions[index] += 1;
        }
        else // First time add robot to our list
        {
            robots.Add(robotElement);
            robot_collisions.Add(1);
        }

        return true;
    }

    // Adds a robot to the lists
    private bool AddGameElement(gameElement game_element)
    {

        // Increment the collision count
        int index = game_elements.IndexOf(game_element);

        // Increment collision counter
        if (index >= 0)
        {
            game_elements_collisions[index] += 1;
        }
        else // First time add robot to our list
        {
            game_elements.Add(game_element);
            game_elements_collisions.Add(1);
        }

        return true;
    }

    // Remove a Robot from the list if collisions reached 0 count
    // Returns true if it cleared it, false if it's still inside
    private bool RemoveExitedRobot(RobotID robotElement)
    {
        // Decerement the collision count
        int index = robots.IndexOf(robotElement);

        if (index >= 0)
        {
            robot_collisions[index] -= 1;
            if (!dont_clear_exited_items && (robot_collisions[index] <= 0))
            {
                robots.RemoveAt(index);
                robot_collisions.RemoveAt(index);
                RobotExited(robotElement);
                return true;
            }
        }

        return false;
    }

    // Remove a game elements that exited the box
    private bool RemoveExitedGameElements(gameElement robotElement)
    {

        // Decerement the collision count
        int index = game_elements.IndexOf(robotElement);

        if (index >= 0)
        {
            game_elements_collisions[index] -= 1;
            if (!dont_clear_exited_items && (game_elements_collisions[index] <= 0))
            {
                game_elements.RemoveAt(index);
                game_elements_collisions.RemoveAt(index);
                GameElementExited(robotElement);
                return true;
            }
        }

        return false;
    }

    virtual public void RobotExited(RobotID robotid)
    {

    }

    virtual public void GameElementExited(gameElement gamelement)
    {

    }

    void OnTriggerExit(Collider collision)
    {
        if (disable_in_client && GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        if (track_gameelements)
        {
            // See if we have a gameElement in the hierarchy
            gameElement game_element = collision.GetComponentInParent<gameElement>();

            if (game_element)
            {
                RemoveExitedGameElements(game_element);
                OnGameElementTriggerExit(game_element);
                return;
            }
        }

        if (track_robots)
        {
            // See if this is a robot instead
            RobotID robot_id = collision.GetComponentInParent<RobotID>();

            if (robot_id)
            {
                // Only look at body colliders if that is the desire
                if (only_robot_body && !(collision.attachedRigidbody.transform.name.StartsWith("Body")))
                {
                    return;
                }

                // Ignore collision boundry item
                if (collision.transform.name.StartsWith("collisionBoundry"))
                {
                    return;
                }

                RemoveExitedRobot(robot_id);
                OnRobotTriggerExit(robot_id);
            }
        }
    }

    // Removes any items that have exited - intended to be used when the "dont_clear_exited_items" is checked
    public void ClearExitedItem()
    {
        RemoveInvalidItems();
    }

    private void RemoveInvalidItems()
    {
        // Remove items without a transform OR who's collision count reached <=0
        int item = robots.Count - 1;
        while (item >= 0)
        {
            if ((robots[item] == null) || (robot_collisions[item] <= 0))
            {
                robots.RemoveAt(item);
                robot_collisions.RemoveAt(item);
            }

            item -= 1;
        }

        // Remove items without a transform
        item = game_elements.Count - 1;
        while (item >= 0)
        {
            if ((game_elements[item] == null) || (game_elements_collisions[item] <= 0))
            {
                game_elements.RemoveAt(item);
                game_elements_collisions.RemoveAt(item);
            }

            item -= 1;
        }
    }

    // returns the closest distance of a point to the tracker(s)
    public float GetClosestDistance(Vector3 point)
    {
        float distance = 99999f;

        foreach (Collider currcollider in GetComponentsInChildren<Collider>())
        {
            Vector3 newdistance = currcollider.ClosestPoint(point);
            if (Vector3.Distance(point, newdistance) < distance)
            {
                distance = Vector3.Distance(point, newdistance);
            }
        }

        return distance;

    }

}
