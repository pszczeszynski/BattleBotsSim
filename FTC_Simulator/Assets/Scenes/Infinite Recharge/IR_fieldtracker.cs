using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IR_fieldtracker : MonoBehaviour
{
    InfiniteRecharge_Settings ir_settings = null;
    public bool isRed = false;  // True if red, otherwise blue
    public bool everyone_is_friend = false; // Ignore collor if this is true

    // list of elements colliding
    public List<gameElement> collisions = new List<gameElement>();
    public bool reset_enemies = false; // enemies get instantly reset
    public bool disable_in_client = true;

    private void Start()
    {
        if (!everyone_is_friend)
        {
            ir_settings = GameObject.Find("GameSettings").GetComponent<InfiniteRecharge_Settings>();
        }
    }

    public void Reset()
    {

    }

    private void Update()
    {
        if (!everyone_is_friend && (ir_settings == null))
        { ir_settings = GameObject.Find("GameSettings").GetComponent<InfiniteRecharge_Settings>(); }

    }

    // List of teamates inside the region
    public List<Transform> friends = new List<Transform>();
    private List<int> friends_collisions = new List<int>();

    void OnTriggerEnter(Collider collision)
    {
        if(disable_in_client && GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // If this is an enemy, mark if for reset
        if (!everyone_is_friend && reset_enemies && IsEnemy(collision))
        {
            // If object is already marked for reset, then skip
            if (collision.transform.root.GetComponent<RobotInterface3D>().GetNeedsReset())
            { return; }

            if (ir_settings.ENABLE_BLOCKING)
            {
                collision.transform.root.GetComponent<RobotInterface3D>().MarkForReset(ir_settings.RESET_DURATION_UNDER_SKYBRIDGE);
            }
            return;
        }

        // If this is a friendly, add it to our list
        if (AddFriend(collision)) { return; }

    }

    public bool IsFriendInside(Transform friend)
    {
        return friends.Contains(friend);
    }

    public float GetClosestDistance(Vector3 point)
    {
        float distance = 99999f;

        foreach( Collider currcollider in GetComponentsInChildren<Collider>())
        {
            Vector3 newdistance = currcollider.ClosestPoint(point);
            if( Vector3.Distance(point,newdistance) < distance)
            {
                distance = Vector3.Distance(point, newdistance);
            }
        }

        return distance;

    }

    // Returns true if the collision is an enemy robot
    private bool IsEnemy(Collider collision)
    {
        if(everyone_is_friend) { return false; }

        // Ignore the "Body" bounding box - it is used to determine robot-to-robot collisions, not to determine if robot is inside something
        if (collision.name.Equals("Body")) { return false; }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }

        if ((robotElement.starting_pos.ToString().StartsWith("Red") && isRed) ||
              (robotElement.starting_pos.ToString().StartsWith("Blue") && !isRed))
        {
            return false;
        }

        // This is an enemy
        return true;
    }

    // Adds a friend robot to the list and/or increment its collision counter
    private bool AddFriend(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }

        if (!everyone_is_friend && !((robotElement.starting_pos.ToString().StartsWith("Blue") && !isRed) ||
              (robotElement.starting_pos.ToString().StartsWith("Red") && isRed)))
        {
            return false;
        }

        // Increment the collision count
        int index = friends.IndexOf(topparent);

        // Increment collision counter
        if (index >= 0)
        {
            friends_collisions[index] += 1;
        }
        else // First time this friend is added
        {
            friends.Add(topparent);
            friends_collisions.Add(1);
        }

        return true;
    }

    // Remove a friends from the list and/or remove its collision counter
    private bool RemoveFriend(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }

        if (!everyone_is_friend && !((robotElement.starting_pos.ToString().StartsWith("Blue") && !isRed) ||
              (robotElement.starting_pos.ToString().StartsWith("Red") && isRed)))
        {
            return false;
        }

        // Decerement the collision count
        int index = friends.IndexOf(topparent);

        // This is a friend
        if (index >= 0)
        {
            friends_collisions[index] -= 1;
            if (friends_collisions[index] <= 0)
            {
                friends.RemoveAt(index);
                friends_collisions.RemoveAt(index);
            }
        }

        return true;
    }

    void OnTriggerExit(Collider collision)
    {
        if (disable_in_client && GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // Remove any friends
        if (RemoveFriend(collision)) { return; }

    }

    private void RemoveInvalidItems()
    {
        // Remove items without a transform
        int item = friends.Count - 1;
        while (item >= 0)
        {
            if (friends[item] == null)
            {
                friends.RemoveAt(item);
                friends_collisions.RemoveAt(item);
            }

            item -= 1;
        }
    }

}
