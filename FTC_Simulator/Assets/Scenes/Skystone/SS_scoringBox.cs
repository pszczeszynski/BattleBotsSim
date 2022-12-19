using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// This scoring box is for the power line transitions
public class SS_scoringBox : MonoBehaviour
{

    Skystone_Settings ss_settings = null;
    public bool isRed = false;  // True if red, otherwise blue

    // list of elements colliding
    public List<gameElement> collisions = new List<gameElement>();
    public List<int> collisions_count = new List<int>();

    public int score = 0;
    public int penalties = 0;
    public bool foundation_box = false;
    public bool non_scoring = false; // If set true, will not keep track of score
    public bool reset_enemies = true; // enemies get instantly reset

    private void Start()
    {
        if (GameObject.Find("GameSettings"))
        {
            ss_settings = GameObject.Find("GameSettings").GetComponent<Skystone_Settings>();
        }
    }

    public void Reset()
    {
        score = 0;
        penalties = 0;
    }

    private void Update()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        if (ss_settings == null)
        { ss_settings = GameObject.Find("GameSettings").GetComponent<Skystone_Settings>(); }

        //
    }

    private gameElement FindGameElement(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform;

        gameElement collision_element = topparent.GetComponent<gameElement>();

        // While we haven't found a gameElement and there is more parents, keep searching for it
        while (collision_element == null && topparent.parent != null)
        {
            topparent = topparent.parent;
            collision_element = topparent.GetComponent<gameElement>();
        }

        // If this isn't an element of the proper type, return null
        if (collision_element == null) { return null; }
        if (collision_element.type == ElementType.Cube) { return collision_element; }  // Return if it's of type cube/Block
        return null;
    }

    // List of teamates inside the region
    public List<Transform> friends = new List<Transform>();
    private List<int> friends_collisions = new List<int>();

    void OnTriggerEnter(Collider collision)
    {
        if( GLOBALS.CLIENT_MODE) { return; }
        // Clean up list
        RemoveInvalidItems();

        // Ignore the collision boundry of the robot 
        if (collision.transform.name.StartsWith("collisionBoundry")) { return; }
        else if (collision.GetComponent<Collider>() == null) { return; }

        // If this is an enemy, mark if for reset
        // An enemy needs to have a RobotID, if not, this is skipped
        if (reset_enemies && IsEnemy(collision))
        {
            // If object is already marked for reset, then skip
            if(collision.transform.root.GetComponent<RobotInterface3D>().GetNeedsReset())
            { return; }

            if (ss_settings.ENABLE_SKYBRIDGE_PENALTIES)
            {
                collision.transform.root.GetComponent<RobotInterface3D>().MarkForReset(ss_settings.RESET_DURATION_UNDER_SKYBRIDGE);
            }

            penalties += (foundation_box) ? ss_settings.PENALTY_SCORING : ss_settings.PENALTY_SKYBRIDGE;
            return;
        }

        // If this is a friendly, add it to our list
        // A friend needs to have a RobotID, otherwise this is skipped
        if (AddFriend(collision)) { return; }

        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        if (collision_element == null) { return; }

        // Update the block's state
        if (!foundation_box && !non_scoring)
        {
            collision_element.GetComponent<block_score_data>().StartTransition(this.transform.position.x);
        }
        // If this is a foundation box, we still want to check all components of object are out of box before removing it,
        // so need to track it


        // Add it to our list
        if (collisions.Contains(collision_element)) {
            collisions_count[collisions.IndexOf(collision_element)] += 1;
        }
        else { 
            collisions.Add(collision_element);
            collisions_count.Add(1);
        }
    }

    public bool IsFriendInside(Transform friend)
    {
        return friends.Contains(friend);
    }

    // Returns true if the collision is an enemy robot
    private bool IsEnemy(Collider collision)
    {
        // Ignore the "Body" bounding box - it is used to determine robot-to-robot collisions, not to determine if robot is inside something
        if( collision.name.Equals("Body") ) { return false; }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }

        if ( (robotElement.starting_pos.ToString().StartsWith("Red") && isRed) ||
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

        if (!((robotElement.starting_pos.ToString().StartsWith("Blue") && !isRed) ||
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

        if (!((robotElement.starting_pos.ToString().StartsWith("Blue") && !isRed) ||
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
            if(friends_collisions[index] <= 0)
            {
                friends.RemoveAt(index);
                friends_collisions.RemoveAt(index);
            }
        }

        return true;
    }

    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // Remove any friends
        if ( RemoveFriend(collision)) { return; }

        // Get the associated game element
        gameElement collision_element = FindGameElement(collision);

        // Make sure we found something
        if (collision_element == null) { return; }
        
        // Add/Subtract points according to previous state
        if (!foundation_box && !non_scoring)
        {
            score += collision_element.GetComponent<block_score_data>().EndTransition(this.transform.position.x);
        }

        // Remove it from our list if it's ready
        if (collisions.Contains(collision_element) )
        {
            // If this is a foundation box, we do the tracking here and only want to remove it once it's 0
            if (foundation_box)
            {
                int index = collisions.IndexOf(collision_element);
                collisions_count[index] -= 1;
                
                if(collisions_count[index] <= 0 )
                {
                    collisions_count.RemoveAt(index);
                    collisions.RemoveAt(index);
                }
            }

            // Otherwise use the object's tracking system if it's being used
            // the blocks trackign does additional state changes unlike the simplistic scoring box one
            else if (collision_element.GetComponent<block_score_data>().OutOfRegion())
            {
                collisions.Remove(collision_element);
            }    
        }


    }

    public int GetElementCount()
    {
        return collisions.Count;
    }

    // Returns the highest block as a function of block height
    public float block_height = 0.2f;

    public int GetHighestBlock( )
    {
        if( non_scoring ) { return 0; }

        int maxheight = 0;
        int currheight = 0;
        float raw_height = 0;

        foreach( gameElement curritem in collisions)
        {
            // This must be a valid block
            block_score_data currblock = curritem.GetComponent<block_score_data>();
            if( currblock == null) { continue; }
            if (currblock.currposition == PositionState.off) { continue; }
            if( currblock.held_by_robot > 0) { continue; }

            raw_height = curritem.transform.position.y - (transform.position.y - transform.localScale.y/2f);
            currheight = (int) Math.Floor( raw_height / block_height + block_height);

            // An interlocked block has a raw_height of 0, so we must add "1" to the height
            currheight++;
            
            if (currheight > maxheight)
            {
                maxheight = currheight;
            }
        }

        return maxheight;
    }

    public bool IsGameElementInside(gameElement item)
    {
        return collisions.Contains(item);
    }

    private void RemoveInvalidItems()
    {
        int item = collisions.Count - 1;
        while (item >= 0)
        {
            if (collisions[item] == null)
            {
                collisions.RemoveAt(item);
                collisions_count.RemoveAt(item);
            }

            item -= 1;
        }

        // Remove items without a transform
        item = friends.Count - 1;
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

