using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class FRC_DepotHandler : MonoBehaviour
{
    InfiniteRecharge_Settings ir_settings = null;

    public bool isRed = false;  // True if red, otherwise blue
    public bool keep_out_enemies = false;


    // Start is called before the first frame update 
    void Start()
    { 
        ir_settings = GameObject.Find("GameSettings").GetComponent<InfiniteRecharge_Settings>();
    }

    void FixedUpdate()
    {
        // Apply a pushing force to opposite players
        if (keep_out_enemies && ir_settings.ENABLE_DEPOT_PENALTIES) { PushOppositePlayers(); }

    }


    void Update()
    {
        if( ir_settings == null )
        { ir_settings = GameObject.Find("GameSettings").GetComponent<InfiniteRecharge_Settings>(); }

    }

    // Gets called on a field reset after all objects are re-initialized
    public void Reset()
    {

        // Reset all game-objects calls trigger exit function which will add an extra block once all things exited.
        
        // Clean up list
        RemoveInvalidItems();

    }
  
    // List of enemies and friends inside us
    public List<RobotInterface3D> enemies = new List<RobotInterface3D>();
    private List<int> enemies_collisions = new List<int>();

    public List<RobotInterface3D> friends = new List<RobotInterface3D>();
    private List<int> friends_collisions = new List<int>();

    public void OnTriggerEnter(Collider collision)
    {
        // Clean up list
        RemoveInvalidItems();

        // If this is an enemy, record our collision
        AddEnemy(collision);
        AddFriend(collision);
    }

    public void OnTriggerExit(Collider collision)
    {
        // Clean up list
        RemoveInvalidItems();

        // If this is an enemy, record our collision
        RemoveEnemy(collision);
        RemoveFriend(collision);
    }

    private void PushOppositePlayers()
    {
        foreach (RobotInterface3D enemy in enemies)
        {
            Transform body = enemy.transform.Find("Body");
            // Ok, now time to push the enemy out of the box
            Rigidbody enemy_rb = body.GetComponent<Rigidbody>();
            if (enemy_rb == null) { return; }

            enemy_rb.velocity = Vector3.zero;
            enemy_rb.angularVelocity = Vector3.zero;

            // And finally we add force in opposite direction to use
            enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized * 200f, ForceMode.Acceleration);
        }
    }

    // Returns true if the collision is an enemy robot
    private bool IsEnemy(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }

        if (robotElement.starting_pos.ToString().StartsWith("Red")  && isRed ||
            robotElement.starting_pos.ToString().StartsWith("Blue") && !isRed)
        {
            return false;
        }

        // This is an enemy
        return true;
    }

    // Returns true if the collision is an enemy robot
    private bool IsFriend(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return false; }

        if (robotElement.starting_pos.ToString().StartsWith("Red") && !isRed ||
            robotElement.starting_pos.ToString().StartsWith("Blue") && isRed)
        {
            return false;
        }

        // This is a friend
        return true;
    }


    // Adds an enemy robot to the list and/or increment its collision counter
    private bool AddEnemy(Collider collision)
    {
        if( !IsEnemy(collision)) { return false;  }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // Increment the collision count
        RobotInterface3D newrobot = topparent.GetComponent<RobotInterface3D>();
        int index = enemies.IndexOf(newrobot);

        // Increment collision counter
        if (index >= 0)
        {
            enemies_collisions[index] += 1;
        }
        else // First time this enemies is added
        {
            enemies.Add(newrobot);
            enemies_collisions.Add(1);
        }

        return true;
    }

    // Adds a Friend robot to the list and/or increment its collision counter
    private bool AddFriend(Collider collision)
    {
        if (!IsFriend(collision)) { return false; }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // Increment the collision count
        RobotInterface3D newrobot = topparent.GetComponent<RobotInterface3D>();
        int index = friends.IndexOf(newrobot);

        // Increment collision counter
        if (index >= 0)
        {
            friends_collisions[index] += 1;
        }
        else // First time this enemies is added
        {
            friends.Add(newrobot);
            friends_collisions.Add(1);
        }

        return true;
    }


    // Remove a enemies from the list and/or remove its collision counter
    private bool RemoveEnemy(Collider collision)
    {
        if (!IsEnemy(collision)) { return false; }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // Decerement the collision count
        RobotInterface3D newrobot = topparent.GetComponent<RobotInterface3D>();
        int index = enemies.IndexOf(newrobot);

        // This is an enemy
        if (index >= 0)
        {
            enemies_collisions[index] -= 1;
            if (enemies_collisions[index] <= 0)
            {
                enemies.RemoveAt(index);
                enemies_collisions.RemoveAt(index);
            }
        }

        return true;
    }

    // Remove a friend from the list and/or remove its collision counter
    private bool RemoveFriend(Collider collision)
    {
        if (!IsFriend(collision)) { return false; }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();

        // Decerement the collision count
        RobotInterface3D newrobot = topparent.GetComponent<RobotInterface3D>();
        int index = friends.IndexOf(newrobot);

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


    private void RemoveInvalidItems()
    {
        // Remove items without a transform
        int item = enemies.Count - 1;
        while (item >= 0)
        {
            if (enemies[item] == null)
            {
                enemies.RemoveAt(item);
                enemies_collisions.RemoveAt(item);
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

    // Returns true if there are enemy collisions
    public bool AreEnemiesColliding()
    {
        RemoveInvalidItems();
        return enemies.Count > 0;
    }

    public bool AreFriendsColliding()
    {
        RemoveInvalidItems();
        return friends.Count > 0;
    }


    public List<RobotInterface3D> GetAllEnemies()
    {
        RemoveInvalidItems();
        return enemies;
    }

    public List<RobotInterface3D> GetAllFriends()
    {
        RemoveInvalidItems();
        return friends;
    }
    
    public bool IsFriendInside( RobotInterface3D friend )
    {
        return friends.Contains(friend);
    }

    public bool IsEnemyInside(RobotInterface3D friend)
    {
        return enemies.Contains(friend);
    }

    // empties lists
    public void Clear()
    {
        enemies.Clear();
        enemies_collisions.Clear();
        friends.Clear();
        friends_collisions.Clear();
        RemoveInvalidItems();
    }
}
