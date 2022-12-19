using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class DepotHandler : MonoBehaviour
{
    Skystone_Settings ss_settings = null;

    public bool isRed = false;  // True if red, otherwise blue
    public int curr_block = 1;         // the block number to create if the space is clear
    public GameObject block_drop_pos = null;

    // list of elements colliding
    // NEED TO BE CAREFUL: box can not collide with MAT or side-walls, so position on field carefully
    // Issue with collision list: if an object is destroyed (e.g. when a player resets their robot or level is restarted), the OnTriggerExit is not called.
    // Furthermore, the list now contains invalid objects.
    // We need to therefore clean it up every so often
    public List<Transform> collisions = new List<Transform>();
    public List<Transform> enemies = new List<Transform>();
    public List<Rigidbody> enemies_ri3d = new List<Rigidbody>();
    private List<int> enemies_collisions = new List<int>();



    // Start is called before the first frame update 
    void Start()
    {
        // Add first block
        Add_Block();

        if (GameObject.Find("GameSettings"))
        {
            ss_settings = GameObject.Find("GameSettings").GetComponent<Skystone_Settings>();
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Apply a pushing force to opposite players
        if (ss_settings && ss_settings.ENABLE_DEPOT_PENALTIES) { PushOppositePlayers(); }

    }

    void Update()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        if ( ss_settings == null )
        { ss_settings = GameObject.Find("GameSettings").GetComponent<Skystone_Settings>(); }

        // Sometimes it messes up after field reset... test this conditions here and fix it
        if( (collisions.Count == 0) && (curr_block == 1) && (block_blanking >= 10))
        {
            Add_Block();
        }
        if (block_blanking < 10)
        {
            block_blanking += 1; // Sometime there's a race-condition where collisions isn't incremended in time for this not to happen twice. Blank for a min number of frames.
        }

    }

    // Gets called on a field reset after all objects are re-initialized
    public void Reset()
    { 
        curr_block = 1;

        // Reset all game-objects calls trigger exit function which will add an extra block once all things exited.
        
        // Clean up list
        RemoveInvalidItems();

    }

    private void PushOppositePlayers()
    {
        if( GLOBALS.CLIENT_MODE) { return; }

        foreach (Rigidbody enemy_rb in enemies_ri3d)
        {
            if (enemy_rb == null) { return; }

            enemy_rb.velocity = Vector3.zero;
            enemy_rb.angularVelocity = Vector3.zero;

            Vector3 delta_pos = enemy_rb.transform.position - transform.position;
            delta_pos.y = 0f;

            // And finally we add force in opposite direction to use
            enemy_rb.AddForce(delta_pos.normalized * 0.5f, ForceMode.VelocityChange);

            // enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized * 30, ForceMode.Impulse);
        }

        /*foreach (Transform enemy in enemies)
        {
            if (!enemy) { continue;  }
                
            Transform topenemy = enemy;

            // Ok, now time to push the item out of the box
            // We will push objects away from center
            Rigidbody enemy_rb = topenemy.GetComponent<Rigidbody>(); ;

            while((enemy_rb == null) && (topenemy != null) )
            {
                topenemy = topenemy.parent;

                if (topenemy)
                {
                    enemy_rb = topenemy.GetComponent<Rigidbody>();
                }
            }
            if( enemy_rb == null) { return;}

            // And finally we add force in opposite direction to use

            enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized* 300);

        }
        */
    }

    private void RememberEnemies(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();
        RobotInterface3D ri3d = topparent.GetComponent<RobotInterface3D>();

        // If this isn't an element of the proper type, then exit
        if ((robotElement == null) || (!ri3d)) { return; }

        if (!((robotElement.starting_pos.ToString().StartsWith("Red") && !isRed) ||
              (robotElement.starting_pos.ToString().StartsWith("Blue") && isRed)))
        {
            return;
        }

        // Find the body
        Transform body = ri3d.transform.Find("Body");
        if (!body) { return; }

        // Ok, now time to push the enemy out of the box
        Rigidbody enemy_rb = body.GetComponent<Rigidbody>();
        if (enemy_rb == null) { return; }

        // This is an enemy object, add it in
        enemies.Add(collision.transform);
        int index = enemies_ri3d.IndexOf(enemy_rb);
        if( index < 0 )
        {
            enemies_ri3d.Add(enemy_rb);
            enemies_collisions.Add(1);
        }
        else
        {
            enemies_collisions[index] += 1;
        }

    }

    // Remove a enemies from the list and/or remove its collision counter
    private bool RemoveEnemy(Collider collision)
    {
        // Find the top parent
        Transform topparent = collision.transform.root;
        RobotInterface3D ri3d = topparent.GetComponent<RobotInterface3D>();
        if( !ri3d ) { return false;  }

        // Find the body
        Transform body = ri3d.transform.Find("Body");
        if (!body) { return false; }

        // Ok, now time to push the enemy out of the box
        Rigidbody enemy_rb = body.GetComponent<Rigidbody>();
        if (enemy_rb == null) { return false; }

        // Find and locate enemy
        int index = enemies_ri3d.IndexOf(enemy_rb);
        if (index < 0)
        {
            return false;
        }

        enemies_collisions[index] -= 1;

        if (enemies_collisions[index] <= 0)
        {
            enemies_ri3d.RemoveAt(index);
            enemies_collisions.RemoveAt(index);
            return true;
        }

        return false;      
    }




    // If there is an object entering, keep track of it and apply counter measured if required
    void OnTriggerEnter(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // Ignore if this is the collisionBoundry
        if( collision.transform.name.StartsWith("collisionBoundry") ) { return; }

        // Add object to our list
        collisions.Add(collision.transform);

        // Remember enemies for repulsion
        RememberEnemies(collision);
    }

    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Remove it from our list
        if (collisions.Contains(collision.transform))
        { collisions.Remove(collision.transform); }
        if (enemies.Contains(collision.transform))
        { enemies.Remove(collision.transform); }

        // Clean up list
        RemoveInvalidItems();

        // Remove enemies if applicable
        RemoveEnemy(collision);

        // Add a block if appropriate
        if ((collisions.Count == 0) && (curr_block <= 24))
        {
            Add_Block();
        }

    }

    long block_blanking = 0;

    // Moves another block onto field
    void Add_Block()
    {
        if( block_blanking < 10) { return; }
        block_blanking = 0;

        // Enable a new block
        GameObject nextblock;

        if (isRed) { nextblock = GameObject.Find("Extra Blocks Red/block_extra (" + curr_block + ")"); }
        else { nextblock = GameObject.Find("Extra Blocks Blue/block_extra (" + curr_block + ")"); }

        // Move onto field and unlock it
        nextblock.transform.position = block_drop_pos.transform.position;
        nextblock.transform.rotation = block_drop_pos.transform.rotation;
        nextblock.GetComponent<Rigidbody>().isKinematic = false;
        nextblock.GetComponent<block_score_data>().currposition = PositionState.Loading;
        curr_block = curr_block + 1;
    }

    // Remove items without a transform
    void RemoveInvalidItems()
    {
        int item = collisions.Count - 1;
        while (item >= 0)
        {
            if( collisions[item] == null)
            {
                collisions.RemoveAt(item);
            }

            item -= 1;
        }

        item = enemies.Count - 1;
        while (item >= 0)
        {
            if (enemies[item] == null)
            {
                enemies.RemoveAt(item);
            }

            item -= 1;
        }

        item = enemies_ri3d.Count - 1;
        while (item >= 0)
        {
            if (enemies_ri3d[item] == null)
            {
                enemies_ri3d.RemoveAt(item);
                enemies_collisions.RemoveAt(item);
            }

            item -= 1;
        }
    }

}
