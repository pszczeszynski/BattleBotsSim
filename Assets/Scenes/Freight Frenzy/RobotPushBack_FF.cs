using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class RobotPushBack_FF : MonoBehaviour
{
    FreightFrenzy_Settings tp_settings = null;

  
    // list of elements colliding
    public List<Transform> enemies = new List<Transform>();
    public List<Rigidbody> enemies_ri3d = new List<Rigidbody>();
    private List<int> enemies_collisions = new List<int>();
    public float force = 200f;


    // Start is called before the first frame update 
    void Start()
    {

        if (GameObject.Find("GameSettings"))
        {
            tp_settings = GameObject.Find("GameSettings").GetComponent<FreightFrenzy_Settings>();
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Apply a pushing force to opposite players
        if (tp_settings && tp_settings.ENABLE_AUTO_PUSHBACK) { PushPlayers(); }

    }


    // Gets called on a field reset after all objects are re-initialized
    public void Reset()
    {         
        // Clean up list
        RemoveInvalidItems();

    }

    private void PushPlayers()
    {
        if( GLOBALS.CLIENT_MODE) { return; }

        foreach (Rigidbody enemy_rb in enemies_ri3d)
        {
            if (enemy_rb == null) { return; }

            // Zero out z-direction velocity
            Vector3 velocity = enemy_rb.velocity;
            velocity.z = 0;
            enemy_rb.velocity = velocity;
            enemy_rb.angularVelocity = Vector3.zero;

            Vector3 force_dir= new Vector3(0, 0, (enemy_rb.transform.position - transform.position).z);

            // And finally we add force in opposite direction to use
            enemy_rb.AddForce(force_dir.normalized * force, ForceMode.Force);

            // enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized * 30, ForceMode.Impulse);
        }
    }

    // Returns true is transform is part of the "Body" object
    private bool IsPartOfBody(Transform start)
    {
        bool body_found = false;

        // Make sure this isn't the collisionBoundry block
        if( start.name.StartsWith("collisionBoundry")) { return false; }

        // Go up hierarchy until we find the body
        while(start && !body_found)
        {
            if( start.name == "Body") { body_found = true;  }

            start = start.parent;
        }

        return body_found;
    }

    private void RememberEnemies(Collider collision)
    {
        // See if this collision is part of the Body
        if( !IsPartOfBody(collision.transform)) { return; }

        // Find the top parent
        Transform topparent = collision.transform.root;

        RobotID robotElement = topparent.GetComponent<RobotID>();
        RobotInterface3D ri3d = topparent.GetComponent<RobotInterface3D>();

        // If this isn't an element of the proper type, then exit
        if ((robotElement == null) || (!ri3d)) { return; }

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

        if (enemies_collisions[index] >= 0)
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

        // Remember enemies for repulsion
        RememberEnemies(collision);
    }

    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Remove it from our list
        if (enemies.Contains(collision.transform))
        { enemies.Remove(collision.transform); }

        // Clean up list
        RemoveInvalidItems();

        // Remove enemies if applicable
        RemoveEnemy(collision);

    }


    // Remove items without a transform
    void RemoveInvalidItems()
    {
        int item = enemies.Count - 1;
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
