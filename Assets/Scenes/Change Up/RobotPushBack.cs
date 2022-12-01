using System.Collections.Generic;
using UnityEngine;

public class RobotPushBack : MonoBehaviour
{
    GameSettings game_settings = null;
    public bool enable_pushback = false;
    BoxCollider mycollider = null;

    // list of elements colliding
    public List<Transform> colliding_pieces = new List<Transform>();
    public List<Rigidbody> colliding_rb = new List<Rigidbody>();
    private List<int> colliding_rb_collisions = new List<int>();
    public float forceValue = 2000f;

    private void OnEnable()
    {
        RemoveInvalidItems();

        // Re-initialize game settings when enabled
        Start();
    }

    // Start is called before the first frame update 
    void Start()
    {
        enable_pushback = false;
        mycollider = GetComponent<BoxCollider>();

        if (GameObject.Find("GameSettings"))
        {
            game_settings = GameObject.Find("GameSettings").GetComponent<GameSettings>();
            if(game_settings==null) { return; }

            var property = game_settings.GetType().GetField("ENABLE_AUTO_PUSHBACK");
            if( property==null) { return; }

            enable_pushback = (bool) property.GetValue((object) game_settings);
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        // Apply a pushing force to opposite players
        if (enable_pushback) { PushPlayers(); }

    }


    // Gets called on a field reset after all objects are re-initialized
    // No one uses this though
    // public void Reset()
    //{         
    //    // Clean up list
    //    RemoveInvalidItems();
    //
    //    // Reinitialize gamesettings
    //    Start();
    //}

    public float extra_power = 1f;
    private void PushPlayers()
    {
        if( GLOBALS.CLIENT_MODE) { return; }

        foreach (Rigidbody enemy_rb in colliding_rb)
        {
            if (enemy_rb == null) { return; }

            // Get the shortest distance to player body, and this is the line of force to apply
            Vector3 vector_to_enemy_raw = enemy_rb.transform.position - mycollider.ClosestPoint(enemy_rb.transform.position);
            Vector3 vector_to_enemy = vector_to_enemy_raw.normalized;

            // Zero out velocity in our direction if coming towards us

            Vector3 velocity = enemy_rb.velocity;

            // Zero out velocity along our forward direction
            float dotproduct = Vector3.Dot(velocity, vector_to_enemy);
            // velocity -= dotproduct * vector_to_enemy;

            if ( dotproduct < 0f)
            {
                velocity -= dotproduct * vector_to_enemy;
            }

            enemy_rb.velocity = velocity;

            // Finally apply pushing force
            float distance_to_body = vector_to_enemy_raw.magnitude;
            extra_power = (distance_to_body < 0.6f) ? 0.6f / (distance_to_body + 0.001f) : 1f;

            enemy_rb.AddForce(vector_to_enemy * forceValue * extra_power, ForceMode.Force);

            // OLD CODE:
            //enemy_rb.velocity = Vector3.zero;
            //enemy_rb.angularVelocity = Vector3.zero;


            // And finally we add force in opposite direction to use
            //enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized * 200f, ForceMode.Acceleration);

            // enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized * 30, ForceMode.Impulse);
        }
    }

    // Returns true is transform is part of the "Body" object
    private Transform FindBody(Transform start)
    {
        // Make sure this isn't the collisionBoundry block
        if( start.name.StartsWith("collisionBoundry")) { return null; }

        // Go up hierarchy until we find the body
        while(start )
        {
            if( start.name == "Body") { return start;  }

            start = start.parent;
        }

        return null;
    }

    private void RememberRobot(Collider collision)
    {
        // Get the body
        Transform body = FindBody(collision.transform);
        if ( body==null) { return; }

        // Make sure this is a robot and not something with a Body component
        RobotID robotElement = body.GetComponentInParent<RobotID>();

        // If this isn't an element of the proper type, then exit
        if (robotElement == null) { return; }

        // Find the associated RB
        Rigidbody found_rb = body.GetComponent<Rigidbody>();
        if (found_rb == null) { return; }

        // Remember the piece as well as the rigid body
        colliding_pieces.Add(collision.transform);
        int index = colliding_rb.IndexOf(found_rb);
        if( index < 0 )
        {
            colliding_rb.Add(found_rb);
            colliding_rb_collisions.Add(1);
        }
        else
        {
            colliding_rb_collisions[index] += 1;
        }

    }

    // Remove all robots from the list and/or remove its collision counter
    private bool RemoveRobot(Collider collision)
    {
        // See if it is in our list
        if (!colliding_pieces.Contains(collision.transform))
        { return false; }

        // Since this piece is in our list, remove it
        // (also this piece is fully qualified so we don't need to do additional checking)
        Rigidbody found_rb = collision.transform.GetComponentInParent<Rigidbody>();
        if (found_rb == null) { return false; }

        // Now remove stuff
        colliding_pieces.Remove(collision.transform);

        // Find and locate rigid body
        int index = colliding_rb.IndexOf(found_rb);
        if (index < 0)
        {
            return false;
        }

        colliding_rb_collisions[index] -= 1;

        if (colliding_rb_collisions[index] <= 0)
        {
            colliding_rb.RemoveAt(index);
            colliding_rb_collisions.RemoveAt(index);
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
        RememberRobot(collision);
    }

    void OnTriggerExit(Collider collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Clean up list
        RemoveInvalidItems();

        // Remove robots if applicable
        RemoveRobot(collision);

    }


    // Remove items without a transform
    void RemoveInvalidItems()
    {
        int item = colliding_pieces.Count - 1;
        while (item >= 0)
        {
            if (colliding_pieces[item] == null)
            {
                colliding_pieces.RemoveAt(item);
            }

            item -= 1;
        }

        item = colliding_rb.Count - 1;
        while (item >= 0)
        {
            if (colliding_rb[item] == null)
            {
                colliding_rb.RemoveAt(item);
                colliding_rb_collisions.RemoveAt(item);
            }

            item -= 1;
        }
    }

}
