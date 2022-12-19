using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WallBallReturn : MonoBehaviour
{
    // Dictionary of Rigidbody vs time entored
    Dictionary<Rigidbody, float> all_bodies = new Dictionary<Rigidbody,float>();
    public float time_to_hold = 3f;
    public float velocity = 0.1f;

    private void OnTriggerEnter(Collider other)
    {      
        Rigidbody newbody = other.attachedRigidbody;

        if( newbody == null) { return; }

        // Add it to our list
        if ( !all_bodies.ContainsKey(other.attachedRigidbody) )
        {
            // Make sure this is a ball
            if( other.GetComponentInParent<ball_data>() == null )
            {
                return;
            }

            all_bodies.Add(newbody, Time.time);

            // Make it's velocity 0
            newbody.velocity = Vector3.zero;
        }


    }

    private void OnTriggerExit(Collider other)
    {
        Rigidbody newbody = other.attachedRigidbody;

        if (newbody == null) { return; }

        // Remove it from our list
        if (all_bodies.ContainsKey(other.attachedRigidbody))
        {
            all_bodies.Remove(newbody);
        }
    }

    private void FixedUpdate()
    {
        // Go through all the rigid bodies and push them back
        foreach (KeyValuePair<Rigidbody, float> entry in all_bodies)
        {
            // Make sure it's met it's holding period
            if( (Time.time - entry.Value) < time_to_hold) {
                entry.Key.velocity = Vector3.zero;
                continue; 
            }

            // Make it move towards center
            Vector3 tocenter = (new Vector3(0,1f,0) - entry.Key.transform.position).normalized;
            entry.Key.velocity = tocenter * velocity;
        }
    }
}
