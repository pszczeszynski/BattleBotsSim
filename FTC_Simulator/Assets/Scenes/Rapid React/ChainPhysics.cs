using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChainPhysics : MonoBehaviour
{
    public float dampening = 0.9f;

    // Create a dictionary of gameobjects and their gameelements and rigidbodies to speed up operation
    private Dictionary<GameObject, Rigidbody> dic_rb = new Dictionary<GameObject, Rigidbody>();


    private void OnTriggerEnter(Collider collision)
    {
        // Add it to our dictionary
        GameObject OnTrigger_object = collision.gameObject;

        // Make sure this is a game element
        gameElement objects_ge = OnTrigger_object.GetComponent<gameElement>();
        if (!objects_ge) { objects_ge = OnTrigger_object.GetComponentInParent<gameElement>(); }
        if (!objects_ge) { return; }

        // We only want to deal with balls, but more specifically, avoid activating the balance beam
        if ((objects_ge.type == ElementType.Stone) || (objects_ge.type == ElementType.Off) || (objects_ge.type == ElementType.NoRigidBody))
        { return; }

        Rigidbody rigidbody = objects_ge.gameObject.GetComponent<Rigidbody>();
        if (rigidbody == null) { return; }

        // Now that we found our rigidbody, add it to our dictionary
        dic_rb.Add(OnTrigger_object, rigidbody);
    }

    void OnTriggerStay(Collider collision)
    {
        // Get out object
        if( !dic_rb.ContainsKey(collision.gameObject)) { return; }

        Rigidbody rigidbody = dic_rb[collision.gameObject];

        // Now kill the non vertical velocity
        Vector3 curr_velocity = rigidbody.velocity;

        // Reduce x,z velocity
        curr_velocity.x *= dampening;
        curr_velocity.z *= dampening;

        // If y is pointing up, reduce it as well
        if( curr_velocity.y > 0f )
        {
            curr_velocity.y *= dampening;
        }

        rigidbody.velocity = curr_velocity;
    }

    private void OnTriggerExit(Collider collision)
    {
        // Remove it from out dictionary
        if (dic_rb.ContainsKey(collision.gameObject)) 
        {
            dic_rb.Remove(collision.gameObject);
            return; 
        }

        return;
    }
   
}
