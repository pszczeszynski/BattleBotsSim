using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TurnOffGravity : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnTriggerEnter(Collider collision)
    {
        // Get Parent

        RobotInterface3D ri3d = collision.transform.GetComponentInParent<RobotInterface3D>();

        if( !ri3d) { return; }


        Rigidbody[] allbodies = ri3d.GetComponentsInChildren<Rigidbody>();

        foreach(Rigidbody rb in allbodies )
        {
            rb.useGravity = false;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

            if (rb.transform.name.Contains("Wheel") || rb.transform.name.Contains("wheel"))
            {
                rb.constraints = RigidbodyConstraints.FreezeAll;
            }
        }

    }
}
