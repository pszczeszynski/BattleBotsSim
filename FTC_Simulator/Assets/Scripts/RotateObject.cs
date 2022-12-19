using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateObject : MonoBehaviour
{
    public bool run = false;
    public new string tag = "";
    public float speed = 1f;
    public float offspeed = 0f;
    public Transform centerpoint; // If not specified, will rotate around our center
    public Vector3 axis = new Vector3(1f, 0, 0);

    private float previous_time = 0;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // If we are turned off, then don't do anything.
        float curr_time = Time.time;

        // Rotate around cetnerpoit
        Vector3 center_of_rotation = transform.position;
        if( centerpoint != null )
        {
            center_of_rotation = centerpoint.position;
        }

        float currspeed = (run) ? speed : offspeed;
        if (speed != 0f)
        {
            transform.RotateAround(center_of_rotation, transform.TransformDirection(axis), currspeed * (curr_time - previous_time));
        }

        previous_time = curr_time;
    }
}
