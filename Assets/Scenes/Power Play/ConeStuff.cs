using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConeStuff : MonoBehaviour
{
    public bool isBeacon = false;
    public bool isRed = true;
    public RobotCollision myRobotCollision = null;
    public float myangle = 0f;

    public void Start()
    {
        // Find the game element
        myRobotCollision = transform.GetComponentInParent<RobotCollision>();
    }

    // Returns true if the cone is within angle_tolerance of the supplied up vector
    public bool isConeUpright(Vector3 junction_up, float tolerance = 45f )
    {
        Vector3 myup = transform.up;
        myangle = Math.Abs(Vector3.Angle(junction_up, myup));

        if (myangle <= tolerance)
        {
            return true;
        }

        return false;
    }

  


}
