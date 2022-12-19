using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotSkin : MonoBehaviour
{
    public RobotInterface3D ri3d;

    virtual public void InitSkin()
    {
        // Find RI3D in the component
        ri3d = GetComponent<RobotInterface3D>();

        if( ri3d == null )
        {
            ri3d = GetComponentInParent<RobotInterface3D>();
        }

        if( ri3d != null)
        {
            // Tie ourselves into the skin updates
            ri3d.robotskin = this;
        }
    }

    virtual public string GetState()
    {
        return "";
    }

    virtual public void SetState(string instate)
    {
        return;
    }
}
