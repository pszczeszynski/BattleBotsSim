using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class RobotID : MonoBehaviour {

    // list of elements colliding
    public Transform wheelTL;
    public Transform wheelTR;
    public Transform wheelBL;
    public Transform wheelBR;
    public Vector3 position;
    public string starting_pos;
    public int id;
    public bool is_red = false;
    public bool is_holding = false;
    public bool is_counting = false;    // true if referee turned on countdown timer
    public float count_start = 0f;      // Start of the countdown timer
    public float count_duration = 5f;   // Total duration of count down timer in seconds. Used to set percentage

    // Generic user Data
    public Dictionary<string, string> userData = new Dictionary<string, string>();

    // Copy over data from one RobotID to this one
    public void Copy(RobotID_Data refin)
    {
        starting_pos = refin.starting_pos;
        id = refin.id;
        is_red = refin.is_red;
        is_holding = refin.is_holding;
        foreach( string currkey in refin.userData.Keys)
        {
            userData[currkey] = refin.userData[currkey];
        }
    }

    public void Copy(RobotID refin)
    {
        starting_pos = refin.starting_pos;
        id = refin.id;
        is_red = refin.is_red;
        is_holding = refin.is_holding;
        foreach (string currkey in refin.userData.Keys)
        {
            userData[currkey] = refin.userData[currkey];
        }
    }

    private void Update()
    {
        // If any of the wheels are null, try to find them
        if (wheelTL == null || wheelTR == null || wheelBL == null || wheelBR == null)
        {
            wheelTL = FindInHierarchy("wheelTL");
            wheelTR = FindInHierarchy("wheelTR");
            wheelBL = FindInHierarchy("wheelBL");
            wheelBR = FindInHierarchy("wheelBR");
        }

        // Update the Robot position (Center of Robot)
        if (wheelTL==null || wheelTR == null || wheelBL==null || wheelBR==null)
        { return;  }

        position = (wheelTL.position + wheelTR.position + wheelBL.position + wheelBR.position) / 4f;
    }

    private Transform FindInHierarchy(string name)
    { 
        
        Transform[] children = gameObject.GetComponentsInChildren<Transform>();
        foreach (Transform child in children)
        {
            if (child.gameObject.name.CompareTo(name)==0)
            {
                return child;
            }
        }

        return null;
    }

    public void SetUserInt(string key, int value)
    {
        userData[key] = value.ToString();
    }

    public int GetUserInt(string key)
    {
        if( !userData.ContainsKey(key))
        {
            userData[key] = "0";
        }

        return int.Parse(userData[key]);
    }

    public void SetUserFloat(string key, float value)
    {
        userData[key] = value.ToString();
    }

    public float GetUserFloat(string key)
    {
        if (!userData.ContainsKey(key))
        {
            userData[key] = "0";
        }

        return float.Parse(userData[key]);
    }

    public bool GetUserBool(string key)
    {
        if (!userData.ContainsKey(key))
        {
            userData[key] = "0";
        }

        return (userData[key][0] == '1');
    }

    public void SetUserBool(string key, bool value = true)
    {
        userData[key] = ((value) ? "1" : "0");
    }

    public void RemoveData(string key)
    {
        if( userData.ContainsKey(key))
        {
            userData.Remove(key);
        }
    }
}

// Need a non-Monobehavior class as temporary storage class
public class RobotID_Data
{
    // list of elements colliding
    public Transform wheelTL;
    public Transform wheelTR;
    public Transform wheelBL;
    public Transform wheelBR;
    public Vector3 position;
    public string starting_pos;
    public int id;
    public bool is_red = false;
    public bool is_holding = false;

    // Generic user Data
    public Dictionary<string, string> userData = new Dictionary<string, string>();

    public void Copy(RobotID refin)
    {
        starting_pos = refin.starting_pos;
        id = refin.id;
        is_red = refin.is_red;
        is_holding = refin.is_holding;
        foreach (string currkey in refin.userData.Keys)
        {
            userData[currkey] = refin.userData[currkey];
        }
    }
}

