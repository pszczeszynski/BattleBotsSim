using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ball_data : MonoBehaviour
{
    public bool scored_red = false;
    public bool scored_blue = false;

    public int thrown_by_id = -1;
    public RobotID thrown_robotid = null;
    public Dictionary<string, string> flags = new Dictionary<string, string>();

    public void Clear()
    {
        thrown_by_id = -1;
        thrown_robotid = null;
        scored_red = false;
        scored_blue = false;
        flags.Clear();
    }

    // Returns true if flag is populated
    public bool IsFlagSet(string myflag)
    {
        return flags.ContainsKey(myflag) && (flags[myflag][0] == '1');
    }

    public void SetFlag(string myflag, bool value = true)
    {
        flags[myflag] = (value) ? "1" : "0";
    }

    public void ClearFlag(string myflag)
    {
        if( flags.ContainsKey(myflag))
        {
            flags.Remove(myflag);
        }
    }

}
