using System.Collections;
using System.Collections.Generic;
using UnityEngine;
// Generic class to define functions for BandwidthHelper to send extra data
// to the object
public class BHelperData : MonoBehaviour
{
    virtual public string GetString()
    {
        return "";
    }

    virtual public void SetString( string indata)
    {

    }
}
