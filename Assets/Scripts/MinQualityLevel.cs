using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MinQualityLevel : MonoBehaviour
{
    // System quality must >= this setting to enable component
    public int quality = 2;
    public int maxquality = -1; // If set >= 1, it wil lbe disable if quality is above this number

}
