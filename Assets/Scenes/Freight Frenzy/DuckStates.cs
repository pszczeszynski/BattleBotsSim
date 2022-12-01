using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DuckStates : MonoBehaviour
{
    public enum DuckPositions
    {
        OffField = 0,
        Placed,
        Touched,
        Scored,
        NonCarousel
    };

    public bool red = false;
    public bool blue = false;

    public DuckPositions mystate = DuckPositions.NonCarousel;


}
