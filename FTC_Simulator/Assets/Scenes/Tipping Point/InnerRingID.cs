using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// Used to identify the inner ring on pole
public class InnerRingID : MonoBehaviour
{

    public bool inner_ring = true;
    public int id = -1;

    private void Start()
    {
        gameElement ge_in_parent = transform.GetComponentInParent<gameElement>();

        if (!ge_in_parent) { return; }
        id = ge_in_parent.id;
    }
}

