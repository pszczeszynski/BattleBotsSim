using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugTransform : MonoBehaviour
{
    public Vector3 onenable_pos;
    public Quaternion onenable_rot;

    public Vector3 reset_pos;
    public Quaternion reset_rot;

    public Vector3 start_pos;
    public Quaternion start_rot;

    public Vector3 fix_pos;
    public Quaternion fix_rot;

    public Vector3 curr_pos;
    public Quaternion curr_rot;
    // Start is called before the first frame update

    private void OnEnable()
    {
        fix_pos = transform.position;
        fix_rot = transform.rotation;
    }

    private void Reset()
    {
        reset_pos = transform.position;
        reset_rot = transform.rotation;
    }

    public void FixVars()
    {
        onenable_pos = transform.position;
        onenable_rot = transform.rotation;
    }

    void Start()
    {
        start_pos = transform.position;
        start_rot = transform.rotation;
    }

    // Update is called once per frame
    private void Update()
    {
        curr_pos = transform.position;
        curr_rot = transform.rotation;

        if( curr_pos != start_pos)
        {
            bool fail = true;
        }
    }
}
