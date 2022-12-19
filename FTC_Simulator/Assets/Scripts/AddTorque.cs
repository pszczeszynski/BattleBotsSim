using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddTorque : MonoBehaviour
{
    public Rigidbody rbtorotate;
    public float x_torque = 0f;
    public float y_torque = 0f;
    public float z_torque = 0f;
    public ForceMode forcetype = ForceMode.Acceleration;
    public bool enable = false;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerStay(Collider other)
    {
        if( !enable ) { return; }

        rbtorotate.AddRelativeTorque(x_torque, y_torque, z_torque, forcetype);
    }
}
