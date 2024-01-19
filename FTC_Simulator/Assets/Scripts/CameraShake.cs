using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraShake : MonoBehaviour
{
    public float x_shake = 0f;
    public float y_shake = 0f;
    public float z_shake = 0f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        float xoffset = 0f;
        float yoffset = 0f;
        float zoffset = 0f;

        if (x_shake > 0f)
        {
            xoffset = Random.Range(-x_shake, x_shake);
        }

        if (y_shake > 0f)
        {
            yoffset = Random.Range(-y_shake, y_shake);
        }

        if (z_shake > 0f)
        {
            zoffset = Random.Range(-z_shake, z_shake);
        }

        Quaternion currot = transform.rotation;
        currot.eulerAngles = new Vector3(xoffset, yoffset, zoffset);
        transform.rotation = currot;

    }
}
