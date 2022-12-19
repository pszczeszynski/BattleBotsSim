using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RollerDetect : MonoBehaviour
{
    public Transform mycilinder;
    public float myrot;
    public bool isred = false;
    public bool isblue = false;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        myrot = MyUtils.AngleWrap(mycilinder.rotation.eulerAngles.x);

        if( myrot <= -21f)
        {
            isblue = true;
        }
        else
        {
            isblue = false;
        }

        if (myrot >= 21f)
        {
            isred = true;
        }
        else
        {
            isred = false;
        }
    }
}
