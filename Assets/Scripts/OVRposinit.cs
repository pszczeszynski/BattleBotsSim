using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OVRposinit : MonoBehaviour
{
    // Start is called before the first frame update
    bool init_done = false;

    void Start()
    {
        init_done = false;
    }

    // Update is called once per frame
    void Update()
    {
        while( !init_done)
        {
            GameObject[] field_structures = GameObject.FindGameObjectsWithTag("FieldStructure");
            if( field_structures.Length < 1) { return; }

            // Get 3D Field
            foreach( GameObject currobj in field_structures)
            {
                if( currobj.name == "3d field")
                {
                    init_done = true;

                    Transform floor = currobj.transform.Find("floor");
                    if( floor == null)
                    {
                        floor = currobj.transform.Find("matts");
                    }
                    if (floor == null)
                    {
                        floor = currobj.transform.Find("matt");
                    }


                    // If for some reason we couldn't find it, abonden the process
                    if (floor == null)
                    {
                        return;
                    }

                    Vector3 mypos = transform.position;
                    mypos.y += floor.transform.position.y;
                    transform.position = mypos;
                    return;
                }
            }
        }
    }
}
