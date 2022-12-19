using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum ElementType
{
    Cube=1, // Light cube
    CubeDark, // Dark varient
    Jewel,
    Stone,
    Red1, // Generic red1 item
    Blue1, // Generic blue1 item
    Red2,  // Generic red2 item
    Blue2, // Generic blue2 item
    NoRigidBody,// No rogid body
    Off // Do not sync/ignore
};

public class gameElement : MonoBehaviour {

    public int id;
    public ElementType type;
    public int lastupdateID;    // stores the message ID last used to update the element
    public string lastPlayerCollision; // stores name of player that last touched this directly
    public float maxAngularRotation = 0f;
    public Vector3 inertiaTensor = Vector3.zero;
    public Vector3 center_of_mass = Vector3.zero;
    public string note = "";
    public string note2 = "";
    public GenericFieldTracker tracker;

    public Vector3 starting_pos;
    private Quaternion starting_rot;
    private Color starting_color;
    private bool starting_kinematic;
    private bool I_have_color = false;
    public int held_by_robot = 0;
    public bool use_local_pos = false;

    public Transform option2;
    public Transform option3;
    public Transform option4;
    public Transform option5;

    public Rigidbody myrb;

    private void Update()
    {
        if( !myrb ) { return; }
    }

    private bool init_done = false;
    private void OnEnable()
    {
        // On Enable can be called whenevre this object is re-enabled, this really is about first time initialization, thus
        // check if we already have been initialized
        if( init_done ) { return;  }
        init_done = true;

        // initialize lastupdateID
        lastupdateID = 0;
        lastPlayerCollision = "";

        if (use_local_pos)
        {
            starting_pos = transform.localPosition;
            starting_rot = transform.localRotation;
        }
        else
        {
            starting_pos = transform.position;
            starting_rot = transform.rotation;
        }

        if( type == ElementType.NoRigidBody) { return; }

        if (GetComponent<Rigidbody>())
        {
            starting_kinematic = GetComponent<Rigidbody>().isKinematic;

            // Set max angular rotation if set
            if (maxAngularRotation != 0)
            {
                GetComponent<Rigidbody>().maxAngularVelocity = maxAngularRotation;
            }

            if( inertiaTensor.magnitude != 0 )
            {
                GetComponent<Rigidbody>().inertiaTensor = inertiaTensor;
            }

            if (center_of_mass.magnitude != 0)
            {
                GetComponent<Rigidbody>().centerOfMass = center_of_mass;
            }
        }

        Renderer myrenderer = GetComponent<Renderer>();
        if (myrenderer)
        {
            Material mymaterial = myrenderer.material;

            if( mymaterial)
            {
                starting_color = mymaterial.color;
                I_have_color = true;
            }
        }

        myrb = GetComponent<Rigidbody>();
    }

  

    private void OnCollisionEnter(Collision collision)
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        if ((type == ElementType.NoRigidBody) || (type == ElementType.Off)) { return; }

        // Find the top parent
        Transform topparent = collision.transform;

        while (topparent.parent != null)
        { topparent = topparent.parent; }

        // Store the name of the last person who collided with this block
        Transform Nametag = topparent.Find("Nametag");

        if( Nametag != null )
        {
            lastPlayerCollision = Nametag.GetComponent<TextMesh>().text;
        }
    }

    public void ResetColor()
    {
        if ((type == ElementType.NoRigidBody) || (type == ElementType.Off)) { return; }

        if (I_have_color)
        {
            GetComponent<Renderer>().material.color = starting_color;
        }
    }

    public virtual void ResetPosition(int option = 1)
    {
        if (type == ElementType.NoRigidBody) { return; }

        Transform final_value = transform;
        if (use_local_pos)
        {
            final_value.localPosition = starting_pos;
            final_value.localRotation = starting_rot;
        }
        else
        {
            final_value.position = starting_pos;
            final_value.rotation = starting_rot;
        }
        held_by_robot = 0;

        switch (option)
        {
            case 2:
                if (option2)
                {
                    final_value = option2;
                };
                break;
            case 3:
                if (option3) { final_value = option3; };
                break;
            case 4:
                if (option4) { final_value = option4; };
                break;
            case 5:
                if (option5) { 
                    final_value = option5; 
                };

                break;
        }

        transform.position = final_value.position;
        transform.rotation = final_value.rotation;

        if (GetComponent<Rigidbody>())
        {
            GetComponent<Rigidbody>().isKinematic = starting_kinematic;
        

            // Reset velocities
            Rigidbody mybody = GetComponent<Rigidbody>();

            mybody.velocity = Vector3.zero;
            mybody.angularVelocity = Vector3.zero;
            mybody.Sleep();

            ResetColor();
        }
    }
}

