using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ballshooting : MonoBehaviour
{

    public bool play_sound = false; // Play shooting sound when game elements makes it through
    public float force_divider = 20f;   // How rapidly to increase velocity to target
    public bool hard_stop = false;      
    public bool off_when_zero = false; // If speed is 0, do nothing to game element
    public bool only_additive_force = false; // If only additive, it will not slow down speed, only add to it
    public bool zero_other_velocities = false; // If set, reduces other velocities to 0 (conforms motion to exact vector)
    public bool set_velocity_directly = false; // If set, sets velocity directly rather than applying force/torque. Risky for side effects


    public List<gameElement> objects_inside = new List<gameElement>(); // List of objects inside this box
    public bool disablePushing = false; // Turns off all forces, only used to detec objects and potentially mark them
    public bool doNotMarkBall = false; // When set, doesn't marke game element with this user
    public bool AddTorque = false; // Torque is added on the x-axis clockwise
    public bool UseDeprecatedAlgorithm = true; // Set here so as not to break old models. By default true, uses original algorithms which have side effects
    
    
    private Rigidbody myRigidBody;
    private RobotInterface3D myrobot;
    private Vector3 old_position = Vector3.zero;
    private Vector3 myVelocity = Vector3.zero;

    private Dictionary<GameObject, gameElement> dict_ge = new Dictionary<GameObject, gameElement>();
    private Dictionary<GameObject, Rigidbody> dict_rb = new Dictionary<GameObject, Rigidbody>();


    // Start is called before the first frame update
    void Start()
    {
        // Get my robot interface 3D
        myrobot = GetComponentInParent<RobotInterface3D>();
        old_position = transform.position;
        myRigidBody = GetComponent<Rigidbody>();

        MyStart();
    }

    public virtual void MyStart()
    {
        // Do additional initializations if required by derived functions
    }


    private bool playsound = false;
    private Collider item_triggering = null;

    // Update is called once per frame
    void Update()
    {
        if (playsound)
        {
            AudioManager themanager = item_triggering.gameObject.GetComponentInParent<AudioManager>();
            if( themanager )
            {
                themanager.Play("ballpass", 0);
            }
            playsound = false;
        }
    }



    // Calculate velocity in fixed update
    
    private void FixedUpdate()
    {
        if (AddTorque)
        {
            if(myRigidBody)
            {
                myVelocity = myRigidBody.angularVelocity;
            }
            else
            {
                myVelocity = Vector3.zero;
            }
        }
        else
        {
            myVelocity = (transform.position - old_position) / Time.fixedDeltaTime;
            old_position = transform.position;
        }
    }

    // Add force to object to give them fixed speed
    public float speed = 1f;


    void OnTriggerStay(Collider collision)
    {
        GameObject OnTrigger_object = collision.gameObject;

        // If not in our dictionary, exit
        if( !dict_ge.ContainsKey(OnTrigger_object)) { return; }

        gameElement objects_ge = dict_ge[OnTrigger_object];

        // This should never happen, but just in case...
        if (!objects_inside.Contains(objects_ge))
        {
            objects_inside.Add(objects_ge);
        }

 
        // If we aren't pushing the ball, we are done
        if (disablePushing) { return; }
        Rigidbody rigidbody = dict_rb[OnTrigger_object];


        if (AddTorque) {
            if (UseDeprecatedAlgorithm)
            {
                DoTorqueDeprecated(rigidbody);
            }
            else
            {
                DoTorque(rigidbody);
            }

        }
        else {
            if (UseDeprecatedAlgorithm)
            {
                DoVelocityDeprecated(rigidbody);
            }
            else
            {
                DoVelocity(rigidbody);
            }
        }

    }

    virtual public void MarkGameElement(ball_data thisballdata)
    {
        // Allow derived classes to do additional things if the item is marked
    }

    private void DoVelocity(Rigidbody rigidbody)
    {
        // We only care about scaling the velocity beyond our own
        // Exract the differenial velocity for use later
        Vector3 curr_velocity = rigidbody.velocity;
        Vector3 diff_velocity = curr_velocity - myVelocity;
        Vector3 velocitydir = this.transform.TransformDirection(-1f, 0, 0);
        Vector3 velocity_other_2 = this.transform.TransformDirection(0, -1f, 0);
        Vector3 velocity_other_3 = this.transform.TransformDirection(0, 0, -1f);

        // Get the velocity that is different from our own. This is the velocity only in our controlled direction       
        float velocity_mag = Vector3.Dot(diff_velocity, velocitydir);
        float velocity_mag2 = Vector3.Dot(diff_velocity, velocity_other_2);
        float velocity_mag3 = Vector3.Dot(diff_velocity, velocity_other_3);
        float velocity_delta = speed - velocity_mag;



        // If we are in a hard-stop position, then stop the object and push it slightly out
        if (hard_stop)
        {
            if (!set_velocity_directly)
            {
                // Set the velocity to slightly opposite to our direction
                rigidbody.AddForce(velocitydir * (velocity_delta - 0.01f * speed), ForceMode.VelocityChange);

                // Zero out other velocities
                if (zero_other_velocities)
                {
                    rigidbody.AddForce(-1f * velocity_mag2 * velocity_other_2, ForceMode.VelocityChange);
                    rigidbody.AddForce(-1f * velocity_mag3 * velocity_other_3, ForceMode.VelocityChange);
                }
                return;
            }
            else
            {
                // My velocity should be our velocity + a tiny bit of opposite direction
                Vector3 velocityout = myVelocity - 0.01f * speed * velocitydir;

                if( !zero_other_velocities) // If we aren't zeroing out other velocities, then add the orthogoanal velocities back in
                {
                    // Add in the other vleocities unchaned
                    velocityout += velocity_mag2 * velocity_other_2;
                    velocityout += velocity_mag3 * velocity_other_3;
                }

                // Now set our velocity
                rigidbody.velocity = velocityout;

                return;
            }
        }

        // If speed is 0, then do nothing more
        if ((speed == 0) && off_when_zero)
        {
            return;
        }


        if (!set_velocity_directly)
        {
            // Zero out other velocities if applicable
            if (zero_other_velocities)
            {
                rigidbody.AddForce(-1f * velocity_mag2 * velocity_other_2 / force_divider, ForceMode.VelocityChange);
                rigidbody.AddForce(-1f * velocity_mag3 * velocity_other_3 / force_divider, ForceMode.VelocityChange);
            }

            // Only add torque if we are allowing full speed control
            if (only_additive_force && (velocity_delta < 0f)) { return; }
            rigidbody.AddForce(velocitydir * velocity_delta / force_divider, ForceMode.VelocityChange);
        }
        else
        {
            // My velocity should be our velocity + 
            Vector3 velocityout = myVelocity + (speed + (speed - velocity_mag)/force_divider) * velocitydir;

            if (!zero_other_velocities) // If we aren't zeroing out other velocities, then add the orthogoanal velocities back in
            {
                // Add in the other vleocities unchaned
                velocityout += velocity_mag2 * velocity_other_2;
                velocityout += velocity_mag3 * velocity_other_3;
            }
            else
            {
                velocityout += velocity_mag2 * velocity_other_2 * (1 - 1 / force_divider);
                velocityout += velocity_mag3 * velocity_other_3 * (1 - 1 / force_divider);
            }

            // Now set our velocity
            rigidbody.velocity = velocityout;
        }
    }

    private void DoVelocityDeprecated( Rigidbody rigidbody)
    { 
        // If we are in a hard-stop position, then stop the object and push it slightly out
        if (hard_stop)
        {
            // Push object out slightly
            rigidbody.velocity = myVelocity;
            rigidbody.AddForce(this.transform.TransformDirection(500f*speed, 0, 0), ForceMode.Acceleration);
            return;
        }

        // If speed is 0, then do nothing more
        if ( (speed == 0) && off_when_zero)
        {
            return;
        }

        // Otherwise move the object forward
        // **** Don't think this is needed anymore?????
        rigidbody.AddForce(this.transform.TransformDirection(-1f, 0, 0) * speed / force_divider, ForceMode.VelocityChange);

        if(only_additive_force) { return; }
        // ****

        // We only care about scaling the velocity beyond our own
        Vector3 curr_velocity = rigidbody.velocity;
        Vector3 diff_velocity = curr_velocity - myVelocity;

        // Normalize the velocity
        float velocity_mag = diff_velocity.magnitude;
        if(velocity_mag < 0.000001f) { velocity_mag = 0.000001f; }

        float scaler = speed / velocity_mag; // Calculate how far away we are from target velocity
        diff_velocity *= 1f + (scaler - 1f) / force_divider; // move it closer to target

        // Add our velocity back in and set it
        curr_velocity = diff_velocity + myVelocity;
        rigidbody.velocity = curr_velocity;
    }


    private void DoTorque(Rigidbody rigidbody)
    {
        // We only care about scaling the velocity beyond our own
        // Exract the differenial velocity for use later
        Vector3 curr_velocity = rigidbody.angularVelocity;
        Vector3 diff_velocity = curr_velocity - myVelocity;
        Vector3 torquedir = this.transform.TransformDirection(-1f, 0, 0);
        Vector3 velocity_other_2 = this.transform.TransformDirection(0, -1f, 0);
        Vector3 velocity_other_3 = this.transform.TransformDirection(0, 0, -1f);

        // Get the angular velocity that is different from our own. This is the angular velocity only in our controlled direction       
        float velocity_mag = Vector3.Dot(diff_velocity, torquedir);
        float velocity_mag2 = Vector3.Dot(diff_velocity, velocity_other_2);
        float velocity_mag3 = Vector3.Dot(diff_velocity, velocity_other_3);
        float velocity_delta = speed - velocity_mag;

        // If we are in a hard-stop position, then stop the object and push it slightly out
        if (hard_stop)
        {
            // Set the angular rotation to slightly opposite to our direction
            rigidbody.AddTorque(torquedir * (velocity_delta-0.01f*speed) , ForceMode.VelocityChange);

            // Zero out other velocities
            if (zero_other_velocities)
            {
                rigidbody.AddTorque(-1f * velocity_mag2 * velocity_other_2, ForceMode.VelocityChange);
                rigidbody.AddTorque(-1f * velocity_mag3 * velocity_other_3, ForceMode.VelocityChange);
            }
            return;
        }

        // If speed is 0, then do nothing more
        if ((speed == 0) && off_when_zero)
        {
            return;
        }

        // Zero out other velocities if applicable
        if (zero_other_velocities)
        {
            rigidbody.AddTorque(-1f * velocity_mag2 * velocity_other_2 / force_divider, ForceMode.VelocityChange);
            rigidbody.AddTorque(-1f * velocity_mag3 * velocity_other_3 / force_divider, ForceMode.VelocityChange);
        }

        // Only add torque if we are allowing full speed control
        if ( only_additive_force && (velocity_delta < 0f)) { return; }
        rigidbody.AddTorque(torquedir * velocity_delta / force_divider, ForceMode.VelocityChange);
    }


    private void DoTorqueDeprecated(Rigidbody rigidbody)
    {
        // If we are in a hard-stop position, then stop the object and push it slightly out
        if (hard_stop)
        {
            // Push object out slightly
            rigidbody.angularVelocity = myVelocity;
            rigidbody.AddTorque(this.transform.TransformDirection(500f * speed, 0, 0), ForceMode.Acceleration);
            return;
        }

        // If speed is 0, then do nothing more
        if ((speed == 0) && off_when_zero)
        {
            return;
        }

        // Otherwise move the object forward
        // **** Don't think this is needed anymore?????
        rigidbody.AddTorque(this.transform.TransformDirection(-1f, 0, 0) * speed / force_divider, ForceMode.VelocityChange);

        if (only_additive_force) { return; }
        // ****

        // We only care about scaling the velocity beyond our own
        Vector3 curr_velocity = rigidbody.angularVelocity;
        Vector3 diff_velocity = curr_velocity - myVelocity;

        // Normalize the velocity
        float velocity_mag = diff_velocity.magnitude;
        if (velocity_mag < 0.000001f) { velocity_mag = 0.000001f; }

        float scaler = speed / velocity_mag; // Calculate how far away we are from target velocity
        diff_velocity *= 1f + (scaler - 1f) / force_divider; // move it closer to target

        // Add our velocity back in and set it
        curr_velocity = diff_velocity + myVelocity;
        rigidbody.angularVelocity = curr_velocity;
    }

    private void OnTriggerEnter(Collider collision)
    {
        // Make sure this is a game element and is active
        GameObject my_go = collision.gameObject;
        if (!my_go || !my_go.activeInHierarchy ) { return; }

        gameElement my_ge = my_go.GetComponent<gameElement>();

        if( !my_ge)
        {
            my_ge = my_go.GetComponentInParent<gameElement>();

            if( !my_ge ) { return; }
        }

        // We only want to deal with balls, but more specifically, avoid activating the balance beam
        if ((my_ge.type == ElementType.Stone) || (my_ge.type == ElementType.Off) || (my_ge.type == ElementType.NoRigidBody))
        { return; }

        // Get rigid body. gameElements is supposed to be added at the same level as the rigidbody
        Rigidbody my_rb = my_ge.GetComponent<Rigidbody>();

        if( !my_rb) { return; }

        // We got all the info now populate it in our dictionary
        CleanUpList(); // First clean up the dictionaries and lists just in case
        dict_ge[my_go] = my_ge;
        dict_rb[my_go] = my_rb;

        // Record current active ge objects
        if (!objects_inside.Contains(my_ge))
        {
            objects_inside.Add(my_ge);
        }

        // Mark the ball if required
        if (!doNotMarkBall)
        {
            // Get the ball data
            ball_data thisballsdata = my_ge.GetComponent<ball_data>();

            if (thisballsdata)
            {
                if (myrobot)
                {
                    thisballsdata.thrown_by_id = myrobot.myRobotID.id;
                    thisballsdata.thrown_robotid = myrobot.myRobotID;
                    MarkGameElement(thisballsdata);
                }
                else
                {
                    thisballsdata.thrown_by_id = -1;
                    thisballsdata.thrown_robotid = null;
                }
            }
        }


        // Play sound if user selected it
        if (play_sound)
        {
            playsound = true;
            item_triggering = collision;
        }

    }

    private void OnTriggerExit(Collider collision)
    {
        // Make sure this is a game element     
        GameObject my_go = collision.gameObject;
        if (!my_go) { return; }

        // If not in our dictionary, leave
        if( !dict_ge.ContainsKey(my_go)) { return; }

        // Romve from our list
        objects_inside.Remove(dict_ge[my_go]);

        // Remove from our dictionary
        dict_ge.Remove(my_go);
        dict_rb.Remove(my_go);
    }

    private void CleanUpList()
    {
        // Clean up our objects list
        for( int i = objects_inside.Count-1; i >= 0; i--)
        {
            if (!objects_inside[i]) { objects_inside.RemoveAt(i); }
            else if(! objects_inside[i].gameObject.activeSelf) { objects_inside.RemoveAt(i); }
        }

        // Clean up our hash tables
        foreach( GameObject curr_obj in dict_ge.Keys)
        {
            if( !curr_obj || !curr_obj.activeInHierarchy )
            {
                dict_ge.Remove(curr_obj);
            }
        }

        foreach (GameObject curr_obj in dict_rb.Keys)
        {
            if (!curr_obj || !curr_obj.activeInHierarchy)
            {
                dict_rb.Remove(curr_obj);
            }
        }
    }

    public gameElement GetBallInside()
    {
        // First cleanup list
        CleanUpList();

        // Next return an available ball
        if( objects_inside.Count > 0)
        {
            gameElement return_obj = objects_inside[0];
            objects_inside.RemoveAt(0);
            return return_obj;
        }

        return null;
    }

    public bool AnyBallsInside()
    {
        return (objects_inside.Count > 0);
    }
}
