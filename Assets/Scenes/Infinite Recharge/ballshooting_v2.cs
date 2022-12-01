using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Ballshooting2
//
// Similar to original but steamlined and tuned for better performance
// The ball needs to have "ball_data" attached to it.


public class ballshooting_v2 : MonoBehaviour
{
    public bool disable = false;    // A way to disable script without turning off monobehavior, thus perserves collision detection
    public bool play_sound = false; // Play shooting sound when game elements makes it through
    public float force_divider = 20f;   // How rapidly to increase velocity to target
    public bool hard_stop = false;
    public bool reverse = false;
    public bool off_when_zero = false; // If speed is 0, do nothing to game element
    public bool zero_other_velocities = false; // If set, reduces other velocities to 0 (conforms motion to exact vector)
    public bool set_velocity_directly = false; // If set, sets velocity directly rather than applying force/torque. Risky for side effects
    public bool doNotMarkBall = false; // When set, doesn't marke game element with this user
    public float hard_stop_speed = 0.5f;
    public bool apply_counterforce = false;  // If try, applies coutner force to body 


    private RobotInterface3D myrobot;
    public Rigidbody myRb;
    private Vector3 old_position = Vector3.zero;
    private Vector3 myVelocity = Vector3.zero;
    private bool playsound = false;
    private Collider item_triggering = null;
    
    // Add force to object to give them fixed speed
    public float speed = 1f;

    private Dictionary<GameObject, Rigidbody> dict_rb = new Dictionary<GameObject, Rigidbody>();



    // Start is called before the first frame update
    void Start()
    {
        // Get my robot interface 3D
        myrobot = GetComponentInParent<RobotInterface3D>();
        old_position = transform.position;

        Rigidbody[] all_bodies = transform.GetComponentsInParent<Rigidbody>();
        foreach (Rigidbody currbody in all_bodies)
        {
            if(!currbody.isKinematic)
            {
                myRb = currbody;
                break;
            }
        }

        MyStart();
    }

    public virtual void MyStart()
    {
        // Do additional initializations if required by derived functions
    }


    // Update is called once per frame
    void Update()
    {
  
        if( disable ) { return; }
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
        myVelocity = (transform.position - old_position) / Time.fixedDeltaTime;
        old_position = transform.position;
    }

    public void Clear()
    {
        myVelocity = Vector3.zero;
        old_position = transform.position;
    }


    void OnTriggerStay(Collider collision)
    {
        if( disable ) { return; }

        DoVelocity(collision);
    }

    //public float enter_time = 0f;
    //public float exit_time = 0f;

    private void OnTriggerEnter(Collider collision)
    {
        // if (disable) { return; }

        // If not valid object, exit
        // must have ball-ball_data in it

        // Get ball data
        GameObject my_go = collision.gameObject;
        if( !my_go || !my_go.activeInHierarchy) { return; }

        ball_data myball = my_go.GetComponentInParent<ball_data>();
        if (!myball) { return; }

        Rigidbody rigidbody = myball.GetComponentInParent<Rigidbody>();
        if (!rigidbody) { return; }

        // Add it to our dictionary
        CleanUpList(); // Clean it up first just in case
        dict_rb[my_go] = rigidbody;
        //enter_time = Time.fixedTime;


        // Play sound if user selectedit
        if ( play_sound)
        {
            playsound = true;
            item_triggering = collision;
        }

        // Make the gameElement remember who we are
        if (!doNotMarkBall && myball)
        {
            if (myrobot)
            {
                myball.thrown_by_id = myrobot.myRobotID.id;
                myball.thrown_robotid = myrobot.myRobotID;
                MarkGameElement(myball);
            }
            else
            {
                myball.thrown_by_id = -1;
                myball.thrown_robotid = null;
            }
        }     
    }

    private void OnTriggerExit(Collider collision)
    {
        // Make sure this is a game element     
        GameObject my_go = collision.gameObject;
        if (!my_go) { return; }

        // If not in our dictionary, leave
        if (!dict_rb.ContainsKey(my_go)) { return; }
        dict_rb.Remove(my_go);

        //exit_time = Time.fixedTime;
    }


    virtual public void MarkGameElement(ball_data thisballdata)
    {
        // Allow derived classes to do additional things if the item is marked
    }

    private void DoVelocity(Collider collision)
    {
        // If speed is 0, then do nothing more
        if ((speed == 0) && off_when_zero)
        {
            return;
        }

        // Exit if not approved object
        if( !dict_rb.ContainsKey(collision.gameObject)) { return; }

        Rigidbody rigidbody = dict_rb[collision.gameObject];


        // We only care about scaling the velocity beyond our own
        // Extract the differenial velocity for use later
        Vector3 curr_velocity = rigidbody.velocity;
        if( myRb )
        {
            myVelocity = myRb.velocity;
        }
        Vector3 diff_velocity = curr_velocity - myVelocity;
        Vector3 velocitydir = this.transform.TransformDirection(-1f, 0, 0);
        Vector3 velocity_other_2 = this.transform.TransformDirection(0, -1f, 0);
        Vector3 velocity_other_3 = this.transform.TransformDirection(0, 0, -1f);

        // Get the velocity that is different from our own. This is the velocity only in our controlled direction       
        float velocity_mag = Vector3.Dot(diff_velocity, velocitydir);
        float velocity_mag2 = Vector3.Dot(diff_velocity, velocity_other_2);
        float velocity_mag3 = Vector3.Dot(diff_velocity, velocity_other_3);
        float velocity_delta = speed - velocity_mag;

        Debug.DrawLine(transform.position, transform.position + 2f * velocitydir);


        // If we are in a hard-stop position, then stop the object and push it slightly out
        if (hard_stop)
        {
            if (!set_velocity_directly)
            {
                // Set the velocity to slightly opposite to our direction
                rigidbody.AddForce(-1f * (velocitydir * velocity_mag + velocitydir*hard_stop_speed), ForceMode.VelocityChange);

                // Zero out other velocities
                if (zero_other_velocities)
                {
                    rigidbody.AddForce(-1f * velocity_mag2 * velocity_other_2, ForceMode.VelocityChange);
                    rigidbody.AddForce(-1f * velocity_mag3 * velocity_other_3, ForceMode.VelocityChange);
                }
            }
            else
            {
                // My velocity should be our velocity + a tiny bit of opposite direction
                Vector3 velocityout = myVelocity - hard_stop_speed * velocitydir;

                if( !zero_other_velocities) // If we aren't zeroing out other velocities, then add the orthogoanal velocities back in
                {
                    // Add in the other vleocities unchaned
                    velocityout += velocity_mag2 * velocity_other_2;
                    velocityout += velocity_mag3 * velocity_other_3;
                }

                // Now set our velocity
                rigidbody.velocity = velocityout;            
            }

            return;
        }

        float reverse_float = (reverse) ? -1f : 1f;

        if (!set_velocity_directly)
        {
            // Set the velocity to slightly opposite to our direction
            Vector3 forcetoadd = (velocitydir * velocity_delta) / force_divider * reverse_float;
            rigidbody.AddForce(forcetoadd, ForceMode.VelocityChange);

            float mass_scaler = rigidbody.mass / myrobot.rb_body.mass;

            if ( apply_counterforce)
            {                
                myrobot.rb_body.AddForce(-1f * forcetoadd * mass_scaler, ForceMode.VelocityChange);
            }

            // Zero out other velocities
            if (zero_other_velocities)
            {
                forcetoadd = -1f * velocity_mag2 * velocity_other_2 / force_divider;
                rigidbody.AddForce(forcetoadd, ForceMode.VelocityChange);
                if (apply_counterforce)
                {
                    myrobot.rb_body.AddForce(-1f * forcetoadd * mass_scaler, ForceMode.VelocityChange);
                }

                forcetoadd = -1f * velocity_mag3 * velocity_other_3 / force_divider;
                rigidbody.AddForce(forcetoadd, ForceMode.VelocityChange);
                if (apply_counterforce)
                {
                    myrobot.rb_body.AddForce(-1f * forcetoadd * mass_scaler, ForceMode.VelocityChange);
                }
            }
        }
        else
        {
            Vector3 velocityout = speed * velocitydir * reverse_float;

            if (!zero_other_velocities) // If we aren't zeroing out other velocities, then add the orthogoanal velocities back in
            {
                // Add in the other vleocities unchaned
                velocityout += velocity_mag2 * velocity_other_2;
                velocityout += velocity_mag3 * velocity_other_3;
            }

            // Now set our velocity
            rigidbody.velocity = curr_velocity * (force_divider-1f)/force_divider + velocityout/force_divider;
        }     

        return;
    }

    private void CleanUpList()
    {       
        foreach (GameObject curr_obj in dict_rb.Keys)
        {
            if (!curr_obj || !curr_obj.activeInHierarchy)
            {
                dict_rb.Remove(curr_obj);
            }
        }
    }
}
