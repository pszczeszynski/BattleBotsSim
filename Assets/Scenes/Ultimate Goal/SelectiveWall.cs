using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectiveWall : GenericFieldTracker
{
    public bool disable = false;

    public bool push_elements = true;
    public ElementType[] elements_to_let_through; // Pushes out all other objects
    public bool push_red_bots = true;
    public bool push_blue_bots = true;
    public Vector3 pushback_dir = new Vector3(1f, 0, 0);


    /*  private void Start()
       {

       }
   */


    /*  public void Reset()
       {

       }
    */

    /*   private void Update()
       {

       }
   */

    private void FixedUpdate()
    {
        // Push valid game objects
        if (!disable) { PushObjects(); }

    }
    private void PushObjects()
    {
        if (GLOBALS.CLIENT_MODE) { return; }

        // Push robots
        foreach (RobotID enemy_id in robots)
        {
            // Sanity check in case it was destroyed
            if (enemy_id == null) { continue; }

            // Dont push red/blue if so specified
            if( (enemy_id.is_red && !push_red_bots) || (!enemy_id.is_red && !push_blue_bots) ) { continue; }

            RobotInterface3D enemy_ri3d = enemy_id.GetComponent<RobotInterface3D>();
            if(enemy_ri3d == null) { continue; }
            Rigidbody enemy_rb = enemy_ri3d.rb_body;
            if( enemy_rb == null ) { continue; }
            
            Vector3 old_velocity = enemy_rb.velocity;
            Vector3 old_rotation = enemy_rb.angularVelocity;

            // Set velocities to 0
            enemy_rb.velocity = Vector3.zero;
            enemy_rb.angularVelocity = Vector3.zero;

            //enemy_rb.velocity = old_velocity*-1f;
            //enemy_rb.angularVelocity = old_rotation * -1f;

            // And finally we add force in opposite direction to use
            //enemy_rb.AddForce((enemy_rb.transform.position - transform.position).normalized * 200f, ForceMode.VelocityChange);
            enemy_rb.AddForce(pushback_dir *  500f , ForceMode.Acceleration);
            //enemy_rb.AddTorque(enemy_rb.angularVelocity * -1f, ForceMode.Acceleration);         
        }

        // Push gameobjects
        if(!push_elements) { return; }
        foreach (gameElement gameobj in game_elements)
        {
            // Sanity check in case it was destroyed
            if (gameobj == null) { continue; }

            // Check if it is an element to be ignored
            bool itsok = false;
            foreach( ElementType ok_element in elements_to_let_through)
            {
                if( gameobj.type == ok_element) { 
                    itsok=true;
                    break;
                }
            }
            if(itsok) { continue; }

            Rigidbody gameelement_rb = gameobj.GetComponent<Rigidbody>();
            if (gameelement_rb == null) { continue; }

            gameelement_rb.velocity = Vector3.zero;
            gameelement_rb.angularVelocity = Vector3.zero;

            // And finally we add force in opposite direction to use
            gameelement_rb.AddForce(pushback_dir * 500f, ForceMode.Acceleration);
        }

    }

}
