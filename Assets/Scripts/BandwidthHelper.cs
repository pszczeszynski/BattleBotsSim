using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class BandwidthHelper : MonoBehaviour
{
    // Debugging variable
    public int priority = 1; // Priority level, script will start with Body, then priority 1, then priority 2, etc... until no objects are found

    // Disable sending position updates, but if referenced to an object, still make it track that object
    public bool do_not_update = false; // Turns off delta updating for this object

    // Disable rotation updates at all times (including if it's tracking an object)
    public bool do_not_rotate = false; // Prevents updating rotation

    // Disable position updates at all times (including if it's tracking an object)
    public bool do_not_move = false; // Prevents updating movement

    // Use global positioning
    public bool use_global = false;

    // Do not interpolate if object is kinematic: allows for teleportation of objects
    public bool no_kinematic_interpolation = false;
    private Rigidbody myrb;

    public interpolation interpolation_child;
    public bool pauseUpdates = false;  // If true, stops updating, allows an animation to take over 

    // Threshold above 0 to send.. also sets the threshold for location change to update
    public float pos_threshold = 0.0001f;
    public float angle_threshold = 0.0001f;

    // Specify which positions not to update
    // also specify it's accuracy
    public bool update_pos_x = true;
    public string update_pos_x_format = "0.####";
    public bool update_pos_y = true;
    public string update_pos_y_format = "0.####";
    public bool update_pos_z = true;
    public string update_pos_z_format = "0.####";

    public bool update_angle_x = true;
    public string update_angle_x_format = "0.####";
    public bool update_angle_y = true;
    public string update_angle_y_format = "0.####";
    public bool update_angle_z = true;
    public string update_angle_z_format = "0.####";

    // ************* NORMAL MODE STUFF ***************
    // Recorded starting positions
    // All positions/angles are sent as a difference to the starting, so that if the object's local position never really changes in one axis, it's never sent
    public Vector3 start_pos = new Vector3(0, 0, 0);
    public Quaternion start_angle;

    public Vector3 delta_pos = new Vector3(0, 0, 0);  // The difference in our position to where we should be - used by Get function to send data (server)
    public Quaternion delta_angle; // the difference in our angle to where we should be - used by Get function to send data (server)
    // **********************************************


    // ************ LINKED TO ANOTHER OBJECT STUFF ********************
    // Additionally, we have the capability of tracking another object, so it's local position changes hopefully track ours
    public Transform refpos;  // If set, uses it's position relative to this one
    public Quaternion ref_start_angle;
    // *******************************************

    public BHelperData extra_data;

    // ******** History for data optimization *******
    // Used to see if the object is moving or not.. if not, don't update as often
    // We can also use it for velocity calculations
    // Time of last update... monitor changes from last update before sending data (e.g. only send frequent data if position is changing)
    public Dictionary<int, long> LastUpdateTime = new Dictionary<int, long>();
    public Dictionary<int, Vector3> delta_pos_old = new Dictionary<int, Vector3>();
    public Dictionary<int, Quaternion> delta_angle_old = new Dictionary<int, Quaternion>();
    public Dictionary<int, string> extra_data_old = new Dictionary<int, string>();

    public Dictionary<int, Vector3> old_velocity = new Dictionary<int, Vector3>();
    public Dictionary<int, Vector3> old_angularVelocity = new Dictionary<int, Vector3>();

    private StringBuilder senddata = new StringBuilder();

    private bool initialized = false;
    private void OnEnable()  // Run before Start() - makes sure interpoalte hasn't changed transform yet
    {
        

        // Only to be used in non-single player mode
        if (GLOBALS.SINGLEPLAYER_MODE) { return; }

        // Only intialize first time it's created
        if (initialized) { return; }

        // Turn off interpolation before it has a chance to change things
        // Initialize starting position
        if (!refpos)
        {
            start_pos = (use_global) ? transform.position : transform.localPosition;
            start_angle = (use_global) ? transform.rotation : transform.localRotation;
        }
        // *************** OBJECT LINK MODE ******************
        // Initialize starting positions/angles
        else
        {
   
            start_pos = ((use_global) ? transform.position : transform.localPosition) - ((use_global) ? refpos.transform.position : refpos.transform.localPosition);
            // start_angle = Quaternion.Inverse((use_global) ? refpos.rotation : refpos.localRotation) * ((use_global) ? transform.rotation : transform.localRotation);
            start_angle = (use_global) ? transform.rotation : transform.localRotation;

            // In this mode, we do not want the interpolation to do the updating, we need to do it ourselves
            // because we need to link positions to the base object first
            // Also is initialized in Send() because it checks for it showing up
            if (interpolation_child) { interpolation_child.do_not_push = true; }
            // Record our reference starting position (so we know our relationship to the object at first, before it's moved/rotated)
            ref_start_angle = (use_global) ? refpos.rotation : refpos.localRotation;
        }

        initialized = true;

    }

    // Try using Global positions instead of locals due to hinge issues
    private void Start()
    {
        // Get the interpolation child
        // NOTE: Interpolation child MAY not be present till later
        interpolation_child = GetComponent<interpolation>();
        myrb = GetComponentInChildren<Rigidbody>(); // RB will be here from the beggining

        // *************** OBJECT LINK MODE ******************
        // Initialize starting positions/angles
        if (refpos)
        {
            // In this mode, we do not want the interpolation to do the updating, we need to do it ourselves
            // because we need to link positions to the base object first
            // Also is initialized in Send() because it checks for it showing up
            if (interpolation_child) { interpolation_child.do_not_push = true; }
        }

        // If extra_data wasn't specified, see if it's part of our root
        if (extra_data == null)
        {
            extra_data = GetComponent<BHelperData>();
        }
    }



    private void Update()
    {
        // Only to be used in client mode
        if (!GLOBALS.CLIENT_MODE ) { return; }

        if( pauseUpdates)
        {
            if( interpolation_child)
            {
                interpolation_child.pauseUpdates = true;
            }
            return;
        }
        else
        {
            if (interpolation_child)
            {
                interpolation_child.pauseUpdates = false;
            }
        }

        // If interpolation and refpos exists, then it's my job here to combine interpolated + reference 
        // Without a refpos, interpolation is left alone to do it's job in it's own update...
        // However, if the reference has interpolation and we don't, we still need to update here
        if (refpos)
        {
            // Make sure our parent is updated first
            if (refpos.GetComponent<interpolation>())
            {
                refpos.GetComponent<interpolation>().DoUpdate();
            }

            // If we have a valid interpolation, use it
            if (interpolation_child)
            {
                delta_pos = interpolation_child.calculated_pos;
                delta_angle.eulerAngles = interpolation_child.calculated_angle;
            }

            // Apply transformations
            if (!do_not_move) {
                if (use_global) { transform.position = CalculatePosWithRef(delta_pos); }
                else            { transform.localPosition = CalculatePosWithRef(delta_pos); }
            }


            if (!do_not_rotate) {
                if (GLOBALS.FORCE_OLD_BHELP)
                {
                    if (use_global) { transform.rotation = (Quaternion.Inverse(ref_start_angle) * refpos.rotation ) * start_angle * delta_angle; }
                    else            { transform.localRotation = (Quaternion.Inverse(ref_start_angle) * refpos.localRotation ) * start_angle * delta_angle; }
                }
                else
                {
                    if (use_global) { transform.rotation = refpos.rotation * Quaternion.Inverse(ref_start_angle) * delta_angle * start_angle; }
                    else { transform.localRotation = refpos.localRotation * Quaternion.Inverse(ref_start_angle) * delta_angle * start_angle; }
                }
            }

        }

        
    }
    private void UpdateCalcs()
    {
        // Simply calculate our current delta from starting position in the normal case
        if (refpos == null)
        {
            delta_pos = (((use_global) ? transform.position : transform.localPosition) - start_pos);
            delta_angle =  ((use_global) ? transform.rotation : transform.localRotation) * Quaternion.Inverse(start_angle);
            return;
        }

        // ******** Reference Object Stuff *********
        // Calculate our delta pos/angle

        // First, un-rotate the difference between us and the reference by how much the reference rotated
        Quaternion ref_inverse_delta_angle = Quaternion.Inverse(((use_global) ? refpos.rotation : refpos.localRotation) * Quaternion.Inverse(ref_start_angle));  // Transformation to go from rotation back to start_angle
        Vector3 my_pos_from_ref = ((use_global) ? transform.position : transform.localPosition) - ((use_global) ? refpos.transform.position : refpos.transform.localPosition); // Our current position relative to ref
        my_pos_from_ref = ref_inverse_delta_angle * my_pos_from_ref; // Our unrotated position relative to ref (as if ref never moved)

        // Now we can determine how much we moved relative to reference
        delta_pos = my_pos_from_ref - start_pos;

        // Our rotation also needs to be undone by how much ref rotated
        delta_angle = ref_inverse_delta_angle * ((use_global) ? transform.rotation : transform.localRotation) * Quaternion.Inverse(start_angle);

    }

    // Applies the transformation to the object
    private void ApplyTransformation(Vector3 new_delta_pos, Vector3 new_delta_angle, int timestamp = -1)
    {
        Vector3 final_pos;
        Quaternion final_angle;

        // Record it in the globals so the update can take advantage of it if required
        delta_pos = new_delta_pos;
        delta_angle = Quaternion.Euler(new_delta_angle);
        
        // If no refpos, then simply apply delta to start
        if (refpos == null)
        { 
            final_pos = new_delta_pos + start_pos;
            final_angle =  delta_angle * start_angle;
        }
        else
        {
            // If refpos exists, we want interpolation to track the delta pos/angles, not the absolutes
            // We will add the ref_pos position in Updates()
            if (interpolation_child)
            {
                if (!do_not_move) {   if (use_global) { interpolation_child.SetPosition(new_delta_pos, timestamp); } else { interpolation_child.SetLocalPosition(new_delta_pos, timestamp); } }
                if (!do_not_rotate) { if (use_global) { interpolation_child.SetRotation(new_delta_angle, timestamp); } else { interpolation_child.SetLocalRotation(new_delta_angle, timestamp); } }
                return;
            }

            // Otherwise, we need to calcualte the new absolute locations now
            final_pos = CalculatePosWithRef(new_delta_pos);
            final_angle = (Quaternion.Inverse(ref_start_angle) * ((use_global) ? refpos.rotation : refpos.localRotation)) * delta_angle *start_angle;
        }
      
        // Set new position
        if (interpolation_child)
        {
            if (!do_not_move) { if (use_global) { interpolation_child.SetPosition(final_pos, timestamp); } else { interpolation_child.SetLocalPosition(final_pos, timestamp); } }
            if (!do_not_rotate) { if (use_global) { interpolation_child.SetRotation(final_angle.eulerAngles, timestamp); } else { interpolation_child.SetLocalRotation(final_angle.eulerAngles, timestamp); } }
        }
        else
        {
            if (!do_not_move) { if (use_global) { transform.position = final_pos; } else { transform.localPosition = final_pos; }   }
            if (!do_not_rotate)  {
                if (use_global) { transform.rotation = final_angle; } else { transform.localRotation = final_angle; } }
        }
    }

    private Vector3 CalculatePosWithRef(Vector3 new_delta_pos)
    {      
        Quaternion ref_rotation_from_start = ((use_global) ? refpos.rotation : refpos.localRotation) * Quaternion.Inverse(ref_start_angle);     // How much ref rotated
        Vector3 my_pos_from_ref = start_pos + new_delta_pos;                             // Our new position relative to unrotated ref
        my_pos_from_ref = ref_rotation_from_start * my_pos_from_ref;  // Our roated position relative to ref

        return my_pos_from_ref + ((use_global) ? refpos.position : refpos.localPosition);   // Final position
    }

    // Returns a string representing this object's transform
    // The passed int represents the cache to use
    public bool do_update = true;
    public int doingit = 0;
    public int notdoingit = 0;
    private Dictionary<int, int> force_update_d = new Dictionary<int, int>();

    public string Get(int cache)
    {
        // If this object doesn't get updates, send empty string
        if( do_not_update) { return ""; }

        // Update important vars
        UpdateCalcs();

        // Make sure all caches have indexes
        if( !LastUpdateTime.ContainsKey(cache)) 
            { LastUpdateTime[cache] = 0; }
        if( !delta_pos_old.ContainsKey(cache)) 
            { delta_pos_old[cache] = new Vector3(0, 0, 0); }
        if( !delta_angle_old.ContainsKey(cache)) 
            { delta_angle_old[cache] = Quaternion.identity; }
        if (!extra_data_old.ContainsKey(cache)) 
            { extra_data_old[cache] =""; }
        if (!old_velocity.ContainsKey(cache))
            { old_velocity[cache] = new Vector3(0, 0, 0); }
        if (!old_angularVelocity.ContainsKey(cache))
            { old_angularVelocity[cache] = new Vector3(0, 0, 0); }
        if (!force_update_d.ContainsKey(cache))
            { force_update_d[cache] = 4; }

        int force_update = force_update_d[cache];

        // Next determine if we need to update or not
        Vector3 change_in_pos = delta_pos - delta_pos_old[cache];
        Vector3 change_in_rot = (delta_angle * Quaternion.Inverse(delta_angle_old[cache])).eulerAngles;
        long currtime = MyUtils.GetTimeMillis();
        float  time_delta = ((float) (currtime - LastUpdateTime[cache]))/1000f;

        // changes in extrapolated value
        Vector3 p_change_in_pos = old_velocity[cache] * time_delta;
        Vector3 p_change_in_rot = old_angularVelocity[cache] * time_delta;

        do_update = false;

        // If time has expired and is forcing an update, then do an update
        if ((currtime - LastUpdateTime[cache]) > GLOBALS.SERVER_MAX_UPDATE_DELAY)
        {
            do_update = true;
        }

        // Check movement threshold
        else if(update_pos_x && (Math.Abs(change_in_pos.x) > pos_threshold) ||
                update_pos_y && (Math.Abs(change_in_pos.y) > pos_threshold) ||
                update_pos_z && (Math.Abs(change_in_pos.z) > pos_threshold) )
        {
            do_update = true;
        }

        // Check rotation threshold
        else if(update_angle_x && (Math.Abs(change_in_rot.x) > angle_threshold) ||
                update_angle_y && (Math.Abs(change_in_rot.y) > angle_threshold) ||
                update_angle_z && (Math.Abs(change_in_rot.z) > angle_threshold))
        {
            do_update = true;
        }
        
        // Check extra data threshold
        else if(extra_data && (extra_data_old[cache] != extra_data.GetString()))
        {
            do_update = true;
        }

        // ****************************
        // Now check movement and rotation threshold based on interpolated movement
        if (update_pos_x && (Math.Abs(p_change_in_pos.x) > pos_threshold) ||
            update_pos_y && (Math.Abs(p_change_in_pos.y) > pos_threshold) ||
            update_pos_z && (Math.Abs(p_change_in_pos.z) > pos_threshold))
        {
            do_update = true;
        }
        else if (update_angle_x && (Math.Abs(p_change_in_rot.x) > angle_threshold) ||
            update_angle_y && (Math.Abs(p_change_in_rot.y) > angle_threshold) ||
            update_angle_z && (Math.Abs(p_change_in_rot.z) > angle_threshold))
        {
            do_update = true;
        }
        


        // If there is no need to update, then don't
        // However make sure to send an extra do_update so that speed can be zeroed out
        if (!do_update && (force_update <= 0f)) 
        {
            notdoingit += 1;
            return "";
            
        }

        if( do_update )
        {
            force_update = 2;
        }
        else
        {
            force_update -= 1;
        }

        force_update_d[cache] = force_update;
        doingit += 1;


        senddata.Clear();

        // Upodate x,y,z pos and angle but only if they change they are enabled
        // Also if they did not change by more then a minimum threhsold, then only send 0 for its place.

        if(update_pos_x)
        {
            if( Math.Abs(delta_pos.x) > pos_threshold) { senddata.Append(delta_pos.x.ToString(update_pos_x_format)); }
            else { senddata.Append(delta_pos.x.ToString("0")); }
        }

        if (update_pos_y)
        {
            if( senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }

            if (Math.Abs(delta_pos.y) > pos_threshold) { senddata.Append(delta_pos.y.ToString(update_pos_y_format)); }
            else { senddata.Append(delta_pos.y.ToString("0")); }
        }

        if (update_pos_z)
        {
            if (senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }

            if (Math.Abs(delta_pos.z) > pos_threshold) { senddata.Append(delta_pos.z.ToString(update_pos_z_format)); }
            else { senddata.Append(delta_pos.z.ToString("0")); }
        }

        //Vector3 delta_angleeulerAngles = GetPitchYawRollDeg(delta_angle);

        if (update_angle_x)
        {
            if (senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }

            if (Math.Abs(wrap(delta_angle.eulerAngles.x)) > angle_threshold) { senddata.Append(delta_angle.eulerAngles.x.ToString(update_angle_x_format)); }
            else { senddata.Append(delta_angle.eulerAngles.x.ToString("0")); }
        }

        if (update_angle_y)
        {
            if (senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }

            if (Math.Abs(wrap(delta_angle.eulerAngles.y)) > angle_threshold) { senddata.Append(delta_angle.eulerAngles.y.ToString(update_angle_y_format)); }
            else { senddata.Append(delta_angle.eulerAngles.y.ToString("0")); }
        }

        if (update_angle_z)
        {
            if (senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }

            if (Math.Abs(wrap(delta_angle.eulerAngles.z)) > angle_threshold) { senddata.Append(delta_angle.eulerAngles.z.ToString(update_angle_z_format)); }
            else { senddata.Append(delta_angle.eulerAngles.z.ToString("0")); }
        }

        // Send kinematic state
        if(no_kinematic_interpolation)
        {
            if (senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }
            senddata.Append(myrb.isKinematic ? "1" : "0");
        }

        if (extra_data)
        {
            if (senddata.Length > 0) { senddata.Append(GLOBALS.SEPARATOR2); }
            extra_data_old[cache] = extra_data.GetString();
            senddata.Append(extra_data_old[cache]);
        }

        // Update saved variables inlcuding velocity for interpolation (to determine if we should send an update)
        old_velocity[cache] = (delta_pos - delta_pos_old[cache]) / time_delta;

        // Mark our last updated time

        Vector3 angularVelocity = delta_angle.eulerAngles - delta_angle_old[cache].eulerAngles;
        angularVelocity.x = (float)MyUtils.AngleWrap(angularVelocity.x);
        angularVelocity.y = (float)MyUtils.AngleWrap(angularVelocity.y);
        angularVelocity.z = (float)MyUtils.AngleWrap(angularVelocity.z);

        angularVelocity /= time_delta;
        old_angularVelocity[cache] = angularVelocity;
        LastUpdateTime[cache] = currtime;
        delta_pos_old[cache] = delta_pos;
        delta_angle_old[cache] = delta_angle;

        return senddata.ToString();
    }

    // Sets the objects position based on string data
    public void Set( string data, int timestamp = -1)
    {

        // If this object doesn't get updates, send empty string
        if (do_not_update) { return; }

        // Divide up the incoming data
        string[] getdata = data.Split(GLOBALS.SEPARATOR2);
        Set(getdata, 0, timestamp);
    }
 
    // If data is already seperated, then start with the array and starting index
    // returns the next position in the data array
    public int Set(string[] getdata, int startpos, int timestamp = -1)
    {
        // If this object doesn't get updates, send empty string
        if (do_not_update ) { return ++startpos; }

        // If this element chose not to update itself, then startpos will be = Count-1 or the first item will be empty
        if( (getdata.Length-1 < startpos) || (getdata[startpos].Length < 1))
        {
            return startpos+1;
        }

        Vector3 new_delta_pos = Vector3.zero;
        Vector3 new_delta_angle = Vector3.zero;

        // Upodate x,y,z pos and angle but only if they change and are enabled
        try
        {
            if (update_pos_x) { new_delta_pos.x = float.Parse(getdata[startpos++]); }
            if (update_pos_y) { new_delta_pos.y = float.Parse(getdata[startpos++]); }
            if (update_pos_z) { new_delta_pos.z = float.Parse(getdata[startpos++]); }
            if (update_angle_x) { new_delta_angle.x = float.Parse(getdata[startpos++]); }
            if (update_angle_y) { new_delta_angle.y = float.Parse(getdata[startpos++]); }
            if (update_angle_z) { new_delta_angle.z = float.Parse(getdata[startpos++]); }
            if (no_kinematic_interpolation)           {
                if (interpolation_child)
                {
                    interpolation_child.disable2 = (getdata[startpos] == "1") ? true : false;
                }
                startpos++;
            }
            if (extra_data)     { extra_data.SetString(getdata[startpos++]); }
        }
        catch (Exception e)
        {
            Debug.LogError("BHelper: " + e.ToString());
            return startpos;
        }

        // Try to get inteprolation if it exists
        if (!interpolation_child)
        {
            interpolation_child = GetComponent<interpolation>();

            // If it showed up, we need to initialize it
            if (interpolation_child && refpos) { interpolation_child.do_not_push = true; }
        }

   

        ApplyTransformation(new_delta_pos, new_delta_angle, timestamp);

        return startpos;
    }

    float total_rotation = 0f;
    public void RotateAroundZ(float degrees)
    {
        total_rotation += degrees;
        total_rotation = wrap(total_rotation);
        ApplyTransformation(Vector3.zero, new Vector3(0,0, total_rotation));
    }

    private float wrap(float degrees)
    {
        while( degrees > 180f) { degrees -= 360f; }
        while( degrees < -180f) { degrees += 360f; }

        return degrees;
    }
}
