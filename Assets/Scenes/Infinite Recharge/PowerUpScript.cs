using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum PowerUpType
{
    NOTUSED = 0,

    // Offensive
    SPEED = 1,
    TORQUE,
    INVISIBILITY,

    // Defensive
    SLOW = 5,
    WEAK,
    INVERTED
};

public class PowerUpScript : MonoBehaviour
{
    public Scorekeeper myscorekeeper;
    public GameObject core_object;
    public float speed = 200f;
    public string text = "0";
    public Color text_color = new Color(1f, 1f, 1f);
    public List<Material> text_materials = new List<Material>();
    public int material_to_use = 0;
    public TMPro.TextMeshPro textfield;

    public PowerUpType myPower = PowerUpType.SPEED;
    private Animator my_animation;
    private AudioManager myaudio;

    // Start is called before the first frame update
    void Start()
    {
        // If we have a type specified, then set the values
        switch (myPower)
        {
            case PowerUpType.SPEED:
                text = "S";
                text_color.r = 1f; text_color.g = 1f; text_color.b = 1f; text_color.a = 1f;
                material_to_use = 0;
                break;

            case PowerUpType.TORQUE:
                text = "T";
                text_color.r = 1f; text_color.g = 1f; text_color.b = 1f; text_color.a = 1f;
                material_to_use = 0;
                break;
            case PowerUpType.INVISIBILITY:
                text = "I";
                text_color.r = 1f; text_color.g = 1f; text_color.b = 1f; text_color.a = 1f;
                material_to_use = 0;
                break;

            case PowerUpType.SLOW:
                text = "s";
                text_color.r = 0f; text_color.g = 0f; text_color.b = 0f; text_color.a = 1f;
                material_to_use = 1;
                break;
            case PowerUpType.WEAK:
                text = "W";
                text_color.r = 0f; text_color.g = 0f; text_color.b = 0f; text_color.a = 1f;
                material_to_use = 1;
                break;
            case PowerUpType.INVERTED:
                text = "i";
                text_color.r = 0f; text_color.g = 0f; text_color.b = 0f; text_color.a = 1f;
                material_to_use = 1;
                break;

            default:
                // Do nothing, based on other settings
                break;
        }

        textfield.text = text;
        textfield.color = text_color;
        my_animation = GetComponent<Animator>();
        textfield.fontSharedMaterial = text_materials[material_to_use];
        myaudio = GetComponent<AudioManager>();

        
    }

    private void OnEnable()
    {
        // Need to start off disabled so that first login will force syncrhonization
        PU_Disable();
    }

    // Update is called once per frame
    void Update()
    {
        // Set the y rotation
        Vector3 rot = transform.rotation.eulerAngles;
        rot.y += Time.deltaTime * speed;
        rot.y = rot.y % 360f;
        transform.rotation = Quaternion.Euler(rot);
    }

    private RobotInterface3D robot_owner = null;
    private long time_of_trigger = -9999999999;
    private bool needs_servicing = false;
    private bool myenabled = true; // Are we enabled?

    private void OnTriggerEnter(Collider other)
    {
        // Ignore this in client
        if( GLOBALS.CLIENT_MODE) { return;  }

        // If we are alread assigned, exit
        if (robot_owner != null) { return; }

        // Get the robotinterface3d
        RobotInterface3D robot = other.GetComponentInParent<RobotInterface3D>();
        if (!robot) { return; }

        // Check if we're allowed to assign this powerup
        if (!myscorekeeper.PU_CheckIfClearToAssign(this, robot)) { return; }

        robot_owner = robot;
        // Play the animation
        my_animation.Play("Base Layer.CLOSE");

        // Play Sound
        myaudio.Play("PowerUpGot");

        myenabled = false;
        time_of_trigger = MyUtils.GetTimeMillisSinceStart();
        needs_servicing = true;
    }

    // Did we mark ourselves for servicing?
    public bool NeedsServicing()
    {
        return needs_servicing;
    }

    // Mark the it was serviced
    public void Serviced()
    {
        needs_servicing = false;
    }

    // Returns the RobotID that owns this, otherwise it returns null
    public RobotInterface3D GetOwner()
    {
        if (robot_owner)
        {
            return robot_owner;
        }

        return null;
    }


    public long GetTimeStarted()
    {
        return time_of_trigger;
    }

    // Removes owner mark
    public void ClearOwner()
    {
        robot_owner = null;
        time_of_trigger = -1;
        needs_servicing = false;

    }

    public void PU_DisableWithAnimation()
    {
        my_animation.Play("Base Layer.CLOSE");
        myenabled = false;
    }

    public void PU_Disable()
    {
        core_object.SetActive(false);
        myenabled = false;
    }

    public bool IsDisabled()
    {
        return !myenabled;
    }

    public void PU_Enable(PowerUpType myNewPower = PowerUpType.SPEED)
    {
        // Clear old data
        robot_owner = null;
        time_of_trigger = -9999999999;
        needs_servicing = false;

        // Assign new powerup 
        myPower = myNewPower;

        core_object.SetActive(true);
        Start();
        my_animation.Play("Base Layer.OPEN");
        myenabled = true;
    }

}
