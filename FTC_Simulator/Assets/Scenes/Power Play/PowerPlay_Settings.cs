using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class PowerPlay_Settings : GameSettings
{
    public GameObject menu;

    public bool ENABLE_AUTO_PUSHBACK = true;
    public Transform Rule_AutoPushback;

    public bool ENABLE_WIRES = true;
    public Transform Show_Wires;

    public int RESTART_POS_PENALTY = 30;
    public InputField reset_penalty_field;

    public int REF_RESET_PENALTY = 10;
    public InputField ref_restart_field;

    public TMPro.TMP_Dropdown physics_speed;
    public int PHYSX_SPEED = 0;

    Scorekeeper_PowerPlay pp_scorer;

    private bool init_done = false;

    int old_iterations = 20;
    float old_timestep = 1 / 250f;
    private void Awake()
    {
        old_iterations = Physics.defaultSolverIterations;
        old_timestep = Time.fixedDeltaTime;
    }
    
    private void OnDestroy()
    {
        Physics.defaultSolverIterations = old_iterations;
        Time.fixedDeltaTime = old_timestep;
    }

    public override void Start()
    {
        base.Start();
        init_done = false;

        // Initialize all values
        Rule_AutoPushback.GetComponent<Toggle>().isOn = ENABLE_AUTO_PUSHBACK;
        Show_Wires.GetComponent<Toggle>().isOn = ENABLE_WIRES;
        reset_penalty_field.text = RESTART_POS_PENALTY.ToString();
        ref_restart_field.text = REF_RESET_PENALTY.ToString();
        physics_speed.value = PHYSX_SPEED;

        pp_scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper_PowerPlay>();
        SetPhysicsSpeed();
        init_done = true;

    }
    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        // Change variables
        ENABLE_AUTO_PUSHBACK = Rule_AutoPushback.GetComponent<Toggle>().isOn;
        ENABLE_WIRES = Show_Wires.GetComponent<Toggle>().isOn;

        RESTART_POS_PENALTY = int.Parse(reset_penalty_field.text);
        REF_RESET_PENALTY = int.Parse(ref_restart_field.text);
        PHYSX_SPEED = physics_speed.value;

        SetPhysicsSpeed();
        UpdateServer();
    }

   
    public void SetPhysicsSpeed()
    {
        // Find all game rigid bodies
        Rigidbody[] allbodies = Resources.FindObjectsOfTypeAll<Rigidbody>();

        int iterations = 20;
        float timestep = 1 / 250f;

        switch( PHYSX_SPEED)
        {
            case 0:
                // No change
                break;

            case 1:
                // Speed things up
                timestep = 1 / 500f;
                iterations = 10;
                break;
        }

        foreach( Rigidbody curr in allbodies)
        {
            curr.solverIterations = iterations;            
        }

        Physics.defaultSolverIterations = iterations;
        Time.fixedDeltaTime = timestep;
    }


    public void OnClose()
    {
        if (menu == null) { return; }

        menu.SetActive(false);
    }

    // Returns a string representation of settings
    override public string GetString()
    {
        return ((ENABLE_AUTO_PUSHBACK) ? "1" : "0") + ":" + ((ENABLE_WIRES) ? "1" : "0") + ":" + RESTART_POS_PENALTY.ToString() + ":" + REF_RESET_PENALTY.ToString() + ":" + PHYSX_SPEED.ToString();
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if (menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if (alldata.Length < 5)
        {
            Debug.Log("Power Play settings string did not have >=5 entries. It had " + alldata.Length);
            return;
        }

        ENABLE_AUTO_PUSHBACK = alldata[0] == "1";
        ENABLE_WIRES = alldata[1] == "1";
        int.TryParse(alldata[2], out RESTART_POS_PENALTY);
        int.TryParse(alldata[3], out REF_RESET_PENALTY);
        int.TryParse(alldata[4], out PHYSX_SPEED);

        // Update menu
        Start();

        UpdateServer();
    }
}
