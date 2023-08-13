using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class BattleBots_Settings : GameSettings
{

    public float PHYSICS_TIMESTEP = 5f; // timestep in miliseconds

    public GameObject menu;
    public InputField Physics_timestep;
 

    Scorekeeper_BattleBots bb_scorer;

    private void Awake()
    {
        // Set the PHYSICS_TIMESTEP parameter to current physis
        PHYSICS_TIMESTEP = Time.fixedDeltaTime * 1000f;

    }

    private bool init_done = false;

    public override void Start()
    {
        base.Start();

        init_done = false;
        bb_scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper_BattleBots>();

        // Initialize all values
        Physics_timestep.GetComponent<InputField>().text = PHYSICS_TIMESTEP.ToString();

        init_done = true;
        Init();
    }

    public override void Init()
    {
        base.Init();
        UpdateField(true);
    }


    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        PHYSICS_TIMESTEP = float.Parse(Physics_timestep.text);

        Time.fixedDeltaTime = PHYSICS_TIMESTEP / 1000f;

        UpdateServer();
        UpdateField();

        // Negate clean run 
        if( GLOBALS.topsingleplayer && GLOBALS.topsingleplayer.scorer)
        {
            GLOBALS.topsingleplayer.scorer.clean_run = false;
        }
    }

    public void OnClose()
    {
        if (menu == null) { return; }

        menu.SetActive(false);
    }

    // Returns a string representation of settings
    override public string GetString()
    {

        return PHYSICS_TIMESTEP.ToString();
      
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if( menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if( alldata.Length < 1)
        {
            Debug.Log("Battle Bots settings string did not have at least 1 entries. It had " + alldata.Length);
            return;
        }

        int i = 0;
        float.TryParse(alldata[i++], out PHYSICS_TIMESTEP);

        // Update menu
        Start();

        // Send server info
        UpdateServer();
    }

    // Update field to new rules
    public void UpdateField(bool force = false)
    {
     
    }

    public override string GetCleanString()
    {
        return GetString();
    }
}
