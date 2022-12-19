using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class UltimateGoal_Settings : GameSettings
{  
    public Transform Penalty_Major;
    public int PENALTY_MAJOR = 30;

    public Transform Auto_Walls;
    public bool ENABLE_AUTO_WALLS = false;

    public Transform Zones_Walls;
    public bool ENABLE_WALL_ZONES = false;

    public Transform Zones_Field;
    public bool ENABLE_HALFFIELD_ZONES = false;

    public Transform Penalty_blocking;
    public int PENALTY_BLOCKING = 10;

    public Transform Enable_blocking;
    public bool ENABLE_BLOCKING = true;

    public GameObject menu;

    
    // Not exposed to GUI
    public float BLOCKING_DURATION = 5f; // duration allowing blocking 
    public float BLOCKING_RESET_HOLDING_TIME = 2f;

    private void Awake()
    {

    }

    private bool init_done = false;
    public override void Start()
    {
        init_done = false;

        // Initialize all values
        Penalty_Major.GetComponent<InputField>().text = PENALTY_MAJOR.ToString();
        Auto_Walls.GetComponent<Toggle>().isOn = ENABLE_AUTO_WALLS;
        Zones_Walls.GetComponent<Toggle>().isOn = ENABLE_WALL_ZONES;
        Zones_Field.GetComponent<Toggle>().isOn = ENABLE_HALFFIELD_ZONES;
        Penalty_blocking.GetComponent<InputField>().text = PENALTY_BLOCKING.ToString();
        Enable_blocking.GetComponent<Toggle>().isOn = ENABLE_BLOCKING;
        base.Start();

        init_done = true;
        return;
      

    }
    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        int.TryParse(Penalty_Major.GetComponent<InputField>().text, out PENALTY_MAJOR);
        PENALTY_MAJOR = Mathf.Abs(PENALTY_MAJOR);

        ENABLE_AUTO_WALLS = Auto_Walls.GetComponent<Toggle>().isOn;
        ENABLE_WALL_ZONES = Zones_Walls.GetComponent<Toggle>().isOn;
        ENABLE_HALFFIELD_ZONES = Zones_Field.GetComponent<Toggle>().isOn;

        int.TryParse(Penalty_blocking.GetComponent<InputField>().text, out PENALTY_BLOCKING);
        PENALTY_BLOCKING = Mathf.Abs(PENALTY_BLOCKING);

        ENABLE_BLOCKING = Enable_blocking.GetComponent<Toggle>().isOn;

        UpdateServer();

    }

    public void OnClose()
    {
        if (menu == null) { return; }

        menu.SetActive(false);
    }

    // Returns a string representation of settings
    override public string GetString()
    {

        return  PENALTY_MAJOR.ToString() + ":" + 
                (ENABLE_AUTO_WALLS ? "1" : "0") + ":" + 
                (ENABLE_WALL_ZONES ? "1" : "0") + ":" +
                (ENABLE_HALFFIELD_ZONES ? "1" : "0") + ":" + 
                PENALTY_BLOCKING.ToString() + ":" + 
                (ENABLE_BLOCKING ? "1" : "0");

    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if( menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if( alldata.Length != 6)
        {
            Debug.Log("Ultimate Goal settings string did not have 6 entries. It had " + alldata.Length);
            return;
        }

        PENALTY_MAJOR = int.Parse(alldata[0]);
        ENABLE_AUTO_WALLS = alldata[1] == "1";
        ENABLE_WALL_ZONES = alldata[2] == "1";
        ENABLE_HALFFIELD_ZONES = alldata[3] == "1";
        PENALTY_BLOCKING = int.Parse(alldata[4]);
        ENABLE_BLOCKING = alldata[5] == "1";

        // Update menu
        Start();

        UpdateServer();
    }

}
