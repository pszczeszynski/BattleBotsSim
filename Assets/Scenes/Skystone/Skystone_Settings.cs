using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class Skystone_Settings : GameSettings
{
    public bool ENABLE_DEPOT_PENALTIES = true;
    public bool ENABLE_SKYBRIDGE_PENALTIES = true;
    public float RESET_DURATION_UNDER_SKYBRIDGE = 2f;  // Seconds to hold player after reseting from under skybridge
    public bool ENABLE_FOUNDATION_PENALTY = true;
    public bool ENABLE_POSSESSION_PENALTY = true;

    // Blocking robot settings
    public bool ENABLE_BLOCKING = true;
    public float BLOCKING_DURATION = 5f; // duration allowing blocking 
    public float BLOCKING_RESET_HOLDING_TIME = 2f;

    public int PENALTY_SKYBRIDGE = 5;
    public int PENALTY_BLOCKING = 5;
    public int PENALTY_SCORING = 20;
    public int PENALTY_POSSESSION = 5;
    public int PENALTY_POSSESSION_GRACE= 500;

    public GameObject menu;

    // Since menu is inactive at start, need to link them in unity - Find wont work
    public Transform Rule_Depots;
    public Transform Rule_Skybridge;
    public Transform Rule_Foundation;
    public Transform Rule_Blocking;
    public Transform Penalty_Skybridge;
    public Transform Penalty_Foundation;
    public Transform Penalty_Blocking;
    public Transform Rule_possession;
    public Transform Penalty_possession;
    public Transform Penalty_possession_grace;


    private bool init_done = false;
    override public void Start()
    {
        init_done = false;

        // Initialize all values
        Rule_Depots.GetComponent<Toggle>().isOn = ENABLE_DEPOT_PENALTIES;
        Rule_Skybridge.GetComponent<Toggle>().isOn = ENABLE_SKYBRIDGE_PENALTIES;
        Rule_Foundation.GetComponent<Toggle>().isOn = ENABLE_FOUNDATION_PENALTY;
        Rule_Blocking.GetComponent<Toggle>().isOn = ENABLE_BLOCKING;
        Rule_possession.GetComponent<Toggle>().isOn = ENABLE_POSSESSION_PENALTY;
        Penalty_Skybridge.GetComponent<InputField>().text = PENALTY_SKYBRIDGE.ToString();
        Penalty_Foundation.GetComponent<InputField>().text = PENALTY_SCORING.ToString();
        Penalty_Blocking.GetComponent<InputField>().text = PENALTY_BLOCKING.ToString();
        Penalty_possession.GetComponent<InputField>().text = PENALTY_POSSESSION.ToString();
        Penalty_possession_grace.GetComponent<InputField>().text = PENALTY_POSSESSION_GRACE.ToString();
        init_done = true;
        base.Start();


    }
    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        ENABLE_DEPOT_PENALTIES = Rule_Depots.GetComponent<Toggle>().isOn;
        ENABLE_SKYBRIDGE_PENALTIES = Rule_Skybridge.GetComponent<Toggle>().isOn;
        ENABLE_FOUNDATION_PENALTY = Rule_Foundation.GetComponent<Toggle>().isOn;
        ENABLE_BLOCKING = Rule_Blocking.GetComponent<Toggle>().isOn;
        ENABLE_POSSESSION_PENALTY = Rule_possession.GetComponent<Toggle>().isOn;
        int.TryParse(Penalty_Skybridge.GetComponent<InputField>().text, out PENALTY_SKYBRIDGE);
        int.TryParse(Penalty_Foundation.GetComponent<InputField>().text, out PENALTY_SCORING);
        int.TryParse(Penalty_Blocking.GetComponent<InputField>().text, out PENALTY_BLOCKING);
        int.TryParse(Penalty_possession.GetComponent<InputField>().text, out PENALTY_POSSESSION);
        int.TryParse(Penalty_possession_grace.GetComponent<InputField>().text, out PENALTY_POSSESSION_GRACE);
        PENALTY_SKYBRIDGE = Mathf.Abs(PENALTY_SKYBRIDGE);
        PENALTY_SCORING = Mathf.Abs(PENALTY_SCORING);
        PENALTY_BLOCKING = Mathf.Abs(PENALTY_BLOCKING);
        PENALTY_POSSESSION = Mathf.Abs(PENALTY_POSSESSION);
        PENALTY_POSSESSION_GRACE = Mathf.Abs(PENALTY_POSSESSION_GRACE);

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
        return ((ENABLE_DEPOT_PENALTIES) ? "1" : "0") + ":" +
               ((ENABLE_SKYBRIDGE_PENALTIES) ? "1" : "0") + ":" +
               ((ENABLE_FOUNDATION_PENALTY) ? "1" : "0") + ":" +
               ((ENABLE_BLOCKING) ? "1" : "0") + ":" +
               ((ENABLE_POSSESSION_PENALTY) ? "1" : "0") + ":" +
               PENALTY_SKYBRIDGE + ":" +
               PENALTY_SCORING + ":" +
               PENALTY_BLOCKING + ":" +
               PENALTY_POSSESSION + ":" +
               PENALTY_POSSESSION_GRACE;
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if (menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if (alldata.Length != 10)
        {
            Debug.Log("Infinite Recharge settings string did not have 10 entries. It had " + alldata.Length);
            return;
        }

        ENABLE_DEPOT_PENALTIES = alldata[0] == "1";
        ENABLE_SKYBRIDGE_PENALTIES = alldata[1] == "1";
        ENABLE_FOUNDATION_PENALTY = alldata[2] == "1";
        ENABLE_BLOCKING = alldata[3] == "1";
        ENABLE_POSSESSION_PENALTY = alldata[4] == "1";
        PENALTY_SKYBRIDGE = int.Parse(alldata[5]);
        PENALTY_SCORING = int.Parse(alldata[6]);
        PENALTY_BLOCKING = int.Parse(alldata[7]);
        PENALTY_POSSESSION = int.Parse(alldata[8]);
        PENALTY_POSSESSION_GRACE = int.Parse(alldata[9]);

        // Update menu
        Start();

        UpdateServer();
    }
}
