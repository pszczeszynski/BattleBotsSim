using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class ChangeUp_Settings : GameSettings
{
    public bool ENABLE_LINES = true;
    public bool ENABLE_LINES_SPEC = true;
    public bool ENABLE_LIGHTS = true;
    public bool ENABLE_AUTO_PUSHBACK = true;
   

    public GameObject menu;

    // Since menu is inactive at start, need to link them in unity - Find wont work
    public Transform Rule_Lines;
    public Transform Rule_Lines_Spec;
    public Transform Rule_Lights;
    public Transform Rule_AutoPushback;


    private bool init_done = false;
    override public void Start()
    {
        init_done = false;

        // Initialize all values
        Rule_Lines.GetComponent<Toggle>().isOn = ENABLE_LINES;
        Rule_Lines_Spec.GetComponent<Toggle>().isOn = ENABLE_LINES_SPEC;
        Rule_Lights.GetComponent<Toggle>().isOn = ENABLE_LIGHTS;
        Rule_AutoPushback.GetComponent<Toggle>().isOn = ENABLE_AUTO_PUSHBACK;
        base.Start();

        init_done = true;

    }
    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        ENABLE_LINES = Rule_Lines.GetComponent<Toggle>().isOn;
        ENABLE_LINES_SPEC = Rule_Lines_Spec.GetComponent<Toggle>().isOn;
        ENABLE_LIGHTS = Rule_Lights.GetComponent<Toggle>().isOn;
        ENABLE_AUTO_PUSHBACK = Rule_AutoPushback.GetComponent<Toggle>().isOn;
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
        return ((ENABLE_LINES) ? "1" : "0") + ":" +
               ((ENABLE_LINES_SPEC) ? "1" : "0") + ":" +
               ((ENABLE_LIGHTS) ? "1" : "0") + ":" +
               ((ENABLE_AUTO_PUSHBACK) ? "1" : "0");
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if (menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if (alldata.Length != 4)
        {
            Debug.Log("Infinite Recharge settings string did not have 4 entries. It had " + alldata.Length);
            return;
        }

        ENABLE_LINES = alldata[0] == "1";
        ENABLE_LINES_SPEC = alldata[1] == "1";
        ENABLE_LIGHTS = alldata[2] == "1";
        ENABLE_AUTO_PUSHBACK = alldata[3] == "1";
       
        // Update menu
        Start();
        UpdateServer();
    }
}
