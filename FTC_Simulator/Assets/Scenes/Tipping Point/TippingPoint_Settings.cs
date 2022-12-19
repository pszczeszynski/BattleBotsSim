using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class TippingPoint_Settings : GameSettings
{
 
    public bool ENABLE_AUTO_PUSHBACK = true;
   

    public GameObject menu;

    // Since menu is inactive at start, need to link them in unity - Find wont work
    public Transform Rule_AutoPushback;


    private bool init_done = false;
    override public void Start()
    {
        init_done = false;

        // Initialize all values
        Rule_AutoPushback.GetComponent<Toggle>().isOn = ENABLE_AUTO_PUSHBACK;
        base.Start();

        init_done = true;

    }
    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }
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
        return  ((ENABLE_AUTO_PUSHBACK) ? "1" : "0");
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if (menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if (alldata.Length != 1)
        {
            Debug.Log("Infinite Recharge settings string did not have 1 entries. It had " + alldata.Length);
            return;
        }

      
        ENABLE_AUTO_PUSHBACK = alldata[0] == "1";
       
        // Update menu
        Start();
        UpdateServer();
    }
}
