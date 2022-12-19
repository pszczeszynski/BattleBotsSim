using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class FreightFrenzy_Settings : GameSettings
{
    public bool ENABLE_AUTO_PUSHBACK = true;
    public Transform Rule_AutoPushback;

    public bool ENABLE_ANCHORS = false;
    public Toggle anchor_hubs;

    public bool ENABLE_HUB_PUSHBACK = false;
    public Toggle hub_pushback_toggle;

    public bool ENABLE_POSSESSION_LIMIT = true;
    public Toggle possession_limit;

    public int RESTART_POS_PENALTY = 30;
    public InputField reset_penalty_field;

    public int REF_RESET_PENALTY = 10;
    public InputField ref_restart_field;

    // Blocking robot settings
    public GameObject menu;
    public bool ENABLE_BLOCKING = false;
    public float BLOCKING_DURATION = 5f; // duration allowing blocking 
    public float BLOCKING_RESET_HOLDING_TIME = 2f;
    public int PENALTIES_MAJOR = 30;
    public int PENALTIES_MINOR = 10;


    public ConfigurableJoint[] hubs;
    public Generic_Robot_Pushback[] hubs_pushbacks;

    float last_time_update = 0f;
    Scorekeeper_FreightFrenzy ff_scorer;

    // Do overlay of info if appropriate
    bool old_toggle_button = false;

    public override void Update()
    {
        if (!GLOBALS.HEADLESS_MODE)
        {
            // ****** DETECT FPS + score details overlay
            bool toggle_button = Input.GetKey(KeyCode.Alpha2) && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift));

            // Check key-code for FPS overlay and score details
            if (!GLOBALS.keyboard_inuse && toggle_button && !old_toggle_button)
            {
                info.SetActive(!info.activeSelf);
            }
            old_toggle_button = toggle_button;
        }

        // Update twice a second
        if ( Time.time - last_time_update > 0.5f)
        {
            last_time_update = Time.time;
            if( info && red_details && blue_details && ff_scorer )
            {
                red_details.text = ff_scorer.GetDetails(true);
                blue_details.text = ff_scorer.GetDetails(false);
            }
        }
    }

    private bool init_done = false;
    public override void Start()
    {
        base.Start();
        init_done = false;

        // Initialize all values
        Rule_AutoPushback.GetComponent<Toggle>().isOn = ENABLE_AUTO_PUSHBACK;
        anchor_hubs.isOn = ENABLE_ANCHORS;
        hub_pushback_toggle.isOn = ENABLE_HUB_PUSHBACK;
        possession_limit.isOn = ENABLE_POSSESSION_LIMIT;
        reset_penalty_field.text = RESTART_POS_PENALTY.ToString();
        ref_restart_field.text = REF_RESET_PENALTY.ToString();

        ff_scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper_FreightFrenzy>();
        UpdateHubs();

        init_done = true;

    }
    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        // Change variables
        ENABLE_AUTO_PUSHBACK = Rule_AutoPushback.GetComponent<Toggle>().isOn;
        ENABLE_ANCHORS = anchor_hubs.isOn;
        ENABLE_HUB_PUSHBACK = hub_pushback_toggle.isOn;
        ENABLE_POSSESSION_LIMIT = possession_limit.isOn;
        RESTART_POS_PENALTY = int.Parse(reset_penalty_field.text);
        REF_RESET_PENALTY = int.Parse(ref_restart_field.text);
        UpdateHubs();
  
        UpdateServer();
    }

    private void UpdateHubs()
    {
        if (!GLOBALS.CLIENT_MODE)
        {
            // Anchor hubs if applicable
            foreach (ConfigurableJoint currhub in hubs)
            {
                if (!currhub) { continue; }

                if (!ENABLE_ANCHORS)
                {
                    currhub.xMotion = ConfigurableJointMotion.Free;
                    currhub.yMotion = ConfigurableJointMotion.Free;
                    currhub.zMotion = ConfigurableJointMotion.Free;
                }
                else
                {
                    currhub.xMotion = ConfigurableJointMotion.Limited;
                    currhub.yMotion = ConfigurableJointMotion.Locked;
                    currhub.zMotion = ConfigurableJointMotion.Locked;
                }
            }

            // Enable pushabck correctly
            foreach (Generic_Robot_Pushback pushback in hubs_pushbacks)
            {
                pushback.enable = ENABLE_HUB_PUSHBACK;
            }
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
        return ((ENABLE_AUTO_PUSHBACK) ? "1" : "0") + ":" + ((ENABLE_ANCHORS) ? "1" : "0") + ":" + ((ENABLE_HUB_PUSHBACK) ? "1" : "0") + ":" + ((ENABLE_POSSESSION_LIMIT) ? "1" : "0") + ":" + RESTART_POS_PENALTY.ToString() + ":" + REF_RESET_PENALTY.ToString();
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if (menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if (alldata.Length < 3)
        {
            Debug.Log("Freight Frenzy settings string did not have >=3 entries. It had " + alldata.Length);
            return;
        }

        ENABLE_AUTO_PUSHBACK = alldata[0] == "1";
        ENABLE_ANCHORS = alldata[1] == "1";
        ENABLE_HUB_PUSHBACK = alldata[2] == "1";

        if (alldata.Length >= 4)
        {
            ENABLE_POSSESSION_LIMIT = alldata[3] == "1";
        }

        if (alldata.Length >= 5)
        {
            int.TryParse(alldata[4], out RESTART_POS_PENALTY);
        }

        if (alldata.Length >= 6)
        {
            int.TryParse(alldata[5], out REF_RESET_PENALTY);
        }

        // Update menu
        Start();

        // Update hubs
        UpdateHubs();

        UpdateServer();
    }
}
