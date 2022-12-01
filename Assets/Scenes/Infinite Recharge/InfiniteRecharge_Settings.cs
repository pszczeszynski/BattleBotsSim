using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class InfiniteRecharge_Settings : GameSettings
{
    public bool ENABLE_DEPOT_PENALTIES = false;   // Enables pushback on ALL FRC_Depots that have pushback enabled
    public float RESET_DURATION_UNDER_SKYBRIDGE = 2f;  // Seconds to hold player after reseting from under skybridge

    // Blocking robot settings
    public bool ENABLE_BLOCKING = true;
    public bool ENABLE_WALLBLOCKING = false;
    public float BLOCKING_DURATION = 5f; // duration allowing blocking 
    public float BLOCKING_RESET_HOLDING_TIME = 2f; // Time a robot is immobilized after reset
    public int PENALTY_BLOCKING = 15;
    public bool ENABLE_BALLRETURNTOBAY = true;

    public GameObject menu;

    // Since menu is inactive at start, need to link them in unity - Find wont work (at least not by default, learnt how to find inactive objects after implementing this)
    public Transform Rule_Depots;
    public Transform Rule_Blocking;
    public Transform Rule_WallBlocking;
    public Transform Penalty_Blocking;
    public Transform Rule_G10G11;
    public Transform Penalty_G10G11;
    public Transform G10_clearance;
    public Transform G11_blanking;
    public Transform BallReturnToBay;
    public Transform ShieldOffset;

    // IRL Settings
    public bool ENABLE_G10_G11 = true;
    public float G10_CLEAR_DISTANCE = 5f; // Units is meters, measured center of body to center of body
    public int G10_G11_BLANKING = 5000; // Blanking time (ms) before G10 G11 gets counted again
    public int PENALTY_G10_G11 = 25;
    public int SHIELD_OFFSET = 0;

    [Header("Power Ups")]
    public Transform Enable_Powerups;
    public bool ENABLE_POWERUPS = false;

    public Transform PU_Onlyone;
    public bool PU_ONLYONE = true;

    public Transform PU_Speed;
    public bool PU_SPEED = true;

    public Transform PU_Torque;
    public bool PU_TORQUE = true;

    public Transform PU_Invisible;
    public bool PU_INVISIBLE = true;

    public Transform PU_Slow;
    public bool PU_SLOW = true;

    public Transform PU_Weak;
    public bool PU_WEAK = true;

    public Transform PU_Inverted;
    public bool PU_INVERTED = true;

    public Transform PU_Off_duration;
    public int PU_OFFENSIVE_DURATION = 14;

    public Transform PU_Def_duration;
    public int PU_DEFENSIVE_DURATION = 7;

    public Transform PU_Center;
    public bool PU_CENTER = true;

    public Transform PU_center_type;
    public int PU_CENTER_TYPE = 1;  // 0 = offensive, 1 = defensive, 2 = both

    public Transform PU_home;
    public bool PU_HOME = true;

    public Transform PU_home_type;
    public int PU_HOME_TYPE = 0;  // 0 = offensive, 1 = defensive, 2 = both

    public Transform PU_Respawn_time;
    public int PU_RESPAWN = 15;

    public Transform PU_Strength;
    public int PU_STRENGTH = 100;

    [Header("Other")]
    public Transform Game_Version;
    public string GAMEVERSION = "2021";
    private string oldGAMEVERSION = "N/A";

    Scorekeeper_InfiniteRecharge ir_scorer;

    private void Awake()
    {
        // Overwrite unity setting if applicable
        // ENABLE_DEPOT_PENALTIES = false;


    }

    private bool init_done = false;

    override public void Start()
    {
        init_done = false;
        base.Start();

        ir_scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper_InfiniteRecharge>();

        // Initialize all values
        Rule_Depots.GetComponent<Toggle>().isOn = ENABLE_DEPOT_PENALTIES;
        Rule_Blocking.GetComponent<Toggle>().isOn = ENABLE_BLOCKING;
        Penalty_Blocking.GetComponent<InputField>().text = PENALTY_BLOCKING.ToString();
        BallReturnToBay.GetComponent<Toggle>().isOn = ENABLE_BALLRETURNTOBAY;

        Rule_G10G11.GetComponent<Toggle>().isOn = ENABLE_G10_G11;
        Penalty_G10G11.GetComponent<InputField>().text = PENALTY_G10_G11.ToString();
        G10_clearance.GetComponent<InputField>().text = G10_CLEAR_DISTANCE.ToString();
        G11_blanking.GetComponent<InputField>().text = G10_G11_BLANKING.ToString();
        ShieldOffset.GetComponent<InputField>().text = SHIELD_OFFSET.ToString();

        // Power Ups
        Enable_Powerups.GetComponent<Toggle>().isOn = ENABLE_POWERUPS;
        PU_Onlyone.GetComponent<Toggle>().isOn = PU_ONLYONE;
        PU_Speed.GetComponent<Toggle>().isOn = PU_SPEED;
        PU_Torque.GetComponent<Toggle>().isOn = PU_TORQUE;
        PU_Invisible.GetComponent<Toggle>().isOn = PU_INVISIBLE;
        PU_Slow.GetComponent<Toggle>().isOn = PU_SLOW;
        PU_Weak.GetComponent<Toggle>().isOn = PU_WEAK;
        PU_Inverted.GetComponent<Toggle>().isOn = PU_INVERTED;
        PU_Off_duration.GetComponent<InputField>().text = PU_OFFENSIVE_DURATION.ToString();
        PU_Def_duration.GetComponent<InputField>().text = PU_DEFENSIVE_DURATION.ToString();
        PU_Center.GetComponent<Toggle>().isOn = PU_CENTER;
        PU_center_type.GetComponent<Dropdown>().value = PU_CENTER_TYPE;
        PU_home.GetComponent<Toggle>().isOn = PU_HOME;
        PU_home_type.GetComponent<Dropdown>().value = PU_HOME_TYPE;
        PU_Respawn_time.GetComponent<InputField>().text = PU_RESPAWN.ToString();
        PU_Strength.GetComponent<InputField>().text = PU_STRENGTH.ToString();

        // Other
        Game_Version.GetComponent<Dropdown>().value = Game_Version.GetComponent<Dropdown>().options.FindIndex( a => a.text == GAMEVERSION);

        // Initialize game option to "2" for 2021 game
        GLOBALS.game_option = 2;

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
      
        ENABLE_DEPOT_PENALTIES = Rule_Depots.GetComponent<Toggle>().isOn;
        
        ENABLE_BLOCKING = Rule_Blocking.GetComponent<Toggle>().isOn;
        ENABLE_WALLBLOCKING = Rule_WallBlocking.GetComponent<Toggle>().isOn;
        ENABLE_BALLRETURNTOBAY = BallReturnToBay.GetComponent<Toggle>().isOn;

        if ( !ENABLE_BLOCKING && !ENABLE_WALLBLOCKING )
        {
            Penalty_Blocking.GetComponent<InputField>().interactable = false;
        }
        else
        {
            Penalty_Blocking.GetComponent<InputField>().interactable = true;
        }

        int.TryParse(Penalty_Blocking.GetComponent<InputField>().text, out PENALTY_BLOCKING);
        PENALTY_BLOCKING = Mathf.Abs(PENALTY_BLOCKING);

        ENABLE_G10_G11 = Rule_G10G11.GetComponent<Toggle>().isOn;
        int.TryParse(Penalty_G10G11.GetComponent<InputField>().text, out PENALTY_G10_G11);
        float.TryParse(G10_clearance.GetComponent<InputField>().text, out G10_CLEAR_DISTANCE);
        if(G10_CLEAR_DISTANCE < 2.0f)
        {
            G10_CLEAR_DISTANCE = 2.0f;
            G10_clearance.GetComponent<InputField>().text = G10_CLEAR_DISTANCE.ToString();
        }
        int.TryParse(G11_blanking.GetComponent<InputField>().text, out G10_G11_BLANKING);
        int.TryParse(ShieldOffset.GetComponent<InputField>().text, out SHIELD_OFFSET);

        // Power Ups
        ENABLE_POWERUPS = Enable_Powerups.GetComponent<Toggle>().isOn;
        PU_SPEED = PU_Speed.GetComponent<Toggle>().isOn;
        PU_ONLYONE = PU_Onlyone.GetComponent<Toggle>().isOn;
        PU_TORQUE = PU_Torque.GetComponent<Toggle>().isOn;
        PU_INVISIBLE = PU_Invisible.GetComponent<Toggle>().isOn;
        PU_SLOW = PU_Slow.GetComponent<Toggle>().isOn;
        PU_WEAK = PU_Weak.GetComponent<Toggle>().isOn;
        PU_INVERTED = PU_Inverted.GetComponent<Toggle>().isOn;
        int.TryParse(PU_Off_duration.GetComponent<InputField>().text, out PU_OFFENSIVE_DURATION);
        int.TryParse(PU_Def_duration.GetComponent<InputField>().text, out PU_DEFENSIVE_DURATION);
        PU_CENTER = PU_Center.GetComponent<Toggle>().isOn;
        PU_CENTER_TYPE = PU_center_type.GetComponent<Dropdown>().value;
        PU_HOME = PU_home.GetComponent<Toggle>().isOn;
        PU_HOME_TYPE = PU_home_type.GetComponent<Dropdown>().value;
        int.TryParse(PU_Respawn_time.GetComponent<InputField>().text, out PU_RESPAWN);
        int.TryParse(PU_Strength.GetComponent<InputField>().text, out PU_STRENGTH);

        GAMEVERSION = Game_Version.GetComponent<Dropdown>().options[Game_Version.GetComponent<Dropdown>().value].text;

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

        return ((ENABLE_DEPOT_PENALTIES) ? "1" : "0") + ":" +
               ((ENABLE_BLOCKING) ? "1" : "0") + ":" +
               PENALTY_BLOCKING.ToString() + ":" +
               ((ENABLE_G10_G11) ? "1" : "0") + ":" +
               PENALTY_G10_G11.ToString() + ":" +
               G10_CLEAR_DISTANCE.ToString() + ":" +
               G10_G11_BLANKING.ToString() + ":" +
               ((ENABLE_POWERUPS) ? "1" : "0") + ":" +
               ((PU_SPEED) ? "1" : "0") + ":" +
               ((PU_ONLYONE) ? "1" : "0") + ":" +
               ((PU_TORQUE) ? "1" : "0") + ":" +
               ((PU_INVISIBLE) ? "1" : "0") + ":" +
               ((PU_SLOW) ? "1" : "0") + ":" +
               ((PU_WEAK) ? "1" : "0") + ":" +
               ((PU_INVERTED) ? "1" : "0") + ":" +
               PU_OFFENSIVE_DURATION.ToString() + ":" +
               PU_DEFENSIVE_DURATION.ToString() + ":" +
               ((PU_CENTER) ? "1" : "0") + ":" +
               PU_CENTER_TYPE.ToString() + ":" +
               ((PU_HOME) ? "1" : "0") + ":" +
               PU_HOME_TYPE.ToString() + ":" +
               PU_RESPAWN.ToString() + ":" +
               PU_STRENGTH.ToString() + ":" +
               ((ENABLE_WALLBLOCKING) ? "1" : "0") + ":" +
               ((ENABLE_BALLRETURNTOBAY) ? "1" : "0") + ":" +
               GAMEVERSION.ToString() + ":" +
               SHIELD_OFFSET.ToString();
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if( menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if( alldata.Length < 3)
        {
            Debug.Log("Infinite Recharge settings string did not have at least 3 entries. It had " + alldata.Length);
            return;
        }

        int i = 0;

        ENABLE_DEPOT_PENALTIES = alldata[i++] == "1";
        ENABLE_BLOCKING = alldata[i++] == "1";
        PENALTY_BLOCKING = int.Parse(alldata[i++]);

        if (alldata.Length >= 7)
        {
            ENABLE_G10_G11 = alldata[i++] == "1";
            PENALTY_G10_G11 = int.Parse(alldata[i++]);
            G10_CLEAR_DISTANCE = float.Parse(alldata[i++]);
            G10_G11_BLANKING = int.Parse(alldata[i++]);
        }

        if (alldata.Length >= 22)
        {
            // Power Ups
            ENABLE_POWERUPS = alldata[i++] == "1";
            PU_SPEED = alldata[i++] == "1";
            PU_ONLYONE = alldata[i++] == "1";
            PU_TORQUE = alldata[i++] == "1";
            PU_INVISIBLE = alldata[i++] == "1";
            PU_SLOW = alldata[i++] == "1";
            PU_WEAK = alldata[i++] == "1";
            PU_INVERTED = alldata[i++] == "1";
            int.TryParse(alldata[i++], out PU_OFFENSIVE_DURATION);
            int.TryParse(alldata[i++], out PU_DEFENSIVE_DURATION);
            PU_CENTER = alldata[i++] == "1";
            PU_CENTER_TYPE = int.Parse(alldata[i++]);
            PU_HOME = alldata[i++] == "1";
            PU_HOME_TYPE = int.Parse(alldata[i++]);
            int.TryParse(alldata[i++], out PU_RESPAWN);
        }

        if (alldata.Length >= i+3)
        {
            int.TryParse(alldata[i++], out PU_STRENGTH);
            ENABLE_WALLBLOCKING = alldata[i++] == "1";
            ENABLE_BALLRETURNTOBAY = alldata[i++] == "1";
        }

        if (alldata.Length >= i+2 )
        {
            GAMEVERSION = alldata[i++];
            SHIELD_OFFSET = int.Parse(alldata[i++]);
        }

        // Update menu
        Start();

        // Send server info
        UpdateServer();
    }

    // Update field to new rules
    public void UpdateField(bool force = false)
    {
        if(!force && (oldGAMEVERSION == GAMEVERSION)) { return;  }

        // Get all the MyFlags objects
        MyFlags[] allflags = Resources.FindObjectsOfTypeAll<MyFlags>();

        // See if a field version exists
        foreach (MyFlags currflag in allflags)
        {
            // Get index of "FieldVersion"
            int index = currflag.flag.IndexOf("FieldVersion");
            if( index < 0) { continue; }

            // If current GAMEVERSION, enable, else disable
            if( currflag.value[index] == GAMEVERSION)
            {
                currflag.gameObject.SetActive(true);
            }
            else
            {
                currflag.gameObject.SetActive(false);
            }
        }

        oldGAMEVERSION = GAMEVERSION;

        switch (GAMEVERSION)
        {
            case "2020":
                GLOBALS.game_option = 1;
                break;
            case "2021":
                GLOBALS.game_option = 2;
                break;
        }

    }

    public override string GetCleanString()
    {
        return GetString();
        // return "," + GAMEVERSION + "," + ((ENABLE_BALLRETURNTOBAY) ? "LB" : "Center") + ",SO=" + SHIELD_OFFSET;
    }
}
