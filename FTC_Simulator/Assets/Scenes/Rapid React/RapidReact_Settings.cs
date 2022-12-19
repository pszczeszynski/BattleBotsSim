using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class RapidReact_Settings : GameSettings
{
    public Transform Auto_Pushback;
    public bool ENABLE_AUTO_PUSHBACK = false;

    public float FOUL_BLANKING = 5f;

    public Toggle enable_possession;
    public bool ENABLE_POSSESSION_LIMIT = true;

    public InputField possession_foul;
    public int POSSESSION_PENALTY = 4;

    public Toggle enable_lp;
    public bool ENABLE_LP_FOUL = true;

    public InputField lp_foul;
    public int LP_FOUL_POINTS = 4;

    public Toggle enable_wp;
    public bool ENABLE_WALLPINS = true;

    public InputField wp_duration;
    public float BLOCKING_DURATION = 5f;
    public int PENALTY_BLOCKING = 4;

    public GameObject menu;

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

    Scorekeeper_RapidReact rr_scorer;

    private void Awake()
    {
        // Overwrite unity setting if applicable
        // ENABLE_DEPOT_PENALTIES = false;


    }

    private bool init_done = false;

    public override void Start()
    {
        base.Start();

        init_done = false;
        rr_scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper_RapidReact>();

        // Initialize all values

        Auto_Pushback.GetComponent<Toggle>().isOn = ENABLE_AUTO_PUSHBACK;
        enable_lp.isOn = ENABLE_LP_FOUL;
        lp_foul.text = LP_FOUL_POINTS.ToString();

        enable_possession.isOn = ENABLE_POSSESSION_LIMIT;
        possession_foul.text = POSSESSION_PENALTY.ToString();

        enable_wp.isOn = ENABLE_WALLPINS;
        wp_duration.text = BLOCKING_DURATION.ToString();

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

        ENABLE_AUTO_PUSHBACK = Auto_Pushback.GetComponent<Toggle>().isOn;
        ENABLE_LP_FOUL = enable_lp.isOn;
        LP_FOUL_POINTS = int.Parse(lp_foul.text);
        ENABLE_POSSESSION_LIMIT = enable_possession.isOn;
        POSSESSION_PENALTY= int.Parse(possession_foul.text);
        ENABLE_WALLPINS = enable_wp.isOn;
        BLOCKING_DURATION = float.Parse(wp_duration.text);


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

        return ((ENABLE_AUTO_PUSHBACK) ? "1" : "0") + ":" +
               ((ENABLE_LP_FOUL) ? "1" : "0") + ":" +
               LP_FOUL_POINTS.ToString() + ":" +
               ((ENABLE_POSSESSION_LIMIT) ? "1" : "0") + ":" +
               POSSESSION_PENALTY.ToString() + ":" +
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
               ((ENABLE_WALLPINS) ? "1" : "0") + ":" +
               BLOCKING_DURATION.ToString()
               ;
      
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

        ENABLE_AUTO_PUSHBACK = alldata[i++] == "1";
        ENABLE_LP_FOUL = alldata[i++] == "1";
        int.TryParse(alldata[i++], out LP_FOUL_POINTS);
        ENABLE_POSSESSION_LIMIT = alldata[i++] == "1";
        int.TryParse(alldata[i++], out POSSESSION_PENALTY);

        if (alldata.Length >= i+16)
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
            int.TryParse(alldata[i++], out PU_STRENGTH);
        }

        if( alldata.Length >= i+2)
        {
            ENABLE_WALLPINS = alldata[i++] == "1";
            float.TryParse(alldata[i++], out BLOCKING_DURATION);
        }

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
