using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class LastManStanding_Settings : GameSettings
{
    public GameObject menu;

    // Since menu is inactive at start, need to link them in unity - Find wont work
    // We want to set the number of hits before you die
    public Transform HitCount;
    public int HIT_COUNT = 2;

    // Set the maximum number of players
    public Transform PlayerCount;
    public int PLAYER_COUNT = 12;

    // Set if it's teams or not
    public Transform FreeForAll;
    public bool FREE_FOR_ALL = true;

    public Transform RowCount;
    public int ROW_COUNT = 3;

    public Transform MapSize;
    public float ROW_SPACING = 2.5f;

    public Transform SpawnWalls;
    public bool SPAWN_WALLS = true;


    private bool init_done = false;
    public override void Start()
    {

        // Initialize all values
        init_done = false;
        HitCount.GetComponent<InputField>().text = HIT_COUNT.ToString();
        PlayerCount.GetComponent<InputField>().text = PLAYER_COUNT.ToString();
        FreeForAll.GetComponent<Toggle>().isOn = FREE_FOR_ALL;
        RowCount.GetComponent<TMPro.TMP_Dropdown>().value = RowValueToIndex(ROW_COUNT);
        MapSize.GetComponent<TMPro.TMP_Dropdown>().value = RowSpacingToMapSize(ROW_SPACING);
        GLOBALS.PlayerCount = PLAYER_COUNT;
        SpawnWalls.GetComponent<Toggle>().isOn = SPAWN_WALLS;
        init_done = true;
        base.Start();

        MenuChanged();

        init_done = true;
    }

    int RowValueToIndex(int value)
    {
        switch (value)
        {
            case 3: return 0;

            case 4: return 1;

            default: return 0;
        }
    }

    int RowSpacingToMapSize(float value)
    {
       if( value < 2.9f) { return 0; }
       if( value > 3.1f) { return 2; }
       return 1;
    }

    float MapSizeToRowSpacing(string size)
    {
        if( size == "Small") { return 2.5f; }
        if (size == "Large") { return 3.5f; }
        return 3f;
    }

    public void MenuChanged()
    {
        if( menu == null) { return; }
        if( !init_done ) { return;  }

        FREE_FOR_ALL = FreeForAll.GetComponent<Toggle>().isOn;

        if ( int.TryParse(HitCount.GetComponent<InputField>().text, out HIT_COUNT) )
        {
            // Validay hit count is reasonable
            if( HIT_COUNT < 1) { HIT_COUNT = 1;  }
        }

        // Back annotate corrected number
        HitCount.GetComponent<InputField>().text = HIT_COUNT.ToString();

        if (int.TryParse(PlayerCount.GetComponent<InputField>().text, out PLAYER_COUNT))
        {
            // Validay hit count is reasonable
            if (PLAYER_COUNT < 2) { PLAYER_COUNT = 2; }
            if (PLAYER_COUNT > 12) { PLAYER_COUNT = 12; }
        }

        // Back annotate corrected number
        PlayerCount.GetComponent<InputField>().text = PLAYER_COUNT.ToString();

        GLOBALS.PlayerCount = PLAYER_COUNT;

        // Get Row count
        ROW_COUNT = int.Parse(RowCount.GetComponent<TMPro.TMP_Dropdown>().options[RowCount.GetComponent<TMPro.TMP_Dropdown>().value].text);
        ROW_SPACING = MapSizeToRowSpacing(MapSize.GetComponent<TMPro.TMP_Dropdown>().options[MapSize.GetComponent<TMPro.TMP_Dropdown>().value].text);

        SPAWN_WALLS = SpawnWalls.GetComponent<Toggle>().isOn;
        
        // Make sure spacing is not set if on smallest field
        if ((ROW_COUNT==3) && (MapSize.GetComponent<TMPro.TMP_Dropdown>().value==0))
        {
            SPAWN_WALLS = false;
            SpawnWalls.GetComponent<Toggle>().isOn = false;
            SpawnWalls.GetComponent<Toggle>().interactable = false;
        }
        else
        {
            SpawnWalls.GetComponent<Toggle>().interactable = true;
        }

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
        return ((FREE_FOR_ALL) ? "1" : "0") + ":" +
               HIT_COUNT.ToString() + ":" +
               PLAYER_COUNT.ToString() + ":" +
               ROW_COUNT.ToString() + ":" +
               ROW_SPACING.ToString() + ":" + 
               ((SPAWN_WALLS) ? "1" : "0");
    }

    // Sets the setting based on the string
    override public void SetString(string data)
    {
        // Do not override the menu if it is open
        if (menu.activeSelf) { return; }

        string[] alldata = data.Split(':');

        if( alldata.Length < 6)
        {
            Debug.Log("Free-For-All settings string did not have >=6 entries. It had " + alldata.Length);
            return;
        }

        FREE_FOR_ALL = alldata[0] == "1";
        HIT_COUNT = int.Parse(alldata[1]);
        PLAYER_COUNT = int.Parse(alldata[2]);
        ROW_COUNT = int.Parse(alldata[3]);
        ROW_SPACING = float.Parse(alldata[4]);
        SPAWN_WALLS = alldata[5] == "1";

        // Update menu
        Start();
        UpdateServer();
    }

}
