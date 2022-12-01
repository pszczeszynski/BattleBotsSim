using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameSettings : MonoBehaviour
{
    // Link to the info window
    public GameObject info;
    public TMPro.TextMeshProUGUI red_details;
    public TMPro.TextMeshProUGUI blue_details;
    bool old_toggle_button = false;
    float last_time_update = 0f;
    Scorekeeper scorer; // The scorer we use to get score details from

    public virtual void Start()
    {
        scorer = GameObject.FindObjectOfType<Scorekeeper>();

        // Get our details components
        if( info && ((red_details == null) || (blue_details==null)) )
        {
            TMPro.TextMeshProUGUI[] alltexts = info.transform.GetComponentsInChildren<TMPro.TextMeshProUGUI>();

            foreach(TMPro.TextMeshProUGUI currtext in alltexts )
            {
                if( currtext.name == "Red")
                {
                    red_details = currtext;
                }

                if( currtext.name == "Blue")
                {
                    blue_details = currtext;
                }
            }

        }
    }

    public virtual void Update()
    {
        // If we have info window, then do the following
        if (info && scorer)
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
            if (Time.time - last_time_update > 0.5f)
            {
                last_time_update = Time.time;
                if (info && red_details && blue_details && scorer)
                {
                    red_details.text = scorer.GetDetails(true);
                    blue_details.text = scorer.GetDetails(false);
                }
            }
        }
    }


    // Let Server/Client know we changed
    public void UpdateServer()
    {
        // If this is a client let server know
        if( GLOBALS.CLIENT_MODE && GLOBALS.topclient)
        {
            GLOBALS.topclient.ChangeGameSettings(GetString());
        }

        // If this is a server, let admin know
        if (GLOBALS.SERVER_MODE && GLOBALS.topserver)
        {
            GLOBALS.topserver.ChangeGameSettings(GetString());
        }
    }

    // Sets all settings
    virtual public void SetString( string data)
    {

    }


    // Gets the string representation
    virtual public string GetString()
    {
        return "";
    }

    // Allow others to initialize settings if required
    virtual public void Init()
    {
        // nothing to do here
    }

    virtual public string GetCleanString()
    {
        return GetString();
    }
}
