using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;

public class JoystickAdvancedMenu : MonoBehaviour
{
    public Dropdown joystic_select;
    public Dropdown axis_select;

    // Axis options
    public InputField axis_leftup;
    public InputField axis_rightdown;
    public InputField axis_deadzone;
    public Text axis_reading;

    // Button Option
    public Dropdown button_leftup;
    public Dropdown button_rightdown;

    // Display active joysticks
    public Text joystick_data_out;

    // Axis menu settings
    public GameObject buttons_settings;
    public GameObject axis_settings;

    public Text button_note;
    public Text threshold;
    public GameObject threshold_go;

    public bool isButton = false;

    public JoystickRawInfo joydata = null;
    public JoystickInfo callback = null;

    private void Update()
    {
        // If this menu is not active, then don't do anything
        if (!gameObject.activeSelf) { return; }

        // Clear old data
        joystick_data_out.text = "";

        // Create list of joystick activity
        for (int joystick_num = 1; joystick_num <= 6; joystick_num++)
        {
            // First add axis
            for (int axis = 1; axis <= 15; axis++)
            {
                // Make sure some basic threshold movement was met
                if (Math.Abs(Input.GetAxis("J" + joystick_num + "Axis" + axis)) > 0.05f)
                {
                    joystick_data_out.text += "Joy " + joystick_num + " Axis " + axis + "=" + Input.GetAxis("J" + joystick_num + "Axis" + axis).ToString("n2") + "\n";
                }
            }

            // Next add buttons
            for (int button = 0; button <= 19; button++)
            {
                if (Input.GetKey("joystick " + joystick_num + " " + "button " + button))
                {
                    joystick_data_out.text += "Joy " + joystick_num + " Button " + button + "\n";
                }
            }
        }

        // Update our existing settings
        int joy_num = joystic_select.value;
        int axis_num = axis_select.value;
        float axis_value = 0;
 
        // Update axis reading
        if( axis_num > 0)
        {
            axis_value = Input.GetAxis("J" + joy_num + "Axis" + axis_num);
            axis_reading.text = axis_value.ToString("n2");
        }

        // Update button setting
        string joystick_string = "joystick ";
        if(joy_num>0)
        {
            joystick_string += joy_num + " ";
        }


        if(Input.GetKey(joystick_string + "button " + button_leftup.value))
        {
            ColorBlock mycolors = button_leftup.colors;
            mycolors.normalColor = Color.green;
            button_leftup.colors = mycolors;
        }
        else
        {
            ColorBlock mycolors = button_leftup.colors;
            mycolors.normalColor = Color.white;
            button_leftup.colors = mycolors;
        }

        if (Input.GetKey(joystick_string + "button " + button_rightdown.value))
        {
            ColorBlock mycolors = button_rightdown.colors;
            mycolors.normalColor = Color.green;
            button_rightdown.colors = mycolors;
        }
        else
        {
            ColorBlock mycolors = button_rightdown.colors;
            mycolors.normalColor = Color.white;
            button_rightdown.colors = mycolors;
        }

        // Calculate our threshold and mark our reading green if above threshold for Button operations
        if (isButton && (axis_num > 0))
        {
            float lu_value = 0;
            float rd_value = 0;
            float deadzone = 0;

            float.TryParse(axis_leftup.text, out lu_value);
            float.TryParse(axis_rightdown.text, out rd_value);
            float.TryParse(axis_deadzone.text, out deadzone);

            float threshold_value = JoystickRawInfo.GetButtonThreshold(lu_value, rd_value, deadzone);
            threshold.text = threshold_value.ToString("N2");

            if( (lu_value < threshold_value) && (axis_value < threshold_value) ||
                (lu_value > threshold_value) && (axis_value > threshold_value))
            {
                axis_reading.color = Color.green;
            }
            else
            {
                axis_reading.color = Color.white;
            }
        }
        else
        {
            axis_reading.color = Color.white;
        }
    }

    private void Start()
    {
        // Init menu
        MenuChanged();
    }

    private bool blank_menuchanged = false;

    // Update the menu 
    public void MenuChanged()
    {
        // If we are manually changing values, blank the callback
        if(blank_menuchanged) { return; }

        // Toggle the correct settings menu
        if (axis_select.value == 0) // Joystick buttons selected
        {
            axis_settings.SetActive(false);
            buttons_settings.SetActive(true);
        }
        else
        {
            axis_settings.SetActive(true);
            buttons_settings.SetActive(false);
        }

        // If this is a button action, turn-off the irrelevant data
        if( isButton )
        {
            button_rightdown.gameObject.SetActive(false);
            button_note.gameObject.SetActive(true);
            threshold_go.SetActive(true);
        }
        else
        {
            button_rightdown.gameObject.SetActive(true);
            button_note.gameObject.SetActive(false);
            threshold_go.SetActive(false);
        }

        // If there is some joydata associated, then copy info over
        if( joydata != null)
        {
            // Joystick and axis pulldowns
            joydata.joystick_num = joystic_select.value;
            joydata.axis = axis_select.value;

            // Read in float values
            float lu_value = 0;
            float rd_value = 0;
            float deadzone = 0;

            if (float.TryParse(axis_leftup.text, out lu_value) )    { joydata.leftup_value = lu_value; };
            if (float.TryParse(axis_rightdown.text, out rd_value))  { joydata.rightdown_value = rd_value; };
            if (float.TryParse(axis_deadzone.text, out deadzone))   { joydata.dead_zone = deadzone; };

            // read in button values
            joydata.button_lu_num = button_leftup.value;
            joydata.button_rd_num = button_rightdown.value;
        }
    }

    public void Close()
    {
        // Delink joydata
        joydata = null;

        // Turn ourselves off
        gameObject.SetActive(false);

        // Call callback 
        if( callback != null )
        {
            callback.AdvMenuClosed();
            callback = null;
        }
    }

    // Initialize menu to the joydata
    public void Init(JoystickRawInfo joydata_in, JoystickInfo callback_in)
    {
        // Turn off menu callback
        blank_menuchanged = true;

        // Link to the data
        joydata = joydata_in;
        callback = callback_in;

        // Initialize all GUI to the settings
        isButton = joydata.isButton;
        joystic_select.value = joydata.joystick_num;
        axis_select.value = joydata.axis;
        axis_leftup.text = joydata.leftup_value.ToString("N2");
        axis_rightdown.text = joydata.rightdown_value.ToString("N2");
        axis_deadzone.text = joydata.dead_zone.ToString("N2");

        button_leftup.value = joydata.button_lu_num;
        button_rightdown.value = joydata.button_rd_num;

        // Turn callback back on
        blank_menuchanged = false;

        // Update GUI
        MenuChanged();
    }
}
