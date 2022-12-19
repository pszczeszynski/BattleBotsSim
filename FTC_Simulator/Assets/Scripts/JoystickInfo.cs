using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;


// Joystick setups
//
// Joystic Axis are referenced as:
//  J<0-6>Axis<1-15>
//  J0 = All joysticks, otherwise specifies specific joystick

public class JoystickRawInfo
{
    // If it's a button, then store it here
    public int joystick_num = 1; // Used for buttons and axis. 0 = any joystick, 6 is max number

    // If true, this function is a button/binary one
    public bool isButton = false;

    // If it's an axis
    public int axis = 1;  // number of the axis: 1 to 15. If 0, it means we use buttons
    public int button_lu_num = 0; // Button range = 0 - 19 . This is the button to use for single-action items
    public int button_rd_num = 1; // Button range = 0 - 19 
    public float leftup_value = -1f; // Maximum reading number
    public float rightdown_value = 1f; // minimum reading
    public float dead_zone = 0.05f; // dead-zone reading

    public string info; // Text shown in joystick menu

    public JoystickRawInfo(string info_in = "",
                           int joystick_num_in = 0,
                           bool isButton_in = false,
                           int axis_in = 1,
                           int button_lu_num_in = 0,
                           int button_rd_num_in = 1,
                           float leftup_value_in = -1f,
                           float rightdown_value_in = 1f,
                           float dead_zone_in = 0.05f)
    {
        info = info_in;
        joystick_num = joystick_num_in;
        isButton = isButton_in;
        axis = axis_in;
        button_lu_num = button_lu_num_in;
        button_rd_num = button_rd_num_in;
        leftup_value = leftup_value_in;
        rightdown_value = rightdown_value_in;
        dead_zone = dead_zone_in;
    }

    public void Copy(JoystickRawInfo inref)
    {
        info = inref.info;
        joystick_num = inref.joystick_num;
        isButton = inref.isButton;
        axis = inref.axis;
        button_lu_num = inref.button_lu_num;
        button_rd_num = inref.button_rd_num;
        leftup_value = inref.leftup_value;
        rightdown_value = inref.rightdown_value;
        dead_zone = inref.dead_zone;
    }

    // For situations where a button value is requested, return the state
    public bool GetButton()
    {
        // If using buttons, get button value
        if (axis == 0)
        {
            string joystick_string = "joystick ";
            if (joystick_num > 0)
            {
                joystick_string += joystick_num + " ";
            }


            return Input.GetKey(joystick_string + "button " + button_lu_num);
        }

        // Now threshold will be the average of start_value and leftup_value
        float threshold = GetButtonThreshold(leftup_value, rightdown_value, dead_zone);
        float read_value = Input.GetAxis("J" + joystick_num + "Axis" + axis);

        if ((leftup_value < threshold) && (read_value < threshold) ||
            (leftup_value > threshold) && (read_value > threshold))
        {
            return true;
        }

        return false;
    }

    public static float GetButtonThreshold(float my_leftup_value, float my_rightdown_value, float my_dead_zone)
    {
        // If using axis, get start of reading value 
        float start_value = (my_leftup_value + my_rightdown_value) / 2.0f;
        if (my_leftup_value < my_rightdown_value)
        {
            start_value -= my_dead_zone;
        }
        else
        {
            start_value += my_dead_zone;
        }

        // Now threshold will be the average of start_value and leftup_value
        return (my_leftup_value + start_value) / 2.0f;
    }

    // For situations where an axis value is requested, return the float value
    // The resturn value should be -1 to 1, where +1 = Left/Up and -1 = Right/Down
    public float GetAxis()
    {
        // If using buttons, get button value
        if (axis == 0)
        {
            string joystick_string = "joystick ";
            if (joystick_num > 0)
            {
                joystick_string += joystick_num + " ";
            }

            if(Input.GetKey(joystick_string + "button " + button_lu_num)) { return -1f; }
            if (Input.GetKey(joystick_string + "button " + button_rd_num)) { return 1f; }
            return 0f;
        }

        // Otherwise get the corrected axis value
        float zero_value = (rightdown_value + leftup_value) / 2.0f;
        float scale = Math.Abs(rightdown_value - zero_value) - dead_zone;
        if( scale == 0f) { scale = 0.0001f; } // Make sure scale doesn't give us divide-by-0
        float invert = (leftup_value > rightdown_value) ? -1f : 1f;
        float read_value = Input.GetAxis("J" + joystick_num + "Axis" + axis);

        // If we're in the dead-zone, then return 0
        if( Math.Abs(read_value - zero_value) < dead_zone) { return 0f; }

        // Correct reading: first center it and subtract dead-zone
        read_value -= zero_value;
        if (read_value > 0f) { read_value -= dead_zone; }
        else { read_value += dead_zone; }

        // Next scale it and invert it
        read_value /= scale * invert;

        // Finally limit it
        if( read_value < -1f) { read_value = -1f; }
        if( read_value > 1f ) { read_value = 1f; }
        return read_value;
    }

    public string GetString()
    {
        // Return latest value
        return "v1" + ";" +
                joystick_num + ";" +
                ((isButton) ? "1" : "0") + ";" +
                axis + ";" +
                button_lu_num + ";" +
                button_rd_num + ";" +
                leftup_value + ";" +
                rightdown_value + ";" +
                dead_zone + ";" +
                info;

               
        // Old string starts with 1 or 0
        // return ((isbutton) ? "1" : "0") + ";" + joystick_num.ToString() + ";" + button_num.ToString() + ";" + axis.ToString() + ";" +  ((invert) ? "1" : "0") + ";" + info;
    }

    public void FromString( string input)
    {
        string[] split = input.Split(';');

        // Something went wrong if not at least 6 items
        if (split.Length < 6) { return; }

        int i = 0;

        // Latest format
        if (split[0] == "v1")
        {
            i++;
            joystick_num = int.Parse(split[i++]);
            isButton = split[i++] == "1";
            axis = int.Parse(split[i++]);
            button_lu_num = int.Parse(split[i++]);
            button_rd_num = int.Parse(split[i++]);
            leftup_value = float.Parse(split[i++]);
            rightdown_value = float.Parse(split[i++]);
            dead_zone = float.Parse(split[i++]);
            info = split[i++]; ;
            return;
        }
        else
        // Original format
        {
            bool isbutton = (int.Parse(split[i++]) > 0) ? true : false;
            joystick_num = int.Parse(split[i++]);
            button_lu_num = int.Parse(split[i++]);
            axis = int.Parse(split[i++]);
            bool invert = (int.Parse(split[i++]) > 0) ? true : false;
            info = split[i++];

            // Translate original into new format
            if( isbutton )  { axis = 0; } // If we are using button, then turn axis off
            if( invert )
            {
                leftup_value = 1f;
                rightdown_value = -1f;
                dead_zone = 0.1f;
            }
            else
            {
                leftup_value = -1f;
                rightdown_value = 1f;
                dead_zone = 0.1f;
            }
        }
    }

    public bool isInverted()
    {
        return leftup_value > rightdown_value;
    }

}


public class JoystickInfo : MonoBehaviour
{
    public JoystickRawInfo joydata = new JoystickRawInfo();
    public JoystickAdvancedMenu adv_menu = null;
    
    // Button states 
    public bool getting_axis = false;

    // On start
    private void Start()
    {
        // Find the advanced menu
        JoystickAdvancedMenu[] all_adv_menues = Resources.FindObjectsOfTypeAll<JoystickAdvancedMenu>();
        if( all_adv_menues.Length >= 1)
        {
            adv_menu = all_adv_menues[0];
        }
    }

    public void Copy(JoystickRawInfo inref)
    {
        joydata.Copy(inref);
    }

    public void UpdateGUI()
    {
        transform.Find("InputField/Label").GetComponent<Text>().text = joydata.info;
        transform.Find("InputField/Text").GetComponent<Text>().text = GetJoystickText(joydata);
        if (joydata.isInverted())
        {
            Color newcolor = transform.Find("InputField").GetComponent<Image>().color;
            newcolor.r = 0.5f;
            transform.Find("InputField").GetComponent<Image>().color = newcolor;
        }
        else
        {
            Color newcolor = transform.Find("InputField").GetComponent<Image>().color;
            newcolor.r = 1f;
            transform.Find("InputField").GetComponent<Image>().color = newcolor;
        }
    }

    // Only called by GUI
    public void GUI_Start_Capturing()
    {
        // Only start capturing if "keyboard" is not in use
        if (GLOBALS.keyboard_inuse)
        { return; }

        if (joydata.isButton)
        {
            transform.Find("InputField/Text").GetComponent<Text>().text = "Press Button";
        }
        else
        {
            transform.Find("InputField/Text").GetComponent<Text>().text = "Move Up/Left";
        }

        GLOBALS.keyboard_inuse = true;
        getting_axis = true;
    }

    private void Update()
    {
        if (getting_axis)
        {
            if (joydata.isButton)
            {
                if (GetJoystickButton() >= 0)
                {
                    getting_axis = false;
                }
            }
            else
            {
                if (GetJoystickAxis() >= 0)
                {
                    getting_axis = false;
                }
            }

            // If we found what we were supposed to, then end this
            if (!getting_axis || Input.GetKeyDown(KeyCode.Escape))
            {
                UpdateGUI();
                GLOBALS.keyboard_inuse = false;

            }

        }

    }

    // Assigns the joystick num and button and returns the button if pressed.
    public int GetJoystickButton()
    {
        for (int i = 0; i <= 16; i++)
        {
            for (int j = 0; j < 20; j++)
            {
                if (Input.GetKey("joystick " + ((i > 0) ? i.ToString() + " " : "") + "button " + j.ToString()))
                {
                    joydata.joystick_num = i;
                    joydata.button_lu_num = j;
                    joydata.isButton = true;
                    return j;
                }
            }
        }

        return -1;
    }

    // assigns the axis number and returns its value
    public int GetJoystickAxis()
    {
        for (int i = 1; i <= 15; i++)
        {
            float read_value = Input.GetAxis("J0Axis" + i.ToString());

            if (Math.Abs(read_value) > 0.5f)
            {
                joydata.axis = i;

                if (read_value > 0.3f)
                {
                    joydata.leftup_value = 1f;
                    joydata.rightdown_value = -1f;
                }
                else
                {
                    joydata.leftup_value = -1f;
                    joydata.rightdown_value = 1f;
                }
                return i;
            }
        }

        return -1;
    }

    // Returns the text used to look up button or axis
    public string GetJoystickText(JoystickRawInfo j)
    {
        string out_string = "";

        // If this is linked to a joystick, then add the text
        if (j.joystick_num > 0)
        {
            out_string += "joy " + j.joystick_num + " ";
        }

        // Get button text
        if (j.isButton)
        {
            out_string += "button ";

            // If using axis threshold, add that text
            if( j.axis > 0 )
            {
                out_string += "axis " + j.axis;
            }
            else
            {
                out_string += j.button_lu_num.ToString();
            }
            
            return out_string;
        }

        // If using axis, mark it as well as button customs
        if( j.axis > 0 )
        {
            out_string += "axis " + j.axis;
        }
        else
        {
            out_string += "button " + j.button_lu_num.ToString() + " & " + j.button_rd_num;
        }
        return out_string;
    }

    // Callback to bring up adv_menu
    public void StartAdvMenu()
    {
        // Make sure the menu exists
        if( adv_menu == null) { return; }

        adv_menu.Init(joydata, this);
        adv_menu.gameObject.SetActive(true);
    }

    // Callback when AdvMenu closes
    public void AdvMenuClosed()
    {
        // Update our gui
        UpdateGUI();
    }
}



