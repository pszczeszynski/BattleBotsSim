using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PacketDebugger : MonoBehaviour
{
    public Text max_packets_text;
    public int curr_packet = 0;
    public InputField curr_packet_field;

    public Text output_text;

    public Dropdown filter_dropdown;

    bool init_done = false;
    ClientLow myclient;


    public enum Packet_Type
    {
        UNKNOWN = 0,
        PLAYERS,
        NEWPLAYER,
        PLAYER_POS,
        FIELDELEMENTS,
        ERROR,
        FLAGS,
        LAST
    };

    int filter = -1;

    // Update is called once per frame
    void Update()
    {

    }

    public void GetNewFilter()
    {
        // Reset filter to be none
        filter = -1;

        for (int i = 0; i < (int)Packet_Type.LAST; i++)
        {
            if (filter_dropdown.options[filter_dropdown.value].text == ((Packet_Type)i).ToString())
            {
                filter = i;
                return;
            }
        }
    }

    public void Next()
    {
        curr_packet += 1;
        GetNewFilter();

        // Find our desired packet
        while (UpdateInfo())
        {
            if ((filter < 0) || ((int)curr_type == filter))
            {
                return;
            }
            curr_packet += 1;
        }
    }

    public void Prev()
    {
        curr_packet -= 1;
        GetNewFilter();

        while (UpdateInfo())
        {
            if ((filter < 0) || ((int)curr_type == filter))
            {
                return;
            }
            curr_packet -= 1;

        }

    }

    public void InputChanged()
    {
        int.TryParse(curr_packet_field.text, out curr_packet);

        UpdateInfo();
    }


    Packet_Type curr_type = Packet_Type.UNKNOWN;

    // Returns false if limit is reached
    public bool UpdateInfo()
    {
        curr_type = Packet_Type.UNKNOWN;

        bool return_value = true;

        // Update max packets
        max_packets_text.text = MyUtils.recorded_data.Count.ToString();

        // Sanitize settings
        if (curr_packet >= MyUtils.recorded_data.Count)
        {
            curr_packet = MyUtils.recorded_data.Count - 1;
            return_value = false;
        }

        if (curr_packet < 0)
        {
            curr_packet = 0;
            return_value = false;
        }

        // Sanitize curr field
        curr_packet_field.SetTextWithoutNotify(curr_packet.ToString());

        // Exit if no packets
        if (MyUtils.recorded_data.Count < 1) { return return_value; }

        // Get packet text info
        Saved_Data currdata = MyUtils.recorded_data[curr_packet];

        string out_text = "Timestamp = " + currdata.timestamp + "\n";

        if (currdata.data.Length < 3) {
            output_text.text = out_text;
            return return_value;

        }

        // Determine packet type
        switch (currdata.data[0])
        {
            case GLOBALS.HEADER_PLAYERS:
                curr_type = Packet_Type.PLAYERS;
                break;
            case GLOBALS.HEADER_NEWPLAYER:
                curr_type = Packet_Type.NEWPLAYER;
                break;
            case GLOBALS.HEADER_PLAYER_POS:
                curr_type = Packet_Type.PLAYER_POS;
                break;
            case GLOBALS.HEADER_FIELDELEMENTS:
                curr_type = Packet_Type.FIELDELEMENTS;
                break;
            case GLOBALS.HEADER_FLAGS:
                curr_type = Packet_Type.FLAGS;
                break;
            case GLOBALS.HEADER_ERROR:
                curr_type = Packet_Type.ERROR;
                break;
            default:
                curr_type = Packet_Type.UNKNOWN;
                break;
        };


        // Packet Type
        out_text += "Type=" + curr_type.ToString() + "\n";

        // Packet ID
        out_text += "ID = " + currdata.data[1] + "/" + currdata.data[2] + "\n";

        // Packet Data
        for (int i = 3; i < currdata.data.Length; i++)
        {
            out_text += PrintPretty(curr_type, currdata.data[i]) + "\n";
        }


        output_text.text = out_text;

        return return_value;
    }

    public string PrintPretty(Packet_Type type, string rawstring)
    {
        string out_string = rawstring;
        if(type == Packet_Type.PLAYER_POS)
        {
            string[] splitdata = rawstring.Split(GLOBALS.SEPARATOR2);
            if( splitdata.Length <3)
            {
                return out_string;
            }

            int id = int.Parse(splitdata[0]);
            int robotnamelength = int.Parse(splitdata[1]);
            out_string = "ID=" + id.ToString() + " NameLength = " + robotnamelength.ToString() + "\n";

            for( int i = 2; i < splitdata.Length; i++)
            {
                out_string += "   " + splitdata[i] + "\n";
            }
            

        }

        return out_string;
    }
}
