using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using UnityEngine.Networking;
using UnityEngine.EventSystems;
using Ionic.Zlib;
using Assets.SimpleZip;
using System.Collections;
using System.Linq;

public enum SendDataFlags
{
    EVERYONE = 0,
    PLAYERS,
    SPECTATORS,
    TARGETID
}

public class ServerLow : MonoBehaviour
{
    public Scorekeeper scorer;
    public string SERVER_COMMENT = "";
    public bool REGISTER_SERVER = false;
    public int ROUTER_PORT = 11115;
    public int PORT = 11115;
    public string PASSWORD = "";
    public int max_spectators = 4;
    public bool tournament_mode = false;
    public bool holding_mode = false;
    public bool start_when_ready = true;
    public long UPDATE_DELAY = 25;
    public long MAX_BYTES = 6000;
    public string ADMIN = "";
    public bool CHAMPS_MODE = false; // enables championship mode
    public string CHAMPS_TEXT = "SRC Championship";

    // Cache structure
    public class CacheString
    {
        public StringBuilder msg = new StringBuilder();
        public int count = 0;

        public CacheString(string inmsg = "")
        {
            msg.Append(inmsg);
        }

        public void Clear()
        {
            msg.Clear();
            count = 0;
        }
    };

    enum TOURNAMENT_STATES
    {
        WAITING,
        HYPE1,
        COUNTDOWN,
        RUNNING,
        END,
        ALLDONE
    };

    private TOURNAMENT_STATES tournament_sm = TOURNAMENT_STATES.WAITING;

    Dictionary<String, int> netmonitor = new Dictionary<String, int>();
    string netmonitor_header;

    // Big String cache to minimize network trafic
    Dictionary<String, CacheString> netcache = new Dictionary<String, CacheString>();
    Dictionary<String, CacheString> netcache_spectators = new Dictionary<String, CacheString>();

    // *************************************************************************
    // ************* Game Object + Scene Manipulation Functions        *********
    // *************************************************************************

    private SortedDictionary<int, GameObject> allFieldElements = new SortedDictionary<int, GameObject>();
    private GameSettings ourgamesettings;

    void OnLevelFinishedLoading(Scene scene, LoadSceneMode mode)
    {
        if (scene.name == "fieldElements")
        { elements_load = true; }

        if (scene.name == "field")
        { 
            field_load = true;
        
            // The field scene wil ldefine the ambient lighting
            SceneManager.SetActiveScene(scene);
        }

        if (scene.name == "Scoring")
        { scorer_load = true; }

        if (scene.name == "server_gui")
        { gui_load = true; }

        if (elements_load && field_load && scorer_load && gui_load && !configuration_done)
        {
            messageLog = GameObject.Find("MessageLogText");

            // Find all field elements and put the in our dictionary using their index number
            allFieldElements.Clear();

            GameObject[] allelements;
            allelements = GameObject.FindGameObjectsWithTag("GameElement");
            foreach (GameObject currobj in allelements)
            {
                gameElement currelement = currobj.GetComponent<gameElement>();
                if (allFieldElements.ContainsKey(currelement.id)) { Debug.Log("Field element " + currelement.id + " is not unique id."); }
                else { 
                    allFieldElements.Add(currelement.id, currobj); 

                    // If it has a rigid body, set sleep to 0
                    if( currobj.GetComponent<Rigidbody>())
                    {
                        currobj.GetComponent<Rigidbody>().sleepThreshold = 0;
                    }
                }
            }

            ConfigureElements();

            scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper>();
            scorer.ScorerReset();

            configuration_done = true;

            ourgamesettings = FindObjectOfType<GameSettings>();

            if (ourgamesettings) { ourgamesettings.Init(); }


            // deal with Headless setup
            if ( GLOBALS.HEADLESS_MODE)
            {
                ProcessHeadlessCommands();
                ServerStartHeadless();
            }

            // Set the camera quality level
            Camera[] allcameras = Resources.FindObjectsOfTypeAll<Camera>();

            foreach (Camera currcamera in allcameras)
            {
                MyUtils.SetCameraQualityLevel(currcamera.gameObject);
            }

            MyUtils.QualityLevel_AdjustObjects();


        }
    }

    public void ProcessCommand( string command, string value)
    {
        // Process all command line arguments
        string[] arg_in = { command, value };


        if (arg_in[0].ToLower().Equals("tmode"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("TMODE incorrectly specified. Specify as TMODE=On/Off.");
                return;
            }

            if (arg_in[1].ToLower().Equals("on"))
            {
                MyUtils.LogMessageToFile("Tournament mode enabled.", false);
                tournament_mode = true;
            }
            else
            {
                MyUtils.LogMessageToFile("Tournament mode disabled.", false);
                tournament_mode = false;

            }
        }
        else
        if (arg_in[0].ToLower().Equals("startwhenready"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("STARTWHENREADY incorrectly specified. Specify as STARTWHENREADY=On/Off");
                return;
            }


            if (arg_in[1].ToLower().Equals("on"))
            {
                MyUtils.LogMessageToFile("Start-when-ready mode enabled.", false);
                start_when_ready = true;
            }
            else
            {
                MyUtils.LogMessageToFile("Start-when-ready mode disabled.", false);
                start_when_ready = false;
            }
        }
        else
        if (arg_in[0].ToLower().Equals("minplayers"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("MINPLAYERS incorrectly specified. Specify as MINPLAYERS=#");
                return;
            }

            //  Make sure it was an int
            int min_players = 0;
            if (!int.TryParse(arg_in[1], out min_players))
            {
                MyUtils.LogMessageToFile("MINPLAYERS incorrectly specified - unable to parse integer after MINPLAYERS= command line. Specify as MINPLAYERS=# where # >= 1.");
                return;
            }

            // Make sure Game is in range
            if (min_players < 1)
            {
                MyUtils.LogMessageToFile("MINPLAYERS number out of range. Needs to be >= 1. You specified " + min_players + ".");
                return;
            }

            GLOBALS.TMinPlayers = min_players;
            MyUtils.LogMessageToFile("MINPLAYERS set to " + min_players, false);
        }
        else
        if (arg_in[0].ToLower().Equals("holdingmode"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("HOLDINGMODE incorrectly specified. Specify as HOLDINGMODE=On/Off");
                return;
            }


            if (arg_in[1].ToLower().Equals("on"))
            {
                MyUtils.LogMessageToFile("Holding mode enabled.", false);
                holding_mode = true;
            }
            else
            {
                MyUtils.LogMessageToFile("Holding mode disabled.", false);
                holding_mode = false;
            }
        }
        else
        if (arg_in[0].ToLower().Equals("gameoption"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("GAMEOPTION incorrectly specified. Specify as GAMEOPTION=#");
                return;
            }

            //  Make sure it was an int
            int game_num = 0;
            if (!int.TryParse(arg_in[1], out game_num))
            {
                MyUtils.LogMessageToFile("GAMEOPTION incorrectly specified - unable to parse integer after GAMEOPTION= command line. Specify as GAMEOPTION=# where # starts from 1 to 5.");
                return;
            }

            // Make sure Game is in range
            if ((game_num < 1) || (game_num >= 5))
            {
                MyUtils.LogMessageToFile("GAMEOPTION number out of range. Starting game # = 1 and last game # is 5. You specified " + game_num + ".");
                return;
            }

            GLOBALS.game_option = game_num;
            MyUtils.LogMessageToFile("GAMEOPTION set to " + game_num, false);
        }
        else
        if (arg_in[0].ToLower().Equals("password"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("PASSWORD incorrectly specified. Specify as PASWORD=XXXXX, where X is an aplhanumeric with no spaces or = signs.");
                return;
            }

            PASSWORD = arg_in[1];
            MyUtils.LogMessageToFile("PASSWORD set to " + PASSWORD, false);
        }
        if (arg_in[0].ToLower().Equals("admin"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("ADMIN incorrectly specified. Specify as ADMIN=XXXXX, where X is an aplhanumeric with no spaces or = signs.");
                return;
            }

            ADMIN = arg_in[1];
            MyUtils.LogMessageToFile("ADMIN set to " + ADMIN, false);
        }
        if (arg_in[0].ToLower().Equals("spectators"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("SPECTATORS incorrectly specified. Specify as SPECTATORS=#");
                return;
            }

            //  Make sure it was an int
            int spec_num = 0;
            if (!int.TryParse(arg_in[1], out spec_num))
            {
                MyUtils.LogMessageToFile("SPECTATORS incorrectly specified - unable to parse integer after SPECTATORS= command line. Specify as SPECTATORS=# where # starts from 0.");
                return;
            }

            // Make sure Game is in range
            if (spec_num < 0)
            {
                MyUtils.LogMessageToFile("SPECTATORS number out of range, must be >0. You specified " + spec_num + ".");
                return;
            }

            max_spectators = spec_num;
            MyUtils.LogMessageToFile("Max spectators set to " + spec_num, false);
        }
        else
        if (arg_in[0].ToLower().Equals("routerport"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("ROUTERPORT incorrectly specified. Specify as ROUTERPORT=#####");
                return;
            }

            //  Make sure it was an int
            int game_num = 0;
            if (!int.TryParse(arg_in[1], out game_num))
            {
                MyUtils.LogMessageToFile("ROUTERPORT incorrectly specified - unable to parse integer after ROUTERPORT= command line. Specify as ROUTERPORT=# where # starts from 1.");
                return;
            }

            // Make sure Game is in range
            if ((game_num < 1) || (game_num >= 65535))
            {
                MyUtils.LogMessageToFile("ROUTERPORT number out of range. Starting port # = 1 and last port # is 65535. You specified " + game_num + ".");
                return;
            }

            ROUTER_PORT = game_num;
            MyUtils.LogMessageToFile("Router port set to " + game_num, false);
        }
        else
        if (arg_in[0].ToLower().Equals("port"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("PORT incorrectly specified. Specify as PORT=#####");
                return;
            }

            //  Make sure it was an int
            int game_num = 0;
            if (!int.TryParse(arg_in[1], out game_num))
            {
                MyUtils.LogMessageToFile("PORT incorrectly specified - unable to parse integer after PORT= command line. Specify as PORT=# where # starts from 1.");
                return;
            }

            // Make sure Game is in range
            if ((game_num < 1) || (game_num >= 65535))
            {
                MyUtils.LogMessageToFile("PORT number out of range. Starting port # = 1 and last port # is 65535. You specified " + game_num + ".");
                return;
            }

            PORT = game_num;
            MyUtils.LogMessageToFile("Port set to " + game_num, false);
        }
        else
        if (arg_in[0].ToLower().Equals("register"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("Register incorrectly specified. Specify as REGISTER=On/Off");
                return;
            }

            if (arg_in[1].ToLower().Equals("on"))
            {
                MyUtils.LogMessageToFile("Register server enabled.", false);
                REGISTER_SERVER = true;
            }
            else
            {
                MyUtils.LogMessageToFile("Register server disabled.", false);
                REGISTER_SERVER = false;
            }
        }
        else
        if (arg_in[0].ToLower().Equals("comment"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("Comment incorrectly specified. Specify as Comment=xxxxx");
                return;
            }

            MyUtils.LogMessageToFile("Server comment field set to " + arg_in[1], false);
            SERVER_COMMENT = arg_in[1];
        }
        else
        if (arg_in[0].ToLower().Equals("updatetime"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("UPDATETIME incorrectly specified. Specify as UPDATETIME=##");
                return;
            }

            //  Make sure it was an int
            int game_num = 0;
            if (!int.TryParse(arg_in[1], out game_num))
            {
                MyUtils.LogMessageToFile("UPDATETIME incorrectly specified - unable to parse integer after UPDATETIME= command line. Specify as UPDATETIME=# where # starts from 1.");
                return;
            }

            // Make sure Game is in range
            if ((game_num < 1) || (game_num >= 1000))
            {
                MyUtils.LogMessageToFile("UPDATEIME number out of range. Min time # = 1 and max port # is 1000 ms. You specified " + game_num + ".");
                return;
            }

            UPDATE_DELAY = game_num;
            MyUtils.LogMessageToFile("Update time set to " + game_num, false);
        }
        else
        if (arg_in[0].ToLower().Equals("maxdata"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("MAXDATA incorrectly specified. Specify as MAXDATA=####");
                return;
            }

            //  Make sure it was an int
            int game_num = 0;
            if (!int.TryParse(arg_in[1], out game_num))
            {
                MyUtils.LogMessageToFile("MAXDATA incorrectly specified - unable to parse integer after MAXDATA= command line. Specify as MAXDATA=#### where # starts from 1.");
                return;
            }

            // Make sure Game is in range
            if (game_num < 1)
            {
                MyUtils.LogMessageToFile("MAXDATA number out of range. Min time # = 1. You specified " + game_num + ".");
                return;
            }

            MAX_BYTES = game_num;
            MyUtils.LogMessageToFile("Max data set to " + game_num, false);
        }
        else
        if (arg_in[0].ToLower().Equals("netstats"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("Netstats incorrectly specified. Specify as REGISTER=On/Off");
                return;
            }

            if (arg_in[1].ToLower().Equals("on"))
            {
                MyUtils.LogMessageToFile("Netstats enabled.", false);
                GLOBALS.NETSTATS = 1;
            }
            else if((arg_in[1].ToLower().Equals("admin")))
            {
                MyUtils.LogMessageToFile("Netstats in admin mode.", false);
                GLOBALS.NETSTATS = 2;
            }
            else 
            {
                MyUtils.LogMessageToFile("Netstats disabled.", false);
                GLOBALS.NETSTATS = 0;
            }
        }
        if (arg_in[0].ToLower().Equals("champs"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("Championship mode requires one argument to set the field text.");
                return;
            }

            CHAMPS_MODE = true;
            CHAMPS_TEXT = arg_in[1];
            ToggleChampsMode();
            MyUtils.LogMessageToFile("Champs mode enabled with text = " + CHAMPS_TEXT, false);
        }
        else if (arg_in[0].ToLower().Equals("output_score_files") )
        {
            // Make sure a score file directory argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("OUTPUT_SCORE_FILES needs a directory destination.");
                return;
            }

            GLOBALS.OUTPUT_SCORING_FILES = true;
            MyUtils.status_file_dir = arg_in[1];
            MyUtils.LogMessageToFile("Score files output set to directory " + arg_in[1], false);
        }
        if (arg_in[0].ToLower().Equals("gamesettings"))
        {
            // Make sure a game argument was specified
            if (arg_in.Length < 2)
            {
                MyUtils.LogMessageToFile("GAMESETTINGS requires one argument to set the options.");
                return;
            }

            if (ourgamesettings)
            {
                ourgamesettings.SetString(arg_in[1]);
            }

            MyUtils.LogMessageToFile("Game settings set to " + arg_in[1], false);
        }

    }

    // Deals with command line stuff that needs to be process if headless
    public void ProcessHeadlessCommands()
    {
        // If we're in headless mode, read in all the line commands
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length; i++)
        {
            string[] arg_in = args[i].Split('=');

            ProcessCommand(arg_in[0], ((arg_in.Length > 1) ? arg_in[1] : ""));
        }

    }

    private void ConfigureElements()
    {
        // Turn off interpolation if accidentally turned on
        SortedDictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;

        foreach (GameObject currobj in allelements)
        {
            interpolation currinterpolation = currobj.GetComponent<interpolation>();

            if (currinterpolation != null) { currinterpolation.enabled = false; }
        }
    }


    private void TurnOffInterpolationInObject(GameObject inobject)
    {
        // Iterate through every child of the object
        for (int j = 0; j < inobject.transform.childCount; j++)
        {
            // Get the interpolation
            interpolation currint = inobject.transform.GetChild(j).GetComponent<interpolation>();
            if (currint != null)
            {
                currint.enabled = false;
            }
        }

        // Don't forget the parent
        interpolation currintp = inobject.transform.GetComponent<interpolation>();
        if (currintp != null)
        {
            currintp.enabled = false;
        }
    }



    // move field elements from bad places (like off screen)
    private void CorrectFieldElements()
    {
        // Check for having flown off the grid
        SortedDictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;

        foreach (GameObject currobj in allelements)
        {
            scorer.CorrectFieldElement(currobj);
        }

   
    }

    // move players from bad positions and/or reset if a reset is required
    private void CorrectPlayers()
    {
        foreach(int currclient in allClients.Keys)
        {
            if ((allClients[currclient].avatar == null) ||
                (allClients[currclient].robot == null)  ||
                (allClients[currclient].robot.rb_body == null) )
                { continue; };

            // if object flew off board, correct it
            if (scorer.IsTransformOffField(allClients[currclient].robot.rb_body.transform))
            {
                // Destructive Reset: all data associated with player is destroyed, but not that held in RobotID
                ResetAvatarPosition(currclient);
            }

            // If a reset is required, correct it
            if(allClients[currclient].robot.GetNeedsReset())
            {
                allClients[currclient].reset_release = Time.time + allClients[currclient].robot.GetResetDuration();
                ResetAvatarPosition(currclient);
            }

            // If holding is required, mark it
            if(allClients[currclient].reset_release > Time.time)
            {
                allClients[currclient].robot.HoldRobot(true);
            }
            else
            {
                if(allClients[currclient].reset_release > 0f)
                {
                    allClients[currclient].robot.HoldRobot(false);
                    allClients[currclient].reset_release = 0f;
                }
            }

        }
    }

    private void ResetAvatarPosition(int playerID)
    {
        // Confirm valid player and that he has an avatar
        if (!allClients.ContainsKey(playerID)) { return; }
        if (allClients[playerID].avatar == null) { return; }

        // Save the robotid data
        RobotID_Data olddata = new RobotID_Data();
        olddata.Copy(allClients[playerID].robot.myRobotID);


        // We will delete the old version and create the new one
        if (allClients[playerID].robot) { allClients[playerID].robot.deleted = true; }
        Destroy(allClients[playerID].avatar);

        CreatePlayerAvatar(allClients[playerID]);
        allClients[playerID].robot.myRobotID.Copy(olddata); // Resotre player relevant data that's not stored inside the RobotInterface3D
       
        // Let scorer know that there is a new instance of the robot
        scorer.FieldChanged();

        return;
    }   

    // Resets all elements to their starting position
    private void RestartLevel()
    {
        // Restart scorer
        // Do it first so it can reset objects that aren't destroyed yet
        scorer.ScorerReset();
        scorer.Restart();

        // Go through clients and reset their positions
        foreach (int currclient in allClients.Keys)
        {
            ResetAvatarPosition(currclient);
        }

        // Go through all game objects and reset their position
        foreach (int currkey in allFieldElements.Keys)
        {
            gameElement currelement = allFieldElements[currkey].GetComponent<gameElement>();
            if (currelement != null)
            { currelement.ResetPosition(GLOBALS.game_option); }        
        }

        // Reset Timer
        scorer.OnTimerReset();

        // Force a scorer reset
        scorer.DoFieldChanged();

        // Start Timer
        scorer.OnTimerClick();

        // Clear unused memory
        Resources.UnloadUnusedAssets();
    }

    private GameObject redtextobj = null;
    private GameObject bluetextobj = null;

    // Update the score field
    public void UpdateScore()
    {
        // Make sure all scenes are loaded 
        if (scorer == null) { return; }

        // This fills out the serve flags, doesn't actually send them
        scorer.SendServerData(serverFlags);

        // Get the red score
        if (redtextobj == null)
        { 
            redtextobj = GameObject.Find("REDSCORE");
            
            // Make sure we found the score
            if (redtextobj == null)
            { return; }
        }

        // Get the blue score
        if (bluetextobj == null)
        { 
            bluetextobj = GameObject.Find("BLUESCORE");

            // Make sure we found the score
            if (bluetextobj == null)
            { return; }
        }

        redtextobj.GetComponent<Text>().text = scorer.GetRedScore().ToString();
        bluetextobj.GetComponent<Text>().text = scorer.GetBlueScore().ToString();
    }

    // *************************************************************************
    // ************* LOGS, Status Messages                             *********
    // *************************************************************************


    // **************  Message Line *********************
    // Limit to 50 messages
    private GameObject messageBox;
    long time_of_message;
    private StringBuilder[] msg_lines = new StringBuilder[50];
    private StringBuilder textmsg = new StringBuilder();
    public GameObject clientlineprefab;
    private bool msgbox_init_done = false;

    public void ShowMessage(string message, int starting_line = 0, bool clear_all = true)
    {
        if(!gui_load) { return; }

        // First attempt to find the message Box if it exists
        if (messageBox == null)
        {
            if (!msgbox_init_done)
            {
                messageBox = GameObject.Find("MessageBox");
                msgbox_init_done = true;
            }
            if (messageBox == null) { return; }
        }

        Text textobj = messageBox.GetComponent<Text>();

        if (textobj == null)
        { return; }

        // Clear it if requested
        if( clear_all )
        {
            for( int i =0; i < msg_lines.Length; i++)
            {
                if (msg_lines[i] == null)
                {
                    msg_lines[i] = new StringBuilder();
                }
                else
                {
                    msg_lines[i].Clear();
                }
            }
        }

        // Add our message
        string[] lines = message.Split('\n');

        for( int i = 0; i < lines.Length; i++)
        {
            if( starting_line + i >= msg_lines.Length )
            { break; }

            msg_lines[i + starting_line].Clear().Append(lines[i]);
        }

        // Create master string
        textmsg.Clear();
        int empty_string = 0;

        // Go through every line and add it
        for( int i=0; i < msg_lines.Length; i++)
        {
            // If the line is empty, skip it but keep track that we did. This way if there
            // is a populated line further down, we can back-fill with newlines
            if(msg_lines[i].Length<1)
            {
                empty_string++;
                continue;
            }
            else
            {
                // Some data found, backfill with new lines if required
                for(int k=0; k < empty_string; k++)
                {
                    textmsg.Append("\n");
                }

                empty_string = 0;
            }

            textmsg.AppendLine(msg_lines[i].ToString());
        }

        textobj.text = textmsg.ToString();
        textobj.enabled = true;
        time_of_message = GetTimeMillis();
            
    }

    public void ClearMessage()
    {
        // First attempt to find the message Box if it exists
        if (messageBox == null)
        {
            return;
            // messageBox = GameObject.Find("MessageBox");
            // if (messageBox == null) { return; }
        }

        Text textobj = messageBox.GetComponent<Text>();

        if (textobj == null)
        { return; }

        textobj.text = "";
        textobj.enabled = false;
    }

    private void UpdateMessage()
    {
        if (messageBox == null)
        {
            return;
        }

        // Get rid of message if period is exceeded
        long time_on_screen = GetTimeMillis() - time_of_message;
        if (time_on_screen > GLOBALS.MESSAGE_DISPLAY_PERIOD)
        {
            ClearMessage();
            return;
        }

        // Otherwise start fading it once the time is at 90%
        Text textobj = messageBox.GetComponent<Text>();

        if (textobj == null)
        { return; }

        float alpha = 1f;

        if( time_on_screen > 0.9* GLOBALS.MESSAGE_DISPLAY_PERIOD)
        {
            alpha = (GLOBALS.MESSAGE_DISPLAY_PERIOD - time_on_screen) / (0.1f * GLOBALS.MESSAGE_DISPLAY_PERIOD);
            if( alpha < 0)
            { alpha = 0;  }
        }
        Color newcolor = new Color(textobj.color.r, textobj.color.g, textobj.color.b, alpha);

        textobj.color = newcolor;
    }

    public void ToggleMessage()
    {
        // Make sure there is a message box
        if (messageBox == null)
        { return; }

        messageBox.transform.parent.gameObject.SetActive(!messageBox.transform.parent.gameObject.activeSelf);

    }


    public void FlagRequest(Dropdown menu)
    {

        switch (menu.value)
        {
            case 0:
                // This is a nothing condition, thus do nothing
                break;
            case 1:
                // Request from server to restart-all
                RestartLevel();
                serverInterrupts["RESTARTALL"] = "Server";
                break;

            default:
                // Set new option and restart level
                GLOBALS.game_option = menu.value - 1;
                RestartLevel();
                serverInterrupts["RESTARTALL"] = "Server";
                break;
        }

        // Reset menu
        menu.value = 0;
        menu.itemText.text = menu.options[0].text;

    }

    public void ServerMenu_RestartLevel()
    {
        if( tournament_mode)
        {
            tournament_sm = TOURNAMENT_STATES.WAITING;
            tournament_force_start = true;
            return;
        }

        RestartLevel();
        AddChat("Field reset by Server.");
    }

    // ************** User Error Message Line *********************
    private GameObject messageLog;  // Master message log line to copy
    class LogLine
    {
        public GameObject TextLine;
        public long time_of_message;
    };

    private List<LogLine> allmessages = new List<LogLine>();

    public void ShowLogMessage(string message, long add_time_s = 0)
    {
        MyUtils.LogMessageToFile(message, false);

        // Make sure there is a messageLog found
        if (!messageLog) { return; }

        // Create a copy
        LogLine newline = new LogLine();
        newline.TextLine = Instantiate(messageLog, messageLog.transform.parent, false);
        newline.time_of_message = GetTimeMillis() + add_time_s * 1000;


        // Set the message
        Text textobj = newline.TextLine.GetComponent<Text>();
        textobj.text = message;
        textobj.enabled = true;
        Canvas.ForceUpdateCanvases();
        float lineheight = textobj.cachedTextGenerator.lineCount * 30f;

        // Go through list and move all lines up one
        foreach (var curr_line in allmessages)
        {
            Vector2 pos = curr_line.TextLine.GetComponent<RectTransform>().anchoredPosition;
            pos.y += lineheight;
            curr_line.TextLine.GetComponent<RectTransform>().anchoredPosition = pos;
        }

        // Add to list
        allmessages.Add(newline);
    }


    private void UpdateLogMessage()
    {
        for (int i = allmessages.Count - 1; i >= 0; i--)
        {
            LogLine curr_line = allmessages[i];

            // Get rid of message if period is exceeded
            long time_on_screen = GetTimeMillis() - curr_line.time_of_message;
            if ((time_on_screen > GLOBALS.MESSAGE_DISPLAY_PERIOD) && ((allmessages.Count-i) > GLOBALS.MESSAGES_TO_KEEP))
            {
                // Remove from list
                allmessages.RemoveAt(i);

                // Destroy object
                Destroy(curr_line.TextLine);
                continue;
            }

            // Otherwise start fading it once the time is at 90%
            Text textobj = curr_line.TextLine.GetComponent<Text>();

            if (textobj == null)
            { return; }

            float alpha = 1f;

            if (time_on_screen > 0.9 * GLOBALS.MESSAGE_DISPLAY_PERIOD)
            {
                alpha = (GLOBALS.MESSAGE_DISPLAY_PERIOD - time_on_screen) / (0.1f * GLOBALS.MESSAGE_DISPLAY_PERIOD);
                if (alpha < 0)
                { alpha = 0; }
            }
            Color newcolor = new Color(textobj.color.r, textobj.color.g, textobj.color.b, alpha);

            textobj.color = newcolor;
        }
    }

    public void ResetChatCounter()
    {
        for (int i = 0; i < allmessages.Count; i++)
        {
            if ((allmessages.Count - i) > GLOBALS.MESSAGES_TO_KEEP) { continue; }


            LogLine curr_line = allmessages[i];
            curr_line.time_of_message = GetTimeMillis();
        }
    }



    // *************************************************************************
    // ************* PROGRAM FLOW CONTROL (Start, Stop, Update, etc..) *********
    // *************************************************************************

    static bool field_load = false;
    static bool elements_load = false;
    static bool scorer_load = false;
    static bool gui_load = false;
    public static bool configuration_done = false;

    // OnEnable: Open log files
    private void OnEnable()
    {
        GLOBALS.LOGS_PATH = ((Application.isEditor) ? "." : Application.persistentDataPath) + Path.DirectorySeparatorChar.ToString() + "logs";

        field_load = false;
        elements_load = false;
        configuration_done = false;
        scorer_load = false;
        gui_load = false;
        GLOBALS.SERVER_MODE = true;
        GLOBALS.topserver = this;
        GLOBALS.TIMER_TOTAL = 120;
        GLOBALS.TIMER_AUTO = 0;
        GLOBALS.game_option = 1;

        UPDATE_DELAY = GLOBALS.SERVER_SEND_UPDATE_DELAY;
        MAX_BYTES = GLOBALS.UDP_MAX_BYTES_IN_MS;

        Physics.autoSimulation = true;

        // Initialize MessageBox
        ShowMessage("");

        // Load field elements
        // OnLevelFinished will trigger when this has finished loading, then we can initialize more data types
        SceneManager.sceneLoaded += OnLevelFinishedLoading;
        SceneManager.LoadScene("Scenes/server_gui", LoadSceneMode.Additive);

        // Load field
        SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/field", LoadSceneMode.Additive);

        // Load field elements
        SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/fieldElements", LoadSceneMode.Additive);
        
        // Load scoring elements
        // SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/Scoring", LoadSceneMode.Additive);
        killme = false;

      

        // Start Server
        // allStrings = new stringPool(); // string pool not used
        thisInstance = this;
        // ServerStart();
        // Server is now started by the menu
        FixAWSProblems();

    }

    private void FixAWSProblems()
    {        // Try to fix problems with running headless in some linux domains
        if (GLOBALS.HEADLESS_MODE)
        {
            // ************** HACK WARNING **************
            // Input Field is giving issues, thus find all and turn it off
            // May be associated with currentInputModule most likelly being in on the AWS server
            InputField[] allfields = GameObject.FindObjectsOfType<InputField>();

            foreach (InputField currfield in allfields)
            {
                currfield.gameObject.SetActive(false);
            }

        }

    }

    // Try to fix issues during late update as well
    private int fix_was_run = 60;
    private void LateUpdate()
    {
        if( GLOBALS.HEADLESS_MODE && serverReady && fix_was_run > 0)
        {
            FixAWSProblems();
            fix_was_run -= 1;
        }
    }

    // OnDisable: close log files
    private void OnDisable()
    {
        SceneManager.sceneLoaded -= OnLevelFinishedLoading;
        GLOBALS.SERVER_MODE = false;
        GLOBALS.topserver = null;
        configuration_done = false;

        // Kill socket asyncrhonous task
        killme = true;

        // Stop the asyncrhonous task
        if (thread_incoming != null)
        {
            thread_incoming.Abort();
        }

        // kill the udpClient
        if (m_udpClient != null)
        {
            m_udpClient.Close();
            m_udpClient.Dispose();
            m_udpClient = null;
        }

        // Close any open files
        MyUtils.CloseScorefiles();


        GLOBALS.client_names.Clear();
        GLOBALS.client_ids.Clear();
    }

    private void Start()
    {
        // Set the camera quality level
        // MyUtils.SetCameraQualityLevel(GameObject.Find("MainCamera"));



        // Clear the client_names tables
        GLOBALS.client_names.Clear();
        GLOBALS.client_ids.Clear();
        clientIDcounter = 1;
    }

    // Not used, but was in multi-threading implementation...
    private static ServerLow thisInstance;

    void FixedUpdate()
    {
        // Used to have thigns here, but it's a bad idea since if it takes a long time, it will slow down physis engine.
        framecount_phys += 1;


        // ***********************************************************
        // ****** TEMP HACK: Make sure all objects are awake
  /*      foreach (GameObject currobj in allFieldElements.Values)
        {
            if (!currobj) { continue; }
            Rigidbody curr_rb = currobj.transform.GetComponent<Rigidbody>();
            if (!curr_rb) { continue; }

            if (curr_rb.IsSleeping()) { curr_rb.WakeUp(); }
        }
  */
        // ***************************************************************
    }

    private long lastSendingTime = 0;
    private string debug_msg1 = "";
    private string debug_msg2 = "";
    private bool full_cycle_done = false;
    private bool second_load = false;

    public long sent_data_count = 0; // bytes sent counter
    public float sent_data_count_time = 0f; // time data count was last reset
    public float data_rate = 0f;
    public int framecount = 0;
    public int framerate = 0;
    public int framecount_phys = 0;
    public int framerate_phys = 0;
    public int update_framerate = 0;
    public int update_framecount = 0;
    private bool tm_holding_update = false;

    private float saved_fixedtimestep = 0;
    private long last_flag_time = 0;
    public long time_last_seen_players = 0;
    float last_physics_time = 0f; // Last time the physics engine updated


    // Update is called once per frame
    void DoAllUpdates()
    {     
        // Process any received messages as soon as they arrive
        allReceivedDataSemaphore.WaitOne();

        try
        {
            if (killme)
            {
                allReceivedData.Clear();
                return;
            }
            // Process any messages
            if (allReceivedData.Count > 0)
            {
                onReceivedData(ref allReceivedData);//do this from the main 
            }
        }
        catch (Exception e)
        {
            MyUtils.LogMessageToFile("Update catch exception: " + e, true);
        }
        finally
        {
            allReceivedData.Clear();
            allReceivedDataSemaphore.Release();
        }

        // Only send updates if enough time passed. In essense we're generating our own fixedupate
        // May consider moving into fixed update at some point
        // Also we will do infrequent processing here (like checking client flags, etc...)
        // During this cycle, we process the main players
        // During the half cycle, we process spectators and serverInfoRequests

        // Get curr_time in milliseconds
        long curr_time = GetTimeMillis();
        long elapsed_time = curr_time - lastSendingTime;

        // Collect some statistics for info
        // Get Ellapsed time
        float time_since_last_update = Time.time - sent_data_count_time;
        framecount += 1;

        // Check to see if physics engine updated since last update
        bool physics_updated = false;
        if( last_physics_time != Time.fixedTime )
        {
            physics_updated = true;
            last_physics_time = Time.fixedTime;
        }


        // Send registration data
        // Moved up to here to ensure registration is periodically updated
        // This function has an internal timer to only periodically update registrations
        SendRegistrationInfo();

        // Refresh timeout counter if players are present
        int player_count = GetPlayerCount() + GetSpectatorCount();

        if( player_count > 0)
        {
            time_last_seen_players = curr_time;
        }

        // If no players are logged in, kill the physics engine and stop here
        if (player_count < 1)
        {
            if (GLOBALS.HEADLESS_MODE)
            {
                Physics.autoSimulation = false;
                // Lets set the frame-rate to a low number
                if (GLOBALS.framerate > 10)
                { Application.targetFrameRate = 10; }

                // Send server info requests
                // 
                ProcessServerUpdateRequest();
                return;
            }
        }
        else if (GLOBALS.HEADLESS_MODE)
        {
            // Return internal update frame-rate to normal
            if (Application.targetFrameRate != GLOBALS.framerate)
            {
                Application.targetFrameRate = GLOBALS.framerate;
            }

            // Turn on physics 
            if (!Physics.autoSimulation)
            {
                Physics.autoSimulation = true;
            }

            // Make sure we initialized saved_fixeddeltatime for later
            if(saved_fixedtimestep == 0 )
            {
                saved_fixedtimestep = Time.fixedDeltaTime;
            }

            // If in tournament mode and holding, slow down the physics engine to save processing power  
            if (tournament_mode && holding_mode && (tournament_sm == TOURNAMENT_STATES.WAITING))
            {
                Time.fixedDeltaTime = 1f / GLOBALS.framerate;
            }
            else
            {
                if (Time.fixedDeltaTime > saved_fixedtimestep)
                {
                    Time.fixedDeltaTime = saved_fixedtimestep;
                }
                else
                {
                    saved_fixedtimestep = Time.fixedDeltaTime;
                }
            }
        }

        // Statistical data averaging info collection
        if (time_since_last_update >= 0.25f)
        {
            data_rate = sent_data_count / time_since_last_update / 1024f;
            sent_data_count_time = Time.time;
            sent_data_count = 0;

            framerate = (int) (((float) framecount) / time_since_last_update);
            framerate_phys = (int) (((float) framecount_phys) / time_since_last_update);
            update_framerate = (int)(((float)update_framecount) / time_since_last_update);
            framecount = 0;
            framecount_phys = 0;
            update_framecount = 0;
            tm_holding_update = true;
        }

   
        ShowMessage("Sent Data Rate (kBytes/s) = " + data_rate.ToString("F0"));
        string message1 = "Frame Rate (Frames/s) = " + String.Format("{0,3}", framerate);
        message1 += ", Network Update Rate (Frames/s) = " + update_framerate.ToString();
        // + " Phys FR (Frames/s) = " + framerate_phys.ToString();
        ShowMessage(message1, 1, false);


        // Check all client connections 
        MonitorConnections();

        // Don't do any new messages if physics engine wasn't update
        if( !physics_updated) { return; }

        // Check full-cycle update
        if (elapsed_time > UPDATE_DELAY)
        {
            update_framecount++;

            // Deal with tournament state machine
            UpdateTournamentMode();

            // Mark that we are updating
            lastSendingTime = curr_time;
            full_cycle_done = true;

            // First lets check for field elements that fell off the board and correct them
            CorrectFieldElements();
            CorrectPlayers();

            // Send flags
            // We don't need to send flags every frame (e.g. not 25frames/s).. to save bandwidth, lets reduce it to a smaller count
            if ((MyUtils.GetTimeMillis() - last_flag_time) > GLOBALS.FLAG_UPDATE_TIME)
            {
                // Update score
                // This only needs to be done ~4 times a second or so..
                UpdateScore();

                SendFlags();
                last_flag_time = MyUtils.GetTimeMillis();
            }

            // Send field and player data: takes the most bandwidth      
            if (tournament_mode && holding_mode && (tournament_sm == TOURNAMENT_STATES.WAITING))
            {
                // If end game, then update robot only
                if (tm_holding_update )
                {
                    SendFieldElements(SendDataFlags.PLAYERS);
                    SendPlayerLocations(SendDataFlags.PLAYERS);                  
                    tm_holding_update = false;
                }
            }
            else
            {
                SendFieldElements(SendDataFlags.PLAYERS);
                SendPlayerLocations(SendDataFlags.PLAYERS);
            }
           
            UpdateMessage();
            UpdateLogMessage();
        }

        // Check half-cycle update
        if ( (elapsed_time > (UPDATE_DELAY / 2.0f)) && full_cycle_done )
        {
            // Do the half cycle
            full_cycle_done = false;

            // Send current players
            sendCurrentPlayers();

            // Send server info requests
            // 
            ProcessServerUpdateRequest();

            // Update spectators
            if (GetSpectatorCount() > 0)
            {
                SendFieldElements(SendDataFlags.SPECTATORS);
                SendPlayerLocations(SendDataFlags.SPECTATORS);              
            }
            

            // **************** DEBUG UPDATE CODE *******************
            
            // Keey it here for packet statistics later

            if (GLOBALS.SHOW_COMPRESSION_TESTCASE)
            {
                // Try the different compression methods
                // ShowMessage(msg_test_case, 5, false);
                System.Diagnostics.Stopwatch timer = new System.Diagnostics.Stopwatch();

                timer.Start();
                byte[] method1 = CompressionHelper.Compress(msg_test_case);
                timer.Stop();
                double time1 = (double)timer.ElapsedTicks / (System.Diagnostics.Stopwatch.Frequency / (1000L * 1000L));


                timer.Reset();
                timer.Start();
                byte[] method3 = BitPacking.Compress(msg_test_case);
                timer.Stop();
                double time3 = (double)timer.ElapsedTicks / (System.Diagnostics.Stopwatch.Frequency / (1000L * 1000L));

                // Combine bit-packing with the other compression
                timer.Reset();
                timer.Start();
                byte[] method4 = BitPacking.Compress(msg_test_case);
                method4 = CompressionHelper.Compress(method4);
                timer.Stop();
                double time4 = (double)timer.ElapsedTicks / (System.Diagnostics.Stopwatch.Frequency / (1000L * 1000L));

                // Combine bit-packing with the zlib
                timer.Reset();
                timer.Start();
                byte[] method5 = BitPacking.Compress(msg_test_case);
                method5 = ZlibStream.CompressBuffer(method5);
                timer.Stop();
                double time5 = (double)timer.ElapsedTicks / (System.Diagnostics.Stopwatch.Frequency / (1000L * 1000L));


                timer.Reset();
                timer.Start();
                byte[] method2 = ZlibStream.CompressString(msg_test_case.ToString());
                timer.Stop();
                double time2 = (double)timer.ElapsedTicks / (System.Diagnostics.Stopwatch.Frequency / (1000L * 1000L));


                // try decompressing it
                byte[] demethod4 = CompressionHelper.Decompress(method4);
                demethod4 = BitPacking.Decompress(demethod4);

                string recovered4 = Encoding.UTF8.GetString(demethod4);



                string msg = "Raw = " + msg_test_case.Length + "\nZip1 = " + method1.Length + " Time=" + time1.ToString("F1") +
                              "\nZip2 = " + method2.Length + " Time=" + time2.ToString("F1") +
                              "\nZip3 = " + method3.Length + " Time=" + time3.ToString("F1") +
                              "\nZip1+Zip3=" + method4.Length + "Time=" + time4.ToString("F1") +
                              "\nZip1+Zip2=" + method5.Length + "Time=" + time5.ToString("F1");


                if (cache_cleared)
                {
                    debug_msg1 = msg;
                }
                else
                {
                    debug_msg2 = msg;
                }

                ShowMessage(debug_msg1 + "\n\n" + debug_msg2, 6, false);
            }
            // *********************************************************
        }

    }

    private bool serverbuild_check = false;
    private bool headless_init = false;

    void Update()
    {
        if (!second_load && gui_load)
        {
            second_load = true;
            // Load scoring elements
            SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/Scoring", LoadSceneMode.Additive);
        }

        // Exit if configuration is not finished
        if(!configuration_done) { return;  }

        if( !headless_init )
        {
            InitHeadlessGraphics();
            headless_init = true;
        }

        // Toggle information screen
        if (!GLOBALS.keyboard_inuse && Input.GetKeyDown(KeyCode.I))
        {
            ToggleMessage();
        }

        // if this is a serial communication method, do it here
        // **** receive any messages ****
        if (GLOBALS.UDP_ALGORITHM == 2)
        {
            DataReceiveSerial();
        }

        // Next do-all-updates - this should have a timer set to limit it's rate
        DoAllUpdates();

        // Update Log Status Files
        MyUtils.DoScoringFiles(serverFlags);


        // *********************
        // For server build, read in stdin
// #if UNITY_SERVER
        if (GLOBALS.HEADLESS_MODE)
        {
            if (!serverbuild_check)
            {
                Console.Out.WriteLine("\nServer console commands enabled.\n");
                serverbuild_check = true;
            }
            ServerSTDINCommands();
        }
// #endif
    }

    private void InitHeadlessGraphics()
    {
        if (GLOBALS.HEADLESS_MODE)
        {
            // Find all cameras and disable
            foreach (Camera curr in Resources.FindObjectsOfTypeAll<Camera>())
            {
                curr.enabled = false;
            }

            // Find all renderers and disable
            foreach (Renderer curr in Resources.FindObjectsOfTypeAll<Renderer>())
            {
                curr.enabled = false;
            }

            // Find all lights and disable
            foreach (Light curr in Resources.FindObjectsOfTypeAll<Light>())
            {
                curr.enabled = false;
            }
        }
        // Find
    }

    // Read STDIN
    private StringBuilder readline = new StringBuilder();

    private void ServerSTDINCommands()
    {
        // Get all the keys
        while (Console.KeyAvailable)
        {
            ConsoleKeyInfo cki = Console.ReadKey(true);
            Console.Write(cki.KeyChar); // Echo the key
            if (cki.Key == ConsoleKey.Enter)
            {
                Console.Write("\n" + ProcessSTDINCommand(readline.ToString()));
                readline.Clear();
            }
            else
            {
                readline.Append(cki.KeyChar);
            }
        }
    }

    private string ProcessSTDINCommand(string readline)
    {
        string outstring = "";
        string[] instrings = readline.Split('=');
        bool success = false;

        // Process the current command
        switch( instrings[0].ToLower())
        {
            case "players":
                // Get the list of players
                outstring += "_BEGIN_\n";

                foreach (Client currclient in allClients.Values)
                {
                    if (currclient.starting_position != "Spectator")
                    {
                        outstring += currclient.name + "\n";
                    }
                }

                outstring += "_END_\n";

                break;

            case "spectators":
                // Get the list of players
                outstring += "_BEGIN_\n";

                foreach (Client currclient in allClients.Values)
                {
                    if (currclient.starting_position == "Spectator")
                    {
                        outstring += currclient.name + "\n";
                    }
                }

                outstring += "_END_\n";

                break;

            case "users":
                // Get the list of players
                outstring += "_BEGIN_\n";

                foreach (Client currclient in allClients.Values)
                {
                    outstring += currclient.id + "=" + currclient.name + "\n";
                }

                outstring += "_END_\n";

                break;


            case "register":

                // Get the list of players
                outstring += "_BEGIN_\n";

                if ( (instrings.Length > 1) && (instrings[1].Length > 4) )
                {
                    GLOBALS.HTTP_REGISTRATION = instrings[1];
                    outstring += "Registration address changed to " + GLOBALS.HTTP_REGISTRATION + "\n";
                }
                else
                {
                    outstring += "Invalid registration address.\n";
                }

                outstring += "_END_\n";

                break;

            case "restart":

                ServerMenu_RestartLevel();

                // Get the list of players
                outstring += "_BEGIN_\n" + 
                             "Restarted game.\n" + 
                             "_END_\n";               
                break;

            case "password":
                outstring += "_BEGIN_\n";

                if(instrings.Length > 1)
                {
                    PASSWORD = instrings[1];
                }

                outstring += "Password = " + PASSWORD + "\n";
                outstring += "_END_\n";

                break;

            case "admin":
                outstring += "_BEGIN_\n";

                if (instrings.Length > 1)
                {
                    ADMIN = instrings[1];
                }

                outstring += "Admin = " + ADMIN + "\n";
                outstring += "_END_\n";

                break;

            case "kickall":
                outstring += "_BEGIN_\n";
                ProcessClientCommands("ADMIN", "/SERVER KICKALL", true);               
                outstring += "Kicked all users\n";
                outstring += "_END_\n";

                break;

            case "kickeveryone":
                outstring += "_BEGIN_\n";
                // Copy over keys so that when we destroy them, we don't get errors from the enumerator
                int[] allkeys = new int[allClients.Keys.Count];

                allClients.Keys.CopyTo(allkeys, 0);

                foreach (int currclient in allkeys)
                {
                    RemoveClient(currclient);
                }
                outstring += "Kicked everyone.\n";
                outstring += "_END_\n";

                break;

            case "kick":

                outstring += "_BEGIN_\n";
                success = false;
                if (instrings.Length > 1)
                {
                    success = ProcessClientCommands("ADMIN", "/SERVER KICK=" + instrings[1], true);
                }

                outstring += (success) ? "Done removing " : "Unable to remove player ";
                outstring += (instrings.Length > 1) ? instrings[1] : "NOT SPECIFIED";
                outstring += "\n_END_\n";
                break;

            case "kickid":

                outstring += "_BEGIN_\n";
                success = false;
                if (instrings.Length > 1)
                {
                    success = ProcessClientCommands("ADMIN", "/SERVER KICKID=" + instrings[1], true);
                }

                outstring += (success) ? "Done removing " : "Unable to remove player ";
                outstring += (instrings.Length > 1) ? instrings[1] : "NOT SPECIFIED";
                outstring += "\n_END_\n";
                break;

            case "gameinfo":

                outstring += "_BEGIN_\n";
                outstring += "State: " + scorer.timerstate.ToString() + "\n";
                outstring += "Time Left: " + scorer.time_total.TotalSeconds.ToString() + "\n";
                outstring += "Timer: " + scorer.GetTimerText() + "\n";
                outstring += "Blue Score: " + scorer.GetBlueScore().ToString() + "\n";
                outstring += "Red Score: " + scorer.GetRedScore().ToString() + "\n";
                outstring += "_END_\n";
                break;

            default:
                // Provide list of commands
                outstring += "_BEGIN_\n" + 
                             "Invalid Command. List of supported commands: \n" +
                             "    PLAYERS - Get list of players (not spectators)\n" +
                             "    SPECTATORS - Get list of spectators\n" +
                             "    USERS - Get list of all users and their ID\n" + 
                             "    KICKALL - Kick all users (but not admin).\n" +
                             "    KICKEVERYONE - Kick everyone including admin.\n" + 
                             "    KICK=<USER> - Kick user by name\n" +
                             "    KICKID=<ID> - Kick user by ID \n" +
                             "    GAMEINFO - Provides information about current game.\n" +
                             "    RESTART - Restart game\n" + 
                             "    PASSWORD[=<password>] - Show game password and/or change it.\n" +
                             "    ADMIN[=<name>] = Show admin name and/or change it.\n" +
                             "    REGISTER=[http://xrcsimulator.org/game/registerserver.php] - Change registration link\n" +
                             "_END_\n";
                break;
        }

        return outstring;
    }


    void OnApplicationQuit()
    {
        killme = true;
    }

    static private bool killme = false;

    Thread thread_incoming;
    public bool serverReady = false;

    public void ServerStart(int port = 1446, string password = "", int spectators = 4, string comment = "", int routerport = 1446, bool register = false, bool tournamentmode = false )
    {
        MyUtils.LogMessageToFile("setting up udp", true);

        max_spectators = spectators;
        tournament_mode = tournamentmode;

        if (comment.Length > 25)
        {
            SERVER_COMMENT = comment.Substring(0, 25);
        }
        else
        {
            SERVER_COMMENT = comment;
        }

        PASSWORD = password;
        ROUTER_PORT = routerport;
        REGISTER_SERVER = register;

        // Create Udp Client
        SetupUdp(port);

        // Start receive system, depending on which algorithm we chose
        if (GLOBALS.UDP_ALGORITHM == 0) // receiveasync method
        {
            m_udpClient.BeginReceive(DataReceived, m_udpClient);
        }
        else if (GLOBALS.UDP_ALGORITHM == 1) // my own parallel thread usingblocking read
        {
            // Begin listening
            thread_incoming = new Thread(new ThreadStart(DataReceive));
            thread_incoming.Start();

            if (thread_incoming.IsAlive) { MyUtils.LogMessageToFile("udp thread is alive.", false); }
            else { MyUtils.LogMessageToFile("udp thread is NOT alive.... yet...", true); }
        }
        else if (GLOBALS.UDP_ALGORITHM == 2) // 100% serial
        {
            // nothing to do
        }

        serverReady = true;

    }

    public void ServerStartHeadless()
    {
        MyUtils.LogMessageToFile("setting up udp", true);

        // Create Udp Client
        SetupUdp(PORT);

        // Start receive system, depending on which algorithm we chose
        if (GLOBALS.UDP_ALGORITHM == 0) // receiveasync method
        {
            m_udpClient.BeginReceive(DataReceived, m_udpClient);
        }
        else if (GLOBALS.UDP_ALGORITHM == 1) // my own parallel thread usingblocking read
        {
            // Begin listening
            thread_incoming = new Thread(new ThreadStart(DataReceive));
            thread_incoming.Start();

            if (thread_incoming.IsAlive) { MyUtils.LogMessageToFile("udp thread is alive.", false); }
            else { MyUtils.LogMessageToFile("udp thread is NOT alive.... yet...", true); }
        }
        else if (GLOBALS.UDP_ALGORITHM == 2) // 100% serial
        {
            // nothing to do
        }

        serverReady = true;
        FixAWSProblems();
    }

    private long tm_timer = 0;
    private bool tournament_force_start = false;
    private bool game_overlay_turned_off = false;

    public void UpdateTournamentMode()
    {
        // If tournament mode is not enabled, skip
        if(!tournament_force_start && (tournament_mode == false) )
        {
            serverFlags["TM_STATE"] = "NO";
            return;
        }

        // Mark if we are in holding mode
        if (holding_mode && (tournament_sm != TOURNAMENT_STATES.RUNNING))
        {
            serverFlags["HOLDING"] = "1";
        }
        else
        {
            if (serverFlags.ContainsKey("HOLDING"))
            {
                serverFlags.Remove("HOLDING");
            }
        }



        // Deal with the state matchine
        switch ( tournament_sm)
        {
            case TOURNAMENT_STATES.WAITING:

                // Remove old TM_ANIM1
                serverFlags.Remove("TM_ANIM2");

                // If waiting for players, check if there are 4 players
                if (!tournament_force_start && (( GetPlayerCount() < GLOBALS.PlayerCount ) || (GetPlayerCount() < GLOBALS.TMinPlayers)))
                {
                    serverFlags["TM_STATE"] = "WAITING";
                    serverFlags["TM_MSG"] =  "Waiting for " + ((GLOBALS.PlayerCount < GLOBALS.TMinPlayers)? GLOBALS.PlayerCount : GLOBALS.TMinPlayers) + " players.\n" + GetReadyPlayersString();
                    break;
                }

                // Wait for all to press ready
                if(tournament_force_start || ((GetReadyPlayers() == GetPlayerCount()) && start_when_ready))
                {
                    serverFlags["TM_STATE"] = "COUNTDOWN";
                    serverFlags["TM_MSG"] = "";
                    if (scorer.GetTimerState() == "RUNNING")
                    {
                        scorer.SetTimerState(Scorekeeper.TimerState.FINISHED);
                    }

                    if (scorer.UseGameStartOverlay())
                    {
                        serverInterrupts["TM_ANIM1"] = "1";
                        scorer.ShowGameStartOverlay(true);
                    }

                    tournament_sm = TOURNAMENT_STATES.HYPE1;
                    tm_timer = GetTimeMillis();
                    
                    break;
                }
              
                serverFlags["TM_STATE"] = "WAITING";
                serverFlags["TM_MSG"] = "Waiting for all players to be ready...\n" + GetReadyPlayersString();
                break;

            case TOURNAMENT_STATES.HYPE1:
                // 4s hype intro followed by 3s countdown
                if (!scorer.UseGameStartOverlay() || (GetTimeMillis() - tm_timer > 4000))
                {
                    serverInterrupts["COUNTDOWN"] = "1"; // Start countdown timer graphics
                    scorer.ScorerReset();
                    scorer.StartCountdown();
                    tournament_sm = TOURNAMENT_STATES.COUNTDOWN;
                    tm_timer = GetTimeMillis();
                }
                break;

            case TOURNAMENT_STATES.COUNTDOWN:
                // If we're 2s in, stop showing the overlay
                // Added new variable to keep track of this instead of adding a new state to ensure backward compatibility.
                // May want to change this in next major release to a new state instead.
                if (GetTimeMillis() - tm_timer > 2000)
                {
                    serverInterrupts["TM_ANIM1"] = "0";
                    serverFlags["TM_ANIM2"] = "0";

                    if (!game_overlay_turned_off)
                    {
                        scorer.ShowGameStartOverlay(false);
                        game_overlay_turned_off = true;
                    }
                }
                else
                {
                    game_overlay_turned_off = false;
                }

                // If we're 3s in, countdown is done.
                if (GetTimeMillis() - tm_timer > 3000 )
                {
                    serverFlags["TM_STATE"] = "RUNNING";
                    serverFlags["TM_MSG"] = "";
                    tournament_sm = TOURNAMENT_STATES.RUNNING;
                    RestartLevel();
                    break;
                }

                serverFlags["TM_STATE"] = "COUNTDOWN";
                // serverFlags["TM_MSG"] = (3f-(current_time - tm_timer)/1000f).ToString("0.0");
                break;

            case TOURNAMENT_STATES.RUNNING:
                serverFlags["TM_STATE"] = "RUNNING";
                serverFlags["TM_MSG"] = "";

                if ( scorer.GetTimerState() == "FINISHED" )
                {
                    serverFlags["TM_STATE"] = "END";
                    serverFlags["TM_MSG"] = "";
                    tournament_sm = TOURNAMENT_STATES.END;
                }
                break;

            case TOURNAMENT_STATES.END:
                serverFlags["TM_STATE"] = "END";

                if( scorer.GetBlueScore() > scorer.GetRedScore())
                {
                    serverFlags["TM_MSG"] = "GAME FINISHED: BLUE WINS! ";
                    serverInterrupts["FIREWORKS"] = "BLUE";
                    scorer.StartFireworks(false);
                      
                }
                else if(scorer.GetBlueScore() < scorer.GetRedScore())
                {
                    serverFlags["TM_MSG"] = "GAME FINISHED: RED WINS!";
                    serverInterrupts["FIREWORKS"] = "RED";
                    scorer.StartFireworks(true);
                }
                else
                {
                    serverFlags["TM_MSG"] = "GAME FINISHED: TIE!";
                }

                tournament_sm = TOURNAMENT_STATES.WAITING;
                tournament_force_start = false;

                break;
        }
    }

    GameObject ChampsObject;
    public void ToggleChampsMode()
    {
        // See if there is a champs section

        ChampsInit[] champsobjects = Resources.FindObjectsOfTypeAll<ChampsInit>();
        if( champsobjects.Length<1) { return; }

        ChampsObject = champsobjects[0].gameObject;

        // toggle Activation
        ChampsObject.SetActive(!ChampsObject.activeSelf);

        // Set text
        TMPro.TextMeshPro[] usertexts = ChampsObject.GetComponentsInChildren<TMPro.TextMeshPro>();

        foreach( TMPro.TextMeshPro currtext in usertexts)
        {
            if( currtext.name.StartsWith("UserText"))
            {
                currtext.text = CHAMPS_TEXT;
            }
        }
        
        if (ChampsObject.activeSelf)
        {
            serverFlags["CHAMPS"] = CHAMPS_TEXT;
        }
        else if(serverFlags.ContainsKey("CHAMPS"))
        {
            serverFlags.Remove("CHAMPS");
        }

    }

    // *************************************************************************
    // ************* HEALTH MONITORING OF OUR CONNECTION ***********************
    // *************************************************************************

    // *************** Check Client Health **********************
    private long time_last_count_check = -1;
    private long time_cache_clear = -1;
    private bool cache_cleared = false;
    private string last_message;

    public void MonitorConnections()
    {
        long current_time = GetTimeMillis();


        // Show message count info
        if (current_time - time_last_count_check > GLOBALS.SERVER_MESSAGE_COUNT_TIME)
        {
           
            string message = "<b> List of Connections on Port " + ((remoteEP!=null) ? remoteEP.Port.ToString() : "") + " (Press i to toggle): </b> ";

            int curr_line = 0;


            foreach (Client curr in allClients.Values)
            {
                if( curr.status_line == null ) { continue;  }

                int rate = (int)((float)curr.message_count / (float)(current_time - time_last_count_check) * 1000f);
                curr.status_line.transform.Find("DATA").GetComponent<Text>().text = rate.ToString();
                curr.status_line.transform.Find("READY").GetComponent<Text>().text = "";

                if( curr.flags.ContainsKey("PL"))
                {
                    curr.status_line.transform.Find("PL").GetComponent<Text>().text = curr.flags["PL"];
                }

                if ( tournament_sm == TOURNAMENT_STATES.WAITING)
                {
                    string readyvalue = "";
                    curr.flags.TryGetValue("READY", out readyvalue);
                    curr.status_line.transform.Find("READY").GetComponent<Text>().text = readyvalue;
                }

                curr.message_count = 0;

                // Set player's parent and location
                if (messageBox)
                {
                    curr.status_line.transform.SetParent(messageBox.transform.parent.transform);
                    curr.status_line.transform.localPosition = new Vector3(24f, -100f - curr_line*30f, -1f);
                    curr.status_line.transform.localScale = new Vector3(1f, 1f, 1f);
                }
                curr_line += 1;
            }

            if (GLOBALS.ENABLE_UDP_STATS && netmonitor != null)
            {
                foreach(KeyValuePair<string, int> curr in netmonitor)
                {
                    message += "\n" + curr.Key + "=" + ((float) curr.Value / (float)(current_time - time_last_count_check) * 1000f);
                }

                netmonitor.Clear();

                message += "\nCached Count = " + items_cached.ToString();
                items_cached = 0;
            }
       
            time_last_count_check = current_time;
            ShowMessage(message,2,false);
            last_message = message;
        }
        else
        {
            ShowMessage(last_message, 2, false);
        }

        // Drop players that have exceeded timeout
        List<int> remove_clients = new List<int>();

        foreach (Client curr in allClients.Values)
        {
            // Drop clients if their internet communication timed 
            if (current_time- curr.time_last_message >= GLOBALS.SERVER_DISCONNECT_TIMEOUT)
            {
                remove_clients.Add(curr.id);
                MyUtils.LogMessageToFile("Lost communication with " + curr.name, false);   
            }

            // If admin, don't drop (no, still need to drop, just not based on keyboard timeout)
            // if ((ADMIN == curr.name) || (curr.isAdmin) ) { continue; }

            // If no robot, don't do anything
            if( !curr.robot ) { continue;  }

            // If this robot didn't expire timing yet, don't do anything
            if((MyUtils.GetTimeMillis() - curr.robot.time_last_button_activitiy) < 1000 * GLOBALS.CLIENT_ROBOT_INACTIVITY_TIMEOUT) { continue; }
            
            // Expired robots: drop if it's a player or if game was innactive
            // Drop non-active non-admin robots or spectators with a robot avatar that are inactive
            if ( !curr.robot.isSpectator || ((MyUtils.GetTimeMillis() - time_last_seen_players) > 1000 * GLOBALS.CLIENT_ROBOT_INACTIVITY_TIMEOUT))
            {
                remove_clients.Add(curr.id);
                MyUtils.LogMessageToFile("Innactivity timeout for " + curr.name, false);
            }

        }
        
        // Remvoe the clients marked for deletion
        foreach ( int currid in remove_clients)
        {
            RemoveClient(currid);
        }

        // Clear cache if required
        if (current_time - time_cache_clear > GLOBALS.SERVER_CACHE_REFRESH)
        {
            time_cache_clear = current_time;

            foreach (CacheString currval in netcache.Values)
            {
                currval.Clear();
            }

            foreach (CacheString currval in netcache_spectators.Values)
            {
                currval.Clear();
            }

            cache_cleared = true;
        }
        else
        { cache_cleared = false; }

    }

    public static long GetTimeMillis()
    {
        return MyUtils.GetTimeMillis();
    }

    public int GetPlayerCount()
    {
        int players = 0;

        foreach( int i in allClients.Keys )
        {
            if( allClients[i].starting_position != "Spectator")
            {
                players++;
            }
        }

        return players;
    }

    public int GetSpectatorCount()
    {
        int players = 0;

        foreach (int i in allClients.Keys)
        {
            if (allClients[i].starting_position == "Spectator")
            {
                players++;
            }
        }

        return players;
    }

    // Returns the number of Admins as spectators
    private int GetAdminSpecCount()
    {
        int players = 0;

        foreach (int i in allClients.Keys)
        {
            if ((allClients[i].starting_position == "Spectator") && ((allClients[i].name == ADMIN) && (ADMIN.Length > 0) || allClients[i].isAdmin))
            {
                players++;
            }
        }

        return players;
    }

    private int GetReadyPlayers()
    {
        int players = 0;
        string value = "FALSE";

        foreach (int i in allClients.Keys)
        {
            if (allClients[i].starting_position != "Spectator")
            {

                if( allClients[i].flags.TryGetValue("READY", out value) && ((value == "YES") || (value == "READY")))
                { 
                    players++;
                }
            }
        }

        return players;
    }

    private string GetReadyPlayersString()
    {
        string players_ready = "Waiting for:";
        string value = "FALSE";

        foreach (int i in allClients.Keys)
        {
            if (allClients[i].starting_position != "Spectator")
            {
                if (allClients[i].flags.TryGetValue("READY", out value) && !((value == "YES") || (value == "READY")))
                {
                    players_ready = players_ready + " " +  allClients[i].name;
                }
            }
        }

        return players_ready;
    }
    // *************************************************************************
    // ************* Outgoing Messages Generation        ***********************
    // *************************************************************************

    private static int message_id = 1;  // Incrementing ID to identify older vs newer packets - generic

    private static int message_id_tracked_players = 1;  // Incrementing ID to identify older vs newer packets - for player locations and field elements only
    private static int message_id_tracked_other = 1;  // Incrementing ID to identify older vs newer packets - for player locations and field elements only



    // List of all variables/flags we want to send to clients or keep track of
    private Dictionary<string, string> serverFlags = new Dictionary<string, string>();
    private Dictionary<string, string> serverInterrupts = new Dictionary<string, string>();


    // Sends the player's avatar locations (including each component).
    // Packets will be split to keep under 4k size
    private void SendPlayerLocations(SendDataFlags who = SendDataFlags.PLAYERS)
    {
        // NOTE: THIS functions can fail with message_id if you alternate between players, spectators and everypone.
        // Use either players + spectators, or everyone, but do not mix everyone with spectators.

        // Do not allow who to be everyone
        if (who == SendDataFlags.EVERYONE) { who = SendDataFlags.PLAYERS; }
        Dictionary<string, CacheString> thecache;
        ref int message_id = ref message_id_tracked_players;

        // Make sure we use the correct cache
        if (who == SendDataFlags.PLAYERS)
        {
            thecache = netcache;

        }
        else
        {
            thecache = netcache_spectators;
            message_id = ref message_id_tracked_other;
        }
           

        netmonitor_header = GLOBALS.HEADER_PLAYER_POS;

        // Since the player robot can be large, we will send each player individually
        // Furthermore, we will send multiple packets if required (if the object count is ultra high)
        foreach (Client currClient in allClients.Values)
        {
            // Only send client information if an avator for the client exists and is instantiated and it's not spectator
            if (currClient.avatar == null) { continue; }
            if (currClient.starting_position == "Spectator") { continue; }
            if (currClient.avatar.scene.name == null) { continue; }
            if (!currClient.avatar.activeInHierarchy) { continue; }

            
            // Start the current message
            string m_player_header = GLOBALS.HEADER_PLAYER_POS + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1;

            // Add the player information
            // Dropped robot model name: it stops us from using bitpacking compression and not used in client anyways. Will send length instead.
            m_player_header += currClient.id.ToString() + GLOBALS.SEPARATOR2 + currClient.robotmodel.Length.ToString();

            // Start the new message
            string m = m_player_header;

            int j = 0;
            StringBuilder currkey = new StringBuilder();
            StringBuilder currval = new StringBuilder();

            // Add the root/parent info

            currkey.Append("SPL").Append((int) who).Append(currClient.id.ToString() + GLOBALS.SEPARATOR1 + (-1).ToString());          
            currval.Append(currClient.avatar.transform.position.x.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.position.y.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.position.z.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.eulerAngles.x.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.eulerAngles.y.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.eulerAngles.z.ToString("0.####") );

            bool skiproot = false;

            // If root of object didn't change, then skip sending the root, otherwise send it out
            if (thecache.ContainsKey(currkey.ToString()))
            {
                // It has a StringBuilder with our value, thus now compare them
                if (thecache[currkey.ToString()].msg.ToString() == currval.ToString())
                {
                    items_cached += 1;

                    if ((++thecache[currkey.ToString()].count) > 2)
                    {
                        skiproot = true;
                        thecache[currkey.ToString()].count = 0;
                    }

                }
                else
                {
                    // Didn't match, so lets remember for next Time
                    thecache[currkey.ToString()].msg.Clear().Append(currval.ToString());
                    thecache[currkey.ToString()].count = 0;
                }
            }
            else
            {
                // Create new key-value pair
                thecache[currkey.ToString()] = new CacheString(currval.ToString());
            }

            // Add to message 
            // The id ==-1 indicates this is the root transform
            if( !skiproot)
            {
                m += GLOBALS.SEPARATOR2 + (-1).ToString() + GLOBALS.SEPARATOR2 + currval.ToString();
            }

            //go through all the children of the robot
            // Want to do Body first since it's used as reference for some bandwidth reduction

            bool Body_done = false;
            bool bandwidth_helper_found = false;
            int priority = 1; // Bandwidth priority

            // Do each child only once, remember it in this hash
            Dictionary<int, bool> children_complete = new Dictionary<int, bool>();

            for (int i = 0; i <= currClient.avatar.transform.childCount; i++)
            {
                // This is our end-checking - do it here so that we can easily "continue" if applicable
                // If we reached the end of the list:
                if( i == currClient.avatar.transform.childCount) 
                {
                    // If we processed a bandwidth helper, then start over with a higher priority
                    if (bandwidth_helper_found)
                    {
                        i = 0;
                        bandwidth_helper_found = false;
                        priority += 1;
                    }
                    else
                    {
                        // Otherwise we finished it all
                        break; // Done all our checkings
                    }
                }

                // If we already processed this item, then skip it
                if (children_complete.ContainsKey(i))
                { continue; }

                // We need to do body first, search for it. If we already have done it then skip this one
                if ((currClient.avatar.transform.GetChild(i).name == "Body") && Body_done ||
                    (currClient.avatar.transform.GetChild(i).name != "Body") && !Body_done) 
                { continue; }


                // Change to local positions and rotation
                currkey.Clear();
                currkey.Append("SPL").Append(currClient.id.ToString() + GLOBALS.SEPARATOR1 + i.ToString());
                currval.Clear();

                // If it has a bandwidthhelper class, then use it's output
                if (currClient.avatar.transform.GetChild(i).GetComponent<BandwidthHelper>())
                {
                    bandwidth_helper_found = true;

                    // Only do the current priority
                    if (currClient.avatar.transform.GetChild(i).GetComponent<BandwidthHelper>().priority == priority)
                    {
                        currval.Append(currClient.avatar.transform.GetChild(i).GetComponent<BandwidthHelper>().Get((int)who));
                        children_complete[i] = true;
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    currval.Append(currClient.avatar.transform.GetChild(i).transform.localPosition.x.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.GetChild(i).transform.localPosition.y.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.GetChild(i).transform.localPosition.z.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.GetChild(i).transform.localEulerAngles.x.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.GetChild(i).transform.localEulerAngles.y.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currClient.avatar.transform.GetChild(i).transform.localEulerAngles.z.ToString("0.####"));
                    children_complete[i] = true;
                }

                // If object didn't change, then skip it, otherwise send it out
                // But let it stay the same for at least 2 cycles for movement extrapolation
                // Also we should not really be doing this if BandwidthHelper exists
                if (thecache.ContainsKey(currkey.ToString()))
                {
                    // It has a StringBuilder with our value, thus now compare them
                    if (thecache[currkey.ToString()].ToString() == currval.ToString())
                    {
                        if (++thecache[currkey.ToString()].count > 2)
                        {
                            items_cached += 1;
                            continue;
                        }
                    }
                    else
                    {
                        // Didn't match, so lets remember for next Time
                        thecache[currkey.ToString()].msg.Clear().Append(currval.ToString());
                        thecache[currkey.ToString()].count = 0;
                    }
                }
                else
                {
                    // Create new key-value pair
                    thecache[currkey.ToString()] = new CacheString(currval.ToString());
                }


                // Add it to our message
                m += GLOBALS.SEPARATOR2 + i.ToString() + GLOBALS.SEPARATOR2 + currval.ToString();


                // Need to limit data packets to 4k. Assuming 12 bytes per float we should allow ~40 count.
                // Will play it safe and only send half of that
                // Zip is making this packet size reasonable, thus will not split packet up if zip enabled
                if ((GLOBALS.PACKET_COMPRESSION == 0) && (j > 15))
                {
                    // Send the message
                    SendUdp(m, null, (GLOBALS.PACKET_COMPRESSION > 0) ? 3 : 0, who);
                    message_id++;
                    m_player_header = GLOBALS.HEADER_PLAYER_POS + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1;
                    m_player_header += currClient.id.ToString() + GLOBALS.SEPARATOR2 + currClient.robotmodel.Length.ToString();

                    // Re-initialize 
                    m = m_player_header;
                    j = 0;
                }
                else
                { j++; }

                // If this was the body, then reset i back to the beggining
                if (!Body_done)
                {
                    Body_done = true;
                    i = -1;
                }
            }

            // Send the packet (if data present)
            if (j != 0) { 
                SendUdp(m, null, (GLOBALS.PACKET_COMPRESSION > 0) ? 3 : 0, who);
                message_id++;
            }
        }

        netmonitor_header = null;
    }

    // Send player list and IDs to clients
    // This makes sure message is sent with 1 UDP packet only, thus can be relied on to
    // identify missing players (e.g. ones not present in the list)
    public void sendCurrentPlayers()
    {
        // Increment message id
        message_id++;

        // Go through each player and send the named plus id + Robot Model + skins + progress bar value
        string msg = GLOBALS.HEADER_PLAYERS + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1;
        bool at_least_one_present = false;

        foreach (Client currclient in allClients.Values)
        {
            // Do not add anything if it's just a spectator
            if ((currclient.robot == null) ||  (currclient.starting_position == "Spectator"))
            {
                msg += currclient.name + GLOBALS.SEPARATOR2 + currclient.id + GLOBALS.SEPARATOR2 + currclient.robotmodel + GLOBALS.SEPARATOR2 + currclient.starting_position + GLOBALS.SEPARATOR2 + "0" + GLOBALS.SEPARATOR1;
            }
            else
            {
                msg += currclient.name + GLOBALS.SEPARATOR2 + currclient.id + GLOBALS.SEPARATOR2 + currclient.robotmodel + GLOBALS.SEPARATOR2 + currclient.starting_position + GLOBALS.SEPARATOR2 + currclient.robot.GetStates() + GLOBALS.SEPARATOR2 + currclient.skins + GLOBALS.SEPARATOR2 + currclient.robotskins + GLOBALS.SEPARATOR1;
            }
                at_least_one_present = true;
        }

        if (at_least_one_present) { SendUdp(msg); }
    }

    // Adds a flag (from outside of Server class) to everyone (by default) or specific user
    public void AddFlag(string key, string value, int id = -1)
    {
        if( id>0)
        {
            if( !allClients.ContainsKey(id)) { return; }

            allClients[id].serverFlags[key] = value;
            return;
        }

        serverFlags[key] = value;
    }

    // Adds a interrupt (from outside of Server class) to everyone (by default) 
    public void AddInterrupt(string key, string value)
    {
        serverInterrupts[key] = value;
    }

    public void ClearFlag(string key, int id = -1)
    {
        if (id > 0)
        {
            if (!allClients.ContainsKey(id)) { return; }

            allClients[id].serverFlags.Remove(key);
            return;
        }

        serverFlags.Remove(key);
    }

    private void SendFlags()
    {
        // Add any server flags that need to be processed at time of being sent
        serverFlags.Remove("ADMIN");
        if (ADMIN.Length > 0)
        {
            if (GLOBALS.client_ids.ContainsKey(ADMIN))
            {
                serverFlags["ADMIN"] = GLOBALS.client_ids[ADMIN].ToString();
            }
            else
            {
                serverFlags["ADMIN"] = "-9";
            }
        }

        // If no flags, send blank
        if( (serverFlags.Count < 1) && (serverInterrupts.Count < 1) )
        {
            serverFlags["NotUsed"] = "1";
        }

        string m = "";

        // Assumed there are less than 4k bytes of data to be sent here, otherwise issues may arise 
        foreach (String currkey in serverFlags.Keys)
        {
            if( m.Length > 1) { m += GLOBALS.SEPARATOR2;  }
            m +=  currkey + GLOBALS.SEPARATOR2 + serverFlags[currkey];
        }

        foreach (String currkey in serverInterrupts.Keys)
        {
            if (m.Length > 1) { m += GLOBALS.SEPARATOR2; }
            m += currkey + GLOBALS.SEPARATOR2 + serverInterrupts[currkey];
        }

        // Add client specific flags
        foreach ( int currid in allClients.Keys)
        {
            string m2 = m;

            // Add client specific flags (ones only sent to a client and not broadcast)
            foreach (String currkey in allClients[currid].serverFlags.Keys)
            {
                if (m2.Length > 1) { m2 += GLOBALS.SEPARATOR2; }
                m2 += currkey + GLOBALS.SEPARATOR2 + allClients[currid].serverFlags[currkey];
            }

            // Add client specific interrupts (ones only sent to a client and not broadcast)
            foreach (String currkey in allClients[currid].serverInterrupts.Keys)
            {
                if (m2.Length > 1) { m2 += GLOBALS.SEPARATOR2; }
                m2 += currkey + GLOBALS.SEPARATOR2 + allClients[currid].serverInterrupts[currkey];
            }

            allClients[currid].serverInterrupts.Clear();

            SendFlagsUDPTracked(m2, 1, SendDataFlags.TARGETID, currid);
        }

        // Clear flags that don't use hand-shaking and we want cleared
        serverInterrupts.Clear();
    }

    // Sends all field element positions
    // Packets are split to keep them under 4k size, however all packets will have
    // same message_id (aging)
    int items_cached = 0;

    string msg_test_case = "";

    // Send Field element data
    private void SendFieldElements(SendDataFlags who = SendDataFlags.PLAYERS )
    {
        // NOTE: THIS functions can fail with message_id if you alternate between players, spectators and everypone.
        // Use either players + spectators, or everyone, but do not mix everyone with spectators.

        msg_test_case = "";

        SortedDictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;
        Dictionary<string, CacheString> thecache;

        if (who == SendDataFlags.EVERYONE) { who = SendDataFlags.PLAYERS; }

        ref int message_id = ref message_id_tracked_players;

        // Make sure we use the correct cache
        if (who == SendDataFlags.PLAYERS)
        {
            thecache = netcache;
        }
        else
        {
            thecache = netcache_spectators;
            message_id = ref message_id_tracked_other;
        }

        // Send message id twice to guarantee no corruption
        string message = GLOBALS.HEADER_FIELDELEMENTS + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1;


        // Send only ~15 elements at a time to keep UDP packet under 4k
        int j = 0;
        StringBuilder currkey = new StringBuilder();
        StringBuilder currval = new StringBuilder();

        foreach (GameObject currobj in allelements)
        {
            Transform currtransform = currobj.GetComponent<Transform>();
            gameElement currelement = currobj.GetComponent<gameElement>();

            // If this field is off, don't send it
            if( currelement.type == ElementType.Off) { continue;  }

            // Check Cache
            // Format numbers so numerical noise doesn't prevent caching
            // Rotation is overkill at 2 decimal places, we'll try 1 decimal
            // Position needs to be at 3 decimal places

            currkey.Clear();
            currkey.Append("SFE").Append((int) who).Append(currelement.id.ToString());
            currval.Clear();
            currval.Append(
                // Type must be integer for bitpacking compression
                ((int)currelement.type).ToString() + GLOBALS.SEPARATOR2);

            // If we have a bandwidth helper, then use it
            if (currtransform.GetComponent<BandwidthHelper>())
            {
                currval.Append(currtransform.GetComponent<BandwidthHelper>().Get((int) who));
            }
            else
            {
                currval.Append( currtransform.position.x.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currtransform.position.y.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currtransform.position.z.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currtransform.rotation.eulerAngles.x.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currtransform.rotation.eulerAngles.y.ToString("0.####") + GLOBALS.SEPARATOR2 +
                                currtransform.rotation.eulerAngles.z.ToString("0.####"));
            }

            // If object didn't change, than skip it, otherwise send it out
            if( thecache.ContainsKey(currkey.ToString()) ) 
            {
                // It has a StringBuilder with our value, thus now compare them
                if (thecache[currkey.ToString()].ToString() == currval.ToString())
                {
                    if (++thecache[currkey.ToString()].count > 2)
                    {
                        items_cached += 1;
                        continue;
                    }
                }

                // Didn't match, so lets remember for next Time
                thecache[currkey.ToString()].msg.Clear().Append(currval.ToString());
                thecache[currkey.ToString()].count = 0;
            }
            else
            {
                // Create new key-value pair
                thecache[currkey.ToString()] = new CacheString(currval.ToString());
            }

            // Sending element ID, element type, transform xyz, rotation xyzw
            message += currelement.id.ToString() + GLOBALS.SEPARATOR2 + currval.ToString() + GLOBALS.SEPARATOR1;

             if (j >= 15)
             {
                j = 0;
                message += GLOBALS.SEPARATOR1;
                SendUdp(message, null, (GLOBALS.PACKET_COMPRESSION > 0) ? 3 : 0, who); // Force bitpacking if enabled
                message_id++;
                msg_test_case += message;
                message = GLOBALS.HEADER_FIELDELEMENTS + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1;
           
            }
            else
            { j++; }
        }

        // Send last message if it exists
        message += GLOBALS.SEPARATOR1;
        if (j != 0)
        {
            SendUdp(message, null, (GLOBALS.PACKET_COMPRESSION > 0) ? 3 : 0, who);// Force bitpacking if enabled
            message_id++;
            msg_test_case += message;
        }
    }

    private void ProcessServerUpdateRequest()
    {
        for (int thisMessage = 0; thisMessage < serverInfoRequests.Count; thisMessage++)
        {
            Message currMessage = serverInfoRequests[thisMessage];
            byte[] message = currMessage.data;

            List<byte[]> extracted_data = new List<byte[]>();

            // Check if this is a server info request
            // <PASSCODE FOR SERVERINFO>|<DATA TO SEND BACK>|

            // Split the message up into the top components
            MyUtils.ExtractMessageHeader(message, extracted_data);

            // Already Verified this is a server info request, thus just send it
            // <SERVERINFO>|<SERVER VARIABLES>|<ORIGINAL [1] data>
            SendServerInfoUdp(
                        "GAME"      + GLOBALS.SEPARATOR2 + GLOBALS.GAME                 + GLOBALS.SEPARATOR2 +
                        "PLAYERS"   + GLOBALS.SEPARATOR2 + GetPlayerCount().ToString()  + GLOBALS.SEPARATOR2 +
                        "MAXPLAYERS"+ GLOBALS.SEPARATOR2 + GLOBALS.PlayerCount          + GLOBALS.SEPARATOR2 +
                        "VERSION"   + GLOBALS.SEPARATOR2 + Application.version          + GLOBALS.SEPARATOR2 +
                        "COMMENT"  + GLOBALS.SEPARATOR2 + SERVER_COMMENT               + GLOBALS.SEPARATOR2 +
                        "PASSWORD"  + GLOBALS.SEPARATOR2 + ((PASSWORD.Length > 0) ? "1" : "0") +
                        
                        GLOBALS.SEPARATOR1 + Encoding.UTF8.GetString(extracted_data[1]) + GLOBALS.SEPARATOR1
                        , currMessage.endpoint);
        }

        // Empty the request queue
        serverInfoRequests.Clear();
    }

    long registration_time = -1;
    private void SendRegistrationInfo()
    {
        // If the time period expired, send the registration info
        if( REGISTER_SERVER &&  (MyUtils.GetTimeMillis() - registration_time > GLOBALS.registration_update_rate))
        {
            // Add some randomization to the registraion time for multiple server updated
            // By default it updates it every 30s, here we add a 1s +/- variation
            registration_time = MyUtils.GetTimeMillis() + rng.Next(-1000, 1000);
            StopAllCoroutines();
            StartCoroutine(SendRegistrationCoroutine());
        }
    }

    IEnumerator SendRegistrationCoroutine()
    {
        //WWWForm form = new WWWForm();

        // Add data
        //form.AddField("PORT", ROUTER_PORT);
        //form.AddField("VERSION", Application.version);
        //form.AddField("GAME", GLOBALS.GAME);
        //form.AddField("PLAYERS", GetPlayerCount());
        //form.AddField("MAXPLAYERS", 4);
        //form.AddField("SPECTATORS", GetSpectatorCount());
        //form.AddField("MAXSPECTATORS", max_spectators);
        //form.AddField("PASSWORD", (PASSWORD.Length > 0) ? 1 : 0);
        //form.AddField("COMMENT", SERVER_COMMENT);

        string putdata = 
            "PORT" + GLOBALS.SEPARATOR1 + ROUTER_PORT + GLOBALS.SEPARATOR2 +
            "VERSION" + GLOBALS.SEPARATOR1 + Application.version + GLOBALS.SEPARATOR2 +
            "GAME" + GLOBALS.SEPARATOR1 + GLOBALS.GAME + GLOBALS.SEPARATOR2 +
            "PLAYERS" + GLOBALS.SEPARATOR1 + GetPlayerCount() + GLOBALS.SEPARATOR2 +
            "MAXPLAYERS" + GLOBALS.SEPARATOR1 + GLOBALS.PlayerCount + GLOBALS.SEPARATOR2 +
            "SPECTATORS" + GLOBALS.SEPARATOR1 + GetSpectatorCount() + GLOBALS.SEPARATOR2 +
            "MAXSPECTATORS" + GLOBALS.SEPARATOR1 + max_spectators + GLOBALS.SEPARATOR2 +
            "PASSWORD" + GLOBALS.SEPARATOR1 + ((PASSWORD.Length > 0) ? 1 : 0) + GLOBALS.SEPARATOR2 +
            "COMMENT" + GLOBALS.SEPARATOR1 + SERVER_COMMENT;

        // Start the web request
        // UnityWebRequest www = UnityWebRequest.Post(GLOBALS.HTTP_REGISTRATION, postdata);
        using (UnityWebRequest www = UnityWebRequest.Put(GLOBALS.HTTP_REGISTRATION, putdata))
        {
            yield return www.SendWebRequest();
        }
    }

    // Server generated chat message: add to flags and display it.
    public void SendChat(string msg)
    {
        // Process chat meesages for commands
        if (msg.StartsWith("/SET "))
        {
            string substring = msg.Substring(5);
            string[] split = substring.Split('=');

            if (split[0] == "OUTPUT_SCORE_FILES")
            {
                GLOBALS.OUTPUT_SCORING_FILES = true;
                
                if (split.Length > 1)
                {
                    MyUtils.status_file_dir = split[1];                  
                }
                Console.Out.WriteLine("/SET: OUTPUT Score files turned on to dir " + MyUtils.status_file_dir);

            }
            else if (split[0] == "PASSWORD")
            {
                if (split.Length > 1)
                {
                    PASSWORD = split[1];
                }
                else
                {
                    PASSWORD = "";
                }
                
                Console.Out.WriteLine("/SET: PASSWORD = " + PASSWORD);
                return;
            }
            else if(split[0] == "TOURNAMENT")
            {
                tournament_force_start = true;
                Console.Out.WriteLine("/SET: Tournament mode enabled.");
            }
            else if(split[0] == "CODE")
            {
                string[] data = split[1].Split(',');
                if( data.Length >= 4 )
                {
                    string code = MyUtils.CreateMD5(int.Parse(data[0]), int.Parse(data[1]), data[2], int.Parse(data[3]));
                    ShowLogMessage("RED=" + int.Parse(data[0]) + ",BLUE=" + int.Parse(data[1]) + ",Position=" + data[2] + ", Code=" + int.Parse(data[3]) + ",CODE=" + code,10);
                }
            }
            else if(split[0] == "FIND")
            {
                if( split.Length >= 3)
                {
                    int red_score, blue_score;
                    string position;

                    string matched = MyUtils.FindMD5(split[1], int.Parse(split[2]), 500, out red_score, out blue_score, out position);
                    ShowLogMessage("Red=" + red_score + ", Blue=" + blue_score + ", POS=" + position + ", CODE=" + matched,10);
                }
               
            }

        }
        else
        {
            String chatmsg = "SERVER: " + msg;
            ShowLogMessage(chatmsg);
            AddChat(chatmsg);
        }
    }

    // Add the chat message to the current one
    public void AddChat(String msg, int id=-1)
    {
        Dictionary<string, string> interrupts;

        if( id<=0)
        {
            interrupts = serverInterrupts;
        }
        else
        {
            interrupts = allClients[id].serverInterrupts;
        }

        // MyUtils.LogMessageToFile(msg, true);
        if (interrupts.ContainsKey("CHAT"))
        {
            interrupts["CHAT"] += "\n" + msg;
        }
        else
        {
            interrupts["CHAT"] = msg;
        }          
    }

    // Changes the server's settings if applicable
    public void ChangeGameSettings(string newsettings)
    {
        // Let everyone know about the game settings so their scorers
        // can do client-side calculations if required
        serverFlags["GameSettings"] = newsettings;
    }

    // *************************************************************************
    // ************* INCOMING MESSAGE PROCESSING         ***********************
    // *************************************************************************
    // *************************************************************************


    public class Message
    {
        public byte[] data;
        public IPEndPoint endpoint;
    }

    private List<Message> allReceivedData = new List<Message>();
    private List<Message> serverInfoRequests = new List<Message>();
    private Semaphore allReceivedDataSemaphore = new Semaphore(1, 1);

    // *************** Client List: observers and players ********************
    public class Client
    {
        public IPEndPoint endpoint;
        public int id = -1;
        public string name = "";
        public long time_last_message = -1;
        public int message_count = 0;
        public int client_confirmed_this_msg_id = -1;  // the last confirmed client received UDP packet from server
        public int server_confirmed_this_msg_id = 1; // The last confirmed server received UDP packet from client
        public GameObject avatar;   //this will hold our actual robot object for each client
        public RobotInterface3D robot; // The robot interface module 
        public string starting_position = "";
        public bool isAdmin = false;
        public float reset_release = 0f;
        public Transform starting_pos = null;
        public Quaternion camera_rotation;
        public string robotmodel = "";
        public string robotskins = "";
        public string skins = "0";
        public string DriveTrain = "Tank";
        public float speed = 0f;
        public float acceleration = 0f;
        public float weight = 0f;
        public float turning_scaler = 0f;
        public bool fieldcentric = false;
        public bool activebreaking = false;
        public bool tankcontrol = false;
        public Dictionary<string, string> flags;  // Hash of all variable/values client sent over
        public Dictionary<int, string> server_sent_packets; // Cache of outgoing Flags
        public int curr_tracked_packet = 1;
        public GameObject status_line;           // Status line that contains information about this client
        public long lastrestart = 0;
        public Dictionary<string, string> serverFlags; // Saves client specific server flags
        public Dictionary<string, string> serverInterrupts; // Saves client specific server interrupts
    }


    // List of all our clients, entered by id
    private Dictionary<int, Client> allClients = new Dictionary<int, Client>();

    // Process received message queue
    public void onReceivedData(ref List<Message> messages)
    {
        for (int thisMessage = 0; thisMessage < messages.Count; thisMessage++)
        {
            Message currMessage = messages[thisMessage];
            byte[] message = currMessage.data;

            List<byte[]> extracted_data = new List<byte[]>();

            // Check if this is a server info request
            // <PASSCODE FOR SERVERINFO>|<DATA TO SEND BACK>|


            // Split the message up into the top components
            MyUtils.ExtractMessageHeader(message, extracted_data);

            // Initial message split into following:
            // <PASSCODE>|<COMPRESSION#>|<CLIENT ID>|<DATA LENGTH>|<DATA>
            // <REQUEST_SERVERINOF>|<DATA TO SEND BACK>|

            // ************ VERIFY PASSCODE **********************

            if (!Encoding.UTF8.GetString(extracted_data[0]).Equals(GLOBALS.PASSCODE))
            {
                if (Encoding.UTF8.GetString(extracted_data[0]).Equals(GLOBALS.GETSERVERINFO) && extracted_data.Count >= 2)
                {
                    // Save server requet processing for the half-cycle transmissions
                    serverInfoRequests.Add(currMessage);
                    continue;
                }

                MyUtils.LogMessageToFile("OnReceiveData passcode failed. " + message, true);
                continue;
            }//needs to match passcode

            // ************ VERIFY AT LEAST >=5 items **********************
            if (extracted_data.Count < 5)
            {
                MyUtils.LogMessageToFile("OnReceiveData split failed on _. " + message, true);
                continue;
            }

            // ************ VERIFY LENGTH **********************
            int length = 0;
            if (!int.TryParse(Encoding.UTF8.GetString(extracted_data[3]), out length))
            {
                MyUtils.LogMessageToFile("OnReceiveData extraction of length failed. " + message, true);
                continue;
            }

            if (length != extracted_data[4].Length)
            {
                MyUtils.LogMessageToFile("OnReceiveData lengths did not equal. Length =" + length.ToString() + " Actual=" + extracted_data[3].Length.ToString(), true);
                continue;
            }

            // Get Compression type
            int compression = 0;
            if (!int.TryParse(Encoding.UTF8.GetString(extracted_data[1]), out compression))
            {
                MyUtils.LogMessageToFile("OnReceiveData extraction of compression failed. " + message, true);
                continue;
            }

            // Get Client ID
            int clientID = 0;
            if (!int.TryParse(Encoding.UTF8.GetString(extracted_data[2]), out clientID))
            {
                MyUtils.LogMessageToFile("OnReceiveData extraction of clientId failed. " + message, true);
                continue;
            }


            // ************ Process Message **********************
            string messageOnly = MyUtils.DecompressMessage(extracted_data[4], compression);
            
            
            ReadOnlySpan<char> spanData = messageOnly.AsSpan();
            List<int> spanDataSplits = MyUtils.Split(spanData, GLOBALS.SEPARATOR1);

            //   string[] splitData = messageOnly.Split(GLOBALS.SEPARATOR1); //our data is separated via a '|'        

            // Sanity check
            // if (splitData.Length < 2)
            if (spanDataSplits.Count < 2)
            {
                MyUtils.LogMessageToFile("onReceivedData unable to split on |. Client IP = " + currMessage.endpoint.Address.ToString(), true);
                continue;
            }


            // Check if new Player (special handling)
            //if (splitData[0] == GLOBALS.HEADER_IN_NEWPLAYER )
            if (MyUtils.CompareSplit(spanData, spanDataSplits, 0, GLOBALS.HEADER_IN_NEWPLAYER) )
            {
                AddNewClient(spanData, spanDataSplits, currMessage.endpoint);
                continue;
            }

            Client dummy;
            // Make sure client exists
            if (!allClients.TryGetValue(clientID, out dummy))
            {
                // Don't bother logging - common problem
                // MyUtils.LogMessageToFile("Client ID not present in our list. clientID=" + clientID + ", message="  + messageOnly, true);
                continue;
            }

            // Now verify allClients containst clientID and if it does, that it has the same endpoint
            // NOTE: May want to remove this - it would allow someone else to spoof themselves as another player, but
            // it would also prevent someone from being dropped if their dynamic DNS IP changed.
            // if (!(clientID >= 0 && allClients.ContainsKey(clientID) && allClients[clientID].endpoint.ToString() == currMessage.endpoint.ToString()))
            // {
            //    MyUtils.LogMessageToFile("onReceivedData clientID and end-point mis-match. Client IP = " + currMessage.endpoint.Address.ToString() + " ID = ", true);
            //    continue;
            // }

            if (MyUtils.GetSplitSpan(spanData, spanDataSplits, 0).SequenceEqual(GLOBALS.HEADER_IN_INPUTS))
            {
                OnClientInputReceive(clientID, spanData, spanDataSplits);
            }
            else if (MyUtils.GetSplitSpan(spanData, spanDataSplits, 0).SequenceEqual(GLOBALS.HEADER_FLAGS))
            {
                OnClientFlags(clientID, spanData, spanDataSplits);
            }
            else
            {
                MyUtils.LogMessageToFile("OnReceiveData invalid HEADER:" + MyUtils.GetSplitString(spanData, spanDataSplits,0), true);
            }

         

            // Update player message time     
            // The clientflags may have deleted the clientID
            if( allClients.ContainsKey(clientID))
            {
            allClients[clientID].time_last_message = GetTimeMillis();

            // Increment player message count
            allClients[clientID].message_count++;
            }
        }
    }
      
    // Process client inputs
    // Future re-write note: currently sending each input as a seperate entity. If Robot controll code is changed to
    // read controller inputs based on a hash/dictionary, than code can be simplified to iterate through
    // hash keys.
    private bool OnClientInputReceive(int cnnId, ReadOnlySpan<char> rawData, List<int> indexes)
    {
        try
        {
            // If no avatar associated with client, don't bother getting the inputs
            if (allClients[cnnId].avatar == null) { return true; }

            RobotInterface3D controller = allClients[cnnId].robot;

            // Exit if no controller found. 
            if (controller == null) { return false; }

            // Split the data based on Separator2
            ReadOnlySpan<char> splitData = MyUtils.GetSplitSpan(rawData, indexes, 1);
            List<int> splitIndexes = MyUtils.Split(splitData, GLOBALS.SEPARATOR2);


            // string[] splitData = rawData[1].Split(GLOBALS.SEPARATOR2); //our data is separated via SEPARATOR2

            int id = 0;
            // Basic Buttons
            controller.gamepad1_a = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_b = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_x = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_y = MyUtils.SpanToBool(splitData, splitIndexes[id++]);

            // Basic Movement
            controller.gamepad1_right_stick_y = float.Parse(MyUtils.GetSplitSpan(splitData, splitIndexes, id++));
            UnityEngine.Debug.Log("WRONG somehow we are setting gampead1_right_stick_x from client input receive");
            controller.gamepad1_right_stick_x = float.Parse(MyUtils.GetSplitSpan(splitData, splitIndexes, id++));
            controller.gamepad1_left_stick_x = float.Parse(MyUtils.GetSplitSpan(splitData, splitIndexes, id++));
            controller.gamepad1_left_stick_y = float.Parse(MyUtils.GetSplitSpan(splitData, splitIndexes, id++));

            // DPAD
            controller.gamepad1_dpad_down = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_dpad_up = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_dpad_left = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_dpad_right = MyUtils.SpanToBool(splitData, splitIndexes[id++]);

            // Other
            controller.gamepad1_right_bumper  = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_left_bumper   = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_left_trigger   = float.Parse(MyUtils.GetSplitSpan(splitData, splitIndexes, id++));
            controller.gamepad1_right_trigger  = float.Parse(MyUtils.GetSplitSpan(splitData, splitIndexes, id++));
            controller.gamepad1_stop        = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
            controller.gamepad1_restart     = MyUtils.SpanToBool(splitData, splitIndexes[id++]);
        }
        catch (Exception e)
        {
            MyUtils.LogMessageToFile("OnClientInputReceive error: " + e, true);
            return false;
        }

        return true;
    }


    // Process client flags
    private void OnClientFlags(int cnnId, ReadOnlySpan<char> rawData, List<int> indexes)
    {
        // See if the cnnId exists
        if (!allClients.ContainsKey(cnnId))
        {
            return;
        }

        // Clear all the client flags
        allClients[cnnId].flags.Clear();

        // Now extract all the messages
        ReadOnlySpan<char> all_flag_messages = MyUtils.GetSplitSpan(rawData, indexes, 1);
        List<int> all_flag_indexes = MyUtils.Split(all_flag_messages, GLOBALS.SEPARATOR3);

        int message_id;
        int message_id2;

        for (int i = 0; i < all_flag_indexes.Count; i++)
        {
            // Split the data based on Separator2
            ReadOnlySpan<char> splitData = MyUtils.GetSplitSpan(all_flag_messages, all_flag_indexes, i);
            List<int> splitDataIndexes = MyUtils.Split(splitData, GLOBALS.SEPARATOR2); //our data is separated via SEPARATOR2

            // Should have at least 4 items for first meaage
            if (splitDataIndexes.Count < 4) { return; }

            // Get the current message id
            message_id = int.Parse( MyUtils.GetSplitSpan(splitData, splitDataIndexes, 0));
            message_id2 = int.Parse(MyUtils.GetSplitSpan(splitData, splitDataIndexes, 1));

            // Drop corrupted ids
            if( message_id != message_id2 ) { continue; }

            // If this is an old meesage, we need to drop it
            if ( message_id <= allClients[cnnId].server_confirmed_this_msg_id ) {

                // Check if data corruption occured and we need to reset the id tracker
                // If the difference is too large, we may have  corruption. We then want to re-initizalize it
                if (message_id + (2 * GLOBALS.MAX_TRACKED_PACKETS) < allClients[cnnId].server_confirmed_this_msg_id)
                {
                    allClients[cnnId].server_confirmed_this_msg_id = message_id;
                }

                continue; 
            }

            else { allClients[cnnId].server_confirmed_this_msg_id = message_id; }

            for (int j = 2; j < splitDataIndexes.Count - 1; j += 2)
            {
                allClients[cnnId].flags[MyUtils.GetSplitString(splitData, splitDataIndexes, j)] = MyUtils.GetSplitString(splitData, splitDataIndexes, j+1);
            }

            if (allClients[cnnId].flags.ContainsKey("TRACKED_FLAG_ID"))
            {
                allClients[cnnId].client_confirmed_this_msg_id = int.Parse(allClients[cnnId].flags["TRACKED_FLAG_ID"]);
                // If this read in wrong to an outragouse number, correct it
                if(allClients[cnnId].client_confirmed_this_msg_id > allClients[cnnId].curr_tracked_packet)
                {
                    allClients[cnnId].client_confirmed_this_msg_id = allClients[cnnId].curr_tracked_packet;
                }
               
            }         
            // Process Client Flags
            ProcessClientFlags();
        }
    }

    // Stores unique ID for each new client
    private int clientIDcounter = 0;

    // public void AddNewClient(string[] data, IPEndPoint endpoint)
    public void AddNewClient(ReadOnlySpan<char> data, List<int> data_splits, IPEndPoint endpoint)
    {
        // If there aren't >=13 items, quit
        if(data_splits.Count < 14)
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + "INVALID" + GLOBALS.SEPARATOR1 + "Invalid server request! ", endpoint);
            return;
        }

        // Extract connection information
        String playerName = MyUtils.GetSplitString( data, data_splits, 1);
        String password = MyUtils.GetSplitString(data, data_splits, 2);
        String startfield = MyUtils.GetSplitString(data, data_splits, 3);
        String RobotModel = MyUtils.GetSplitString(data, data_splits, 4);
        String gamename = MyUtils.GetSplitString(data, data_splits, 5);
        String clientversion = MyUtils.GetSplitString(data, data_splits, 6);
        String DriveTrain = MyUtils.GetSplitString(data, data_splits, 7);
        String speed = MyUtils.GetSplitString(data, data_splits, 8);
        String acceleration = MyUtils.GetSplitString(data, data_splits, 9);
        String weight = MyUtils.GetSplitString(data, data_splits, 10);
        String turning_scaler = MyUtils.GetSplitString(data, data_splits, 11);
        String fieldcentric = MyUtils.GetSplitString(data, data_splits, 12);
        String activebreaking = MyUtils.GetSplitString(data, data_splits, 13);
        String tankcontrol = MyUtils.GetSplitString(data, data_splits, 14);
        String skins = "0";
        bool isAdmin = false;


        if(data_splits.Count >= 16)
        {
            skins = MyUtils.GetSplitString(data, data_splits, 15);
        }
        String robotskins = "";
        if (data_splits.Count >= 17)
        {
            robotskins = MyUtils.GetSplitString(data, data_splits, 16);
        }

        // Check password
        if ( (PASSWORD.Length > 0 && PASSWORD != password) &&  (startfield != "Admin") )
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Invalid Password! ", endpoint);
            return;
        }


        // Check Game
        if ( gamename != GLOBALS.GAME)
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Wrong game: server is running  " + GLOBALS.GAME + "!", endpoint);
            return;
        }

        // Check Version
        if (clientversion != Application.version)
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Version mis-match: server is running " + Application.version + "!", endpoint);
            return;
        }

        // Check if Spectator limit is reached
        if (startfield == "Spectator" && GetSpectatorCount() >= max_spectators)
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Maximum spectator count reached! ", endpoint);
            return;
        }

        // If the person's admin name is incorrect, drop them
        if( (startfield == "Admin") && ((ADMIN.Length < 1) || (password != ADMIN)))
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Incorrect admin credentials! ", endpoint);
            return;
        }

        // Check if Admin limit is reached
        // Admin position will be changed to "Spectator" once done here.
        if ((startfield == "Admin") && (GetAdminSpecCount() >=2) && (GetSpectatorCount() >= max_spectators))
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Maximum admin count reached! ", endpoint);
            return;
        }

        // Change position of Admin back to spectator now that it was let in
        if( startfield == "Admin")
        {
            startfield = "Spectator";
            isAdmin = true;
        }

        if( (ADMIN.Length>= 1) && (playerName == ADMIN))
        {
            isAdmin = true;
        }

        Transform starting_pos = null;

        // If this is a non Spectator position, correct position for game restrictions
        if( startfield != "Spectator")
        {
            List<string> used_positions = new List<string>();

            // Add existing positions to used positions
            foreach (Client currclient in allClients.Values)
            {
                used_positions.Add(currclient.starting_position);
            }

            // Qualify positions
            starting_pos = scorer.CorrectRobotPosition(startfield, used_positions);
            if (starting_pos) { startfield = starting_pos.name; }
        }


        // Only create a new player if the new is unique and it's position isn't taken
        // First check if position is taken (assuming if correctRobotPosition returned null that means it's taken)
        if ((startfield != "Spectator") && (starting_pos == null))
        {
            SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Position " + startfield + " already taken!", endpoint);
            return;
        }
        
        // Check for unique name if player 
        foreach (Client currclient in allClients.Values)
        {
            if (currclient.name == playerName)
            {
                // If the end-point is the same, then silently ignore, otherwise, notify user
                if (currclient.endpoint != endpoint)
                {
                    SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Name " + playerName + " already taken!", endpoint);
                }
                return;
            }
        }


        int cnnId = clientIDcounter;//the id will be a number that is the order in which they connected, so each client is given a unique id
        GLOBALS.client_names[cnnId] = playerName;
        GLOBALS.client_ids[playerName] = cnnId;

        clientIDcounter++;

        Client newclient = new Client();
        newclient.name = playerName;

        // Add it to the player list immedietally in case something errors out below
        allClients.Add(cnnId, newclient);

        // Correct robot model to allowed values
        if (startfield != "Spectator")
        {
            RobotModel = scorer.CorrectRobotChoice(RobotModel);
        }

        newclient.id = cnnId;
        newclient.starting_position = startfield;
        newclient.endpoint = endpoint;
        newclient.robotmodel = RobotModel;
        newclient.DriveTrain = DriveTrain;
        newclient.skins = skins;
        newclient.robotskins = robotskins;
        float.TryParse(speed, out newclient.speed);
        float.TryParse(acceleration, out newclient.acceleration);
        float.TryParse(weight, out newclient.weight);
        float.TryParse(turning_scaler, out newclient.turning_scaler);
        bool.TryParse(fieldcentric, out newclient.fieldcentric);
        bool.TryParse(activebreaking, out newclient.activebreaking);
        bool.TryParse(tankcontrol, out newclient.tankcontrol);
        newclient.isAdmin = isAdmin;

        // Do sanity check on speed, acceleration and weight
        // Weight needs to be between 15 and 42
        // This really shouldn't be done here but in RobotControlled3D
        if (newclient.weight < 15f) { newclient.weight = 15f;  }
        if (newclient.weight > 42f) { newclient.weight = 42f; }
        
        // Next, weight * speed * acceleration < 2600
        if(newclient.speed * newclient.acceleration * newclient.weight > 2600f)
        {
            newclient.speed = 6.28f;
            newclient.acceleration = 25.1f;
            newclient.weight = 15f;
        }

        newclient.flags = new Dictionary<string, string>();
        newclient.server_sent_packets = new Dictionary<int, string>();
        newclient.serverFlags = new Dictionary<string, string>();
        newclient.serverInterrupts = new Dictionary<string, string>();
        newclient.time_last_message = GetTimeMillis();  

        // Try to get the starting position
        Transform position = transform;

        if (starting_pos != null)
        {
            position = starting_pos;
            newclient.starting_pos = starting_pos;
        }

        // Create the object
        // But only if not a spectator
        if (startfield != "Spectator")
        {
            GameObject go = CreatePlayerAvatar(newclient);

            if( go == null )
            {
                SendUdp(GLOBALS.HEADER_ERROR + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + "Robot model " + RobotModel + " doesn't exist!", endpoint);
                return;
            }

            // Tell everybody that a new player has connected
            // We don't do anything with this
            // SendUdp(GLOBALS.HEADER_NEWPLAYER + GLOBALS.SEPARATOR1 + message_id.ToString() + GLOBALS.SEPARATOR1 + playerName + GLOBALS.SEPARATOR1 + cnnId, null);
        }

        // Create the status line around it
        newclient.status_line = (GameObject)Instantiate(clientlineprefab);
        newclient.status_line.transform.Find("USER").GetComponent<Text>().text = newclient.name;
        newclient.status_line.transform.Find("Location").GetComponent<Text>().text = newclient.starting_position;
        newclient.status_line.transform.Find("IP").GetComponent<Text>().text = newclient.endpoint.Address.ToString();
        newclient.status_line.transform.Find("DATA").GetComponent<Text>().text = "1";
        newclient.status_line.transform.Find("Kick").GetComponent<Button>().onClick.AddListener(delegate { this.RemoveClient(newclient.id); } );

        // Finaly have scorekeeper parse this person
        if (newclient.robot) { scorer.AddPlayer(newclient.robot, true); }

        MyUtils.LogMessageToFile("Player " + newclient.name + " joined on position " + newclient.starting_position + " from IP=" + newclient.endpoint.Address.ToString() + ". Total Players = " + GetPlayerCount() + ", Specs = " + GetSpectatorCount(), false);

        // If Admin, update its menu
        if( isAdmin )
        {
            if (ourgamesettings)
            {
                ourgamesettings.UpdateServer();
            }
        }
    }

    // Destructive operation: CreatePlayerAvatar deletes all data that may have been added/associated with the avatar
    private GameObject CreatePlayerAvatar(Client client)
        // string RobotModel, Transform position, string RobotPosition, int cnnId, string playerName, string DriveTrain )
    {
        // Instantiate it
        Vector3 pos = client.starting_pos.position;
        Quaternion rot = client.starting_pos.rotation;
        GameObject robot_instance = MyUtils.InstantiateRobot(client.robotmodel, pos, rot, client.skins, client.robotskins) as GameObject;
        robot_instance.name = "PLAYER" + client.id.ToString();
        client.avatar = robot_instance;

        // Assign the RobotID
        RobotID newid = robot_instance.AddComponent<RobotID>() as RobotID;
        newid.starting_pos = client.starting_position;
        newid.id = client.id;

        // Turn off interpolation in object
        TurnOffInterpolationInObject(robot_instance);

        // Set the drive-train
        client.robot = robot_instance.GetComponent<RobotInterface3D>();
        client.robot.SetUserParameters(client.speed, client.acceleration, client.weight, client.DriveTrain, client.turning_scaler, 
                                            (client.fieldcentric) ? 1:0, (client.activebreaking) ? 1:0, (client.tankcontrol) ? 1:0);
        client.robot.SetName(client.name);

        // Set it's color
        client.robot.SetColorFromPosition(client.starting_position);

        // Set the camera position for field centric controls
        GameObject target_camera = GameObject.Find(client.starting_position + " Cam");
        if (!target_camera) // If there is no camera location for this position, make it the spectator
        {
            target_camera = GameObject.Find("Spectator Cam");
        }
        
        client.camera_rotation = target_camera.transform.rotation;

        client.robot.fieldcentric_rotation = client.camera_rotation;
        client.robot.Initialize();

        // Update robot's internal calculations

        // Update scorer so it can keep track of robot-to-robot collisions
        scorer.FieldChanged();

        InitHeadlessGraphics();

        return robot_instance;
    }

    // Remove client from the scene
    private void RemoveClient(int cnndId)
    {
        // Make sure key exists
        if( !allClients.ContainsKey(cnndId)) { return; }

        MyUtils.LogMessageToFile("Removing " + allClients[cnndId].name, false);

        if (allClients[cnndId].avatar != null)
        {
            if (allClients[cnndId].robot) { allClients[cnndId].robot.deleted = true; }
            Destroy(allClients[cnndId].avatar);
            allClients[cnndId].avatar = null;
        }

        if (allClients[cnndId].status_line != null)
        {
            Destroy(allClients[cnndId].status_line);
            allClients[cnndId].status_line = null;
        }

        allClients.Remove(cnndId);
    }

    // Reset player position
    // Implement 1s timeouts
    Dictionary<int, float> pos_reset_tracker = new Dictionary<int, float>();
    private void ResetPlayerPosition(int playerid, bool referee = false)
    {
        if (scorer.ALLOW_RESET_POS)
        {
            // Limit reset to once a second
            if( !referee && pos_reset_tracker.ContainsKey(playerid) && (Time.time - pos_reset_tracker[playerid] < 1f))
            {
                    return;
            }

            // Update last reset to current time
            pos_reset_tracker[playerid] = Time.time;

            // Let scorer know a position reset occured
            scorer.PlayerReset(playerid, referee);

            // Destructive operation: deletes data inside the avatar
            ResetAvatarPosition(playerid);
        }
    }


    // Called from Update to process any flags that already arrived
    // NOTE: Flags are sent only when client and/or server gets to them. 
    //       A client communicates a desire by setting a flag.. when server sees it, it will
    //       set a confirmation flag back. At that point the client should de-assert the flag after
    //       which the server de-asserts the o.k. 
    // This has been changed a bit since adding trackkedUDP packets.
    private void ProcessClientFlags()
    {

        // Reset any flags needed at this stage

        // Go through each client and look for specific tags
        // Since server commands such as "Kill All" may modify this list, will do checks to make sure everything is kocher everytime
        int[] allkeys = allClients.Keys.ToArray();

        foreach (int currindex in allkeys)
        {
            // Make sure this cleint still exists
            if( ! allClients.ContainsKey(currindex)) { continue; }

            Client currclient = allClients[currindex];

            // Reset our response to the client (resides in the clients index)
            serverFlags.Remove(currindex.ToString());

            foreach (String currkey in currclient.flags.Keys)
            {
                switch (currkey)
                {
                    case "RESTARTALL":

                        // Make sure user didn't restart in the last 2m35s, or at least not during the game
                        if( ((GetTimeMillis() - currclient.lastrestart) < 155000) && (scorer.timerstate == Scorekeeper.TimerState.RUNNING) ) { break; }

                        if ( tournament_mode )
                        {
                            if ((tournament_sm != TOURNAMENT_STATES.RUNNING) && !holding_mode)
                            {
                                tournament_sm = TOURNAMENT_STATES.WAITING;
                                currclient.lastrestart = GetTimeMillis();
                                RestartLevel();
                            }
                            else
                            {
                                ShowLogMessage("Can't reset field while running tournament game (" + currclient.name + ")");
                                AddChat("Can't reset field while running tournament game (" + currclient.name + ")");
                                break;
                            }
                        }
                        else
                        {
                            currclient.lastrestart = GetTimeMillis();
                            RestartLevel();
                        }
                        
                        ShowLogMessage("Field reset by " + currclient.name);
                        AddChat("Field reset by " + currclient.name);
                        
                        break;

                    case "RESETMYPOS":
                        ResetPlayerPosition(currclient.id);                      
                        break;

                    case "CHAT":
                        // Process any client commands first, and if none found, send it out as a chat
                        if (!ProcessClientCommands(currclient.name, currclient.flags[currkey], currclient.isAdmin))
                        {
                            // Chat messages by ADMIN will be processed in Client Commands
                            ShowLogMessage(currclient.name + ": " + currclient.flags[currkey]);
                            AddChat(currclient.name + ": " + currclient.flags[currkey]);
                        }
                        break;

                    case "GameSettings":
                        // Change game settings if this is the admin
                        if( (ADMIN.Length > 0) && (currclient.name == ADMIN) || currclient.isAdmin)
                        {
                            if( ourgamesettings)
                            {
                                ourgamesettings.SetString(currclient.flags[currkey]);
                            }
                        }

                        break;
                }
            }

            // Now clear the flags that were processed
            currclient.flags.Remove("RESTARTALL");
            currclient.flags.Remove("RESETMYPOS");
            currclient.flags.Remove("CHAT");
            currclient.flags.Remove("GameSettings");

            // currclient.flags.Clear(); // Only clear the flags that were processed above
        }

  
    }

    // Application manager will set this when it finds the server
    public ApplicationManager top_application_manager = null;
    private bool ProcessClientCommands(string name,string msg, bool isAdmin = false)
    {     
        // Do admin only commands from here on
        // See if this person is an admin
        if( (name != ADMIN) && !isAdmin ) { return false; }

        // Process chat meesages for commands
        if (msg.StartsWith("/SERVER "))
        {
            string substring = msg.Substring(8);
            string[] split = substring.Split('=');
            split[0] = split[0].TrimEnd();

            if (split[0] == "RESTART")
            {
                ServerMenu_RestartLevel();
                MyUtils.LogMessageToFile("/SERVER: Restarted game.", false);
            }
            else if (split[0] == "PASSWORD")
            {
                PASSWORD = split[1];
                MyUtils.LogMessageToFile("/SERVER: PASSWORD = " + PASSWORD, false);
            }
            else if (split[0] == "KICKALL")
            {
                // Copy over keys so that when we destroy them, we don't get errors from the enumerator
                int[] allkeys = new int[allClients.Keys.Count];

                allClients.Keys.CopyTo(allkeys, 0);

                foreach ( int currclient in allkeys)
                {
                    if( (allClients[currclient].name != ADMIN) && (!allClients[currclient].isAdmin))
                    {
                        RemoveClient(currclient);
                    }
                }

                MyUtils.LogMessageToFile("/SERVER: Kicked all players.", false);

            }
            else if (split[0] == "KICK")
            {
                string playername = split[1];
                bool kickedplayer = false;

                foreach (int currclient in allClients.Keys)
                {
                    if (allClients[currclient].name == playername)
                    {
                        RemoveClient(currclient);
                        kickedplayer = true;
                        break;
                    }
                }

                if (!kickedplayer) { 
                    MyUtils.LogMessageToFile("/SERVER: Kicked player " + playername + " not found!", false);
                    return false;
                }
                else { MyUtils.LogMessageToFile("/SERVER: Kicked player " + playername, false); }
            }
            else if (split[0] == "KICKID")
            {
                int id = -1;
                if (!int.TryParse(split[1], out id))
                {
                    MyUtils.LogMessageToFile("/SERVER: KICKID failed to extract id number from: " + split[1], false);
                    return false;
                }
            
                if( !allClients.ContainsKey(id))
                {
                    MyUtils.LogMessageToFile("/SERVER: KICKID player with id not found: " + split[1], false);
                    return false;
                }

                string playername = allClients[id].name;

                // Remove the client
                RemoveClient(id);

                MyUtils.LogMessageToFile("/SERVER: Kicked player " + playername, false); 
            }
            else if (split[0] == "STOP")
            {
                scorer.SetTimerState(Scorekeeper.TimerState.STOPPED);
                MyUtils.LogMessageToFile("/SERVER: Timer stopped.", false);
            }
            else if (split[0] == "BLUEADJ")
            {
                int adjust = 0;
                if( int.TryParse(split[1],out adjust))
                {
                    scorer.score_blueadj = adjust;
                }

                MyUtils.LogMessageToFile("/SERVER: Blue score adj = " + adjust, false);
            }
            else if (split[0] == "REDADJ")
            {
                int adjust = 0;
                if (int.TryParse(split[1], out adjust))
                {
                    scorer.score_redadj = adjust;
                }
                MyUtils.LogMessageToFile("/SERVER: Red score adj = " + adjust, false);
            }
            else if (split[0] == "MESSAGE")
            {
                if( split.Length < 3)
                {
                    MyUtils.LogMessageToFile("/SERVER: MESSAGE missing 2 '=' signs. ", false);
                    return false;
                }

                int id = -1;
                if (!int.TryParse(split[1], out id))
                {
                    MyUtils.LogMessageToFile("/SERVER: MESSAGE failed to extract id number from: " + split[1], false);
                    return false;
                }

                if (!allClients.ContainsKey(id))
                {
                    MyUtils.LogMessageToFile("/SERVER: MESSAGE player with id not found: " + split[1], false);
                    return false;
                }

                string outmsg = string.Join("=", split, 2, split.Length - 2);

                AddChat(outmsg, id);
                string playername = allClients[id].name;              
                MyUtils.LogMessageToFile("/SERVER: Message sent to player " + playername + " = " + outmsg, false);         
            }
            else if (split[0] == "RESET")
            {
                int id = -1;
                if (!int.TryParse(split[1], out id))
                {
                    MyUtils.LogMessageToFile("/SERVER: RESET failed to extract id number from: " + split[1], false);
                    return false;
                }

                if (!allClients.ContainsKey(id))
                {
                    MyUtils.LogMessageToFile("/SERVER: RESET player with id not found: " + split[1], false);
                    return false;
                }

                string playername = allClients[id].name;

                // Reset the client
                ResetPlayerPosition(id, true);  
                
                MyUtils.LogMessageToFile("/SERVER: Ref reset player " + playername, false);
            }
            else if (split[0] == "ROBOTCOUNTER")
            {
                if (split.Length < 3)
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTER missing 2 '=' signs. ", false);
                    return false;
                }

                int id = -1;
                if (!int.TryParse(split[1], out id))
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTER failed to extract id number from: " + split[1], false);
                    return false;
                }

                if (!allClients.ContainsKey(id))
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTER player with id not found: " + split[1], false);
                    return false;
                }

                float duration = -1;
                if (!float.TryParse(split[2], out duration))
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTER failed to extract duration from: " + split[2], false);
                    return false;
                }

                scorer.SetRobotCounter(id, duration);

                string playername = allClients[id].name;
                MyUtils.LogMessageToFile("/SERVER: Robot counter set on " + playername + " = " + split[2], false);
            }
            else if (split[0] == "ROBOTCOUNTERRESET")
            {
                if (split.Length < 2)
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTERRESET missing 1 '=' signs. ", false);
                    return false;
                }

                int id = -1;
                if (!int.TryParse(split[1], out id))
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTERRESET failed to extract id number from: " + split[1], false);
                    return false;
                }

                if (!allClients.ContainsKey(id))
                {
                    MyUtils.LogMessageToFile("/SERVER: ROBOTCOUNTERRESET player with id not found: " + split[1], false);
                    return false;
                }
            
                scorer.ResetRobotCounter(id);

                string playername = allClients[id].name;
                MyUtils.LogMessageToFile("/SERVER: Robot counter reset on " + playername, false);
            }
            else
            {
                // Process through our line argument system
                ProcessCommand(split[0], ((split.Length > 1) ? split[1] : ""));
            }

            // Make server admin menu synchronized to changes created here
            if (top_application_manager)
            {
                top_application_manager.UpdateMenuesToServerSettings();
            }

        }
        // Otherwise process as regular chat
        else
        {
            ShowLogMessage("<ADMIN>: " + msg);
            AddChat("<ADMIN>: " + msg);
        }
        return true;  
    }

    // ********************** SOUND STUFF *****************************
    public void playSound(int id, string soundname, float crossFadeTime, float volume, float pitch = -1f)
    {
        // We will send sounds via flags (since we don't want to miss important sounds like start of game
        string msg = id.ToString() + GLOBALS.SEPARATOR5 + soundname + GLOBALS.SEPARATOR5 + crossFadeTime.ToString("0.#") + GLOBALS.SEPARATOR5 + volume.ToString("0.#") + GLOBALS.SEPARATOR5 + pitch.ToString("0.#");

        if (serverInterrupts.ContainsKey("SOUND_PLAY"))
        {
            serverInterrupts["SOUND_PLAY"] += GLOBALS.SEPARATOR4 + msg;
        }
        else
        {
            serverInterrupts["SOUND_PLAY"] = msg;
        }
    }

    public void stopSound(int id, string soundname, float crossFadeTime)
    {
        // We will send sounds via flags (since we don't want to miss important sounds like start of game
        string msg = id.ToString() + GLOBALS.SEPARATOR5 + soundname + GLOBALS.SEPARATOR5 + crossFadeTime.ToString("0.#");

        if (serverInterrupts.ContainsKey("SOUND_STOP"))
        {
            serverInterrupts["SOUND_STOP"] += GLOBALS.SEPARATOR4 + msg;
        }
        else
        {
            serverInterrupts["SOUND_STOP"] = msg;
        }
    }

    private void ClearSoundFlags()
    {
        serverFlags.Remove("SOUND_STOP");
        serverFlags.Remove("SOUND_PLAY");
    }

    // **************************************************************
    // **************************************************************
    // ****************RAW UDP related section **********************
    // **************************************************************



    // ************ UDP Algorithm 0: udpClient asyncrhonous receive
    // used udpClient asynch receive to process incoming packets
    private int datareceivederrors = 0;

    private void DataReceived(IAsyncResult ar)
    {
        if (killme)
        { return; }

        // Finish reading rest of data
        UdpClient currentclient = (UdpClient)ar.AsyncState;
        Message newmessage = new Message();
        newmessage.endpoint = new IPEndPoint(IPAddress.Any, 0);

        try
        {
            newmessage.data = currentclient.EndReceive(ar, ref newmessage.endpoint);

            // should never fault past this point, but don't want to execute it if above fails
            if (allReceivedDataSemaphore.WaitOne())
            {
                allReceivedData.Add(newmessage);
                allReceivedDataSemaphore.Release();
            }
        }
        catch (Exception e)
        {
            if (datareceivederrors < 10)
            { MyUtils.LogMessageToFile("DR Exception: " + e.ToString(), false); }
            datareceivederrors++;

            // Will not "return" in order not to kill communication
        }

        // Start next receive cycle
        if (!killme)
        {
            currentclient.BeginReceive(DataReceived, ar.AsyncState);
        }
    }


    // ************ UDP Algorithm 1: Single blocking thread
    // This function will use semaphore to gain control of udpClient, but be running in
    // separate thread.
    Semaphore udpSemaphore = new Semaphore(1, 1);

    private void DataReceive()
    {
        // Keep receiving new messages until we have a problem
        // We are using 2 sempahores: messages and udp
        // to make sure we don't have an interlock issues, lets split the code so only one 
        // is required at any given time


        while (!killme)
        {
            int sleep_duration = 0;
            byte[] receivedBytes;
            IPEndPoint incomingEP;

            // First deal with the udpClient data (and semaphore)
            try
            {
                // Do check if read is possible
                udpSemaphore.WaitOne();
                if (m_udpClient.Available < 1)
                {
                    // If no data available, then might as well sleep for a little while
                    sleep_duration = 1;
                    continue;
                }

                incomingEP = new IPEndPoint(IPAddress.Any, 0);
                receivedBytes = m_udpClient.Receive(ref incomingEP);
            }
            catch (Exception e)
            {
                MyUtils.LogMessageToFile("DR GLOBAL udpClient Exception: " + e.ToString(), false);

                // Try again later?
                continue;
            }
            finally
            {
                udpSemaphore.Release();
            }

            // If we didn't see any data, sleep a little
            if (sleep_duration > 0)
            {
                Thread.Sleep(sleep_duration);
                continue;
            }

            // If we got here it's because there is a mesage
            Message newmessage = new Message();
            newmessage.endpoint = incomingEP;
            newmessage.data = receivedBytes;

            // Add to our queue of messages to be processed
            allReceivedDataSemaphore.WaitOne();
            allReceivedData.Add(newmessage);
            allReceivedDataSemaphore.Release();
            // Thread.Yield();
            Thread.Sleep(0);
        }
    }

    // ************ UDP Algorithm 2: Serial Access
    // This function will called inside fixed update.
    // It therefore doesn't need to worry about semaphore. 

    private void DataReceiveSerial()
    {
        // Keep receiving new messages until we have a problem
        // We are using 2 sempahores: messages and udp
        // to make sure we don't have an interlock issues, lets split the code so only one 
        // is required at any given time
        if (killme) { return; }

        if (m_udpClient == null) { return; }

        // get any data that is present
        while (m_udpClient.Available > 10)
        {
            byte[] receivedBytes;
            IPEndPoint incomingEP;
            incomingEP = new IPEndPoint(IPAddress.Any, 0);

            try
            {
                receivedBytes = m_udpClient.Receive(ref incomingEP);
                Message newmessage = new Message
                {
                    endpoint = incomingEP,
                    data = receivedBytes
                };

                // Add to our queue of messages to be processed
                allReceivedData.Add(newmessage);
            }
            catch (Exception e)
            {
                if (datareceivederrors > 10)
                {
                    MyUtils.LogMessageToFile("Error while receiving: " + e, true);
                    datareceivederrors++;
                }
            }
        }

    }

    public UdpClient m_udpClient;
    public IPEndPoint remoteEP;

    public void SetupUdp(int port = 14446)
    {
        remoteEP = new IPEndPoint(IPAddress.Any, port);
        m_udpClient = new UdpClient(remoteEP);

        // Don't allow fragmentation: the name of the property is BACKWARDS to how you set it?
        // Maybe fragmentation is o.k. (meaning don't drop fragmented packets)... rely on ip layer 3 to re-assemble.
        m_udpClient.DontFragment = false;

        MyUtils.LogMessageToFile("done setting up udp", false);
    }

    public long datacounter_old = 0; // time of last counting
    public long datacounter = 0;       // number of bytes sent

    private void ResetDataCounter(bool force = false)  // Resets data counter if it expired (or force it to reset)
    {
        // If time exceeds count period, then reset counter
        if (force || (GetTimeMillis() - datacounter_old) > GLOBALS.UDP_DELAY_TIME_MS )
        {
            datacounter = 0;
            datacounter_old = GetTimeMillis();
        }
    }

    public long udp_sleep_lastime = 0;
    
    private void UDPSendSleep(float durations_ms)  // Keeps polling for incoming data until time expired
    {
        long start_time = GetTimeMillis();


        while ( (GetTimeMillis() - start_time) < durations_ms )
        {
            udp_sleep_lastime = GetTimeMillis();

            // if this is a serial communication method, do it here
            // **** receive any messages ****
            if (GLOBALS.UDP_ALGORITHM == 2)
            {
                DataReceiveSerial();
            }
        }
    }

    public float UDP_SLEEP = GLOBALS.UDP_DELAY_TIME_MS;
    private int time_day_count = 0;
    private Dictionary<string, long> stats_outgoing = new Dictionary<string, long>();
    private long STATS_PERIOD = 60000; // ms to wait to collect data
    private long STATS_LAST_TIME = 0; // last output time

    // SendUdp:
    // if destination present (single receiver)
    // If destination null, it will send it to the group of people specified by "who"
    public bool SendUdp(string messageraw, IPEndPoint destination = null, int Compression = GLOBALS.PACKET_COMPRESSION, SendDataFlags who = SendDataFlags.EVERYONE)
    {
        // Output network statistics if applicable
        string statkey = "";

        if( GLOBALS.NETSTATS > 0)
        {
            long period = (MyUtils.GetTimeMillis() - STATS_LAST_TIME);
         
            if (GLOBALS.NETSTATS == 1)
            {
                statkey = "DATA OUT";
            }
            else
            {
                statkey = messageraw.Split(GLOBALS.SEPARATOR1)[0]; // get the current key name
            }

            // Output all data if time expired
            if (period > STATS_PERIOD)
            {
                STATS_LAST_TIME = MyUtils.GetTimeMillis(); // Remember last time we output stat data

                if (GLOBALS.NETSTATS == 2) { MyUtils.LogMessageToFile("NETSTAT (ms): Period=" + period, false); }

                foreach( string currdata in stats_outgoing.Keys)
                {
                    MyUtils.LogMessageToFile("NETSTAT (B/s): " + currdata + "=" + (stats_outgoing[currdata] * 1000 / period).ToString("0"), false);
                }

                stats_outgoing.Clear();
            }

            // Otherwise just make sure our key is initialized
            if( !stats_outgoing.ContainsKey(statkey) )
            {
                stats_outgoing[statkey] = 0;
            }

        }

        // Will re-purpose "client id" space for timestamp
        // Time is ms since start of game, but wrapped to 24hour periods
        int timestamp = (int) (1000f*(Time.fixedTime - 60f * 60f * 24f * ((float) time_day_count)));
        if( timestamp/1000 > 60 * 60 * 24 ) { time_day_count++; }

        // Check if sleep is required 
        if ( datacounter > MAX_BYTES)
        {
            UDPSendSleep(UDP_SLEEP);
            ResetDataCounter(true);
        }
        else
        {
            ResetDataCounter();
        }

        // Use compression if enabled
        // Compress Message will overwrtie Compression if packet-compression is turned off
        byte[] message = MyUtils.CompressMessage(messageraw, Compression);

        bool bad_packet = false;
        if( (message[0] == 0x34) && (message[1] == 0x2e) && (message[2] == 0x35) && (message[3] == 0x11) && (message[4] == 0x34) && (message[5] == 0x34))
        {
            bad_packet = true;
        }

        string header = GLOBALS.PASSCODE + GLOBALS.SEPARATOR1 + 
                        Compression.ToString() + GLOBALS.SEPARATOR1 +
                        timestamp.ToString() + GLOBALS.SEPARATOR1 + // Server ID will be -1
                        message.Length.ToString() + GLOBALS.SEPARATOR1;

        byte[] messageBytes = MyUtils.CombineByteArrays(Encoding.UTF8.GetBytes(header), message);
        int bytes_sent;

        // Send a message to only one destination
        if (destination != null)
        {
            try
            {
                udpSemaphore.WaitOne();
                bytes_sent = m_udpClient.Send(messageBytes, messageBytes.Length, destination);
                sent_data_count += bytes_sent;
                datacounter += bytes_sent;

            }
            catch (Exception t)
            {
                MyUtils.LogMessageToFile("exeption when sending to player " + " : " + t.ToString(), true);
                return false;
            }
            finally
            {
                udpSemaphore.Release();
            }

            if (GLOBALS.NETSTATS > 0) { stats_outgoing[statkey] += bytes_sent; }

            return true;
        }
        // However, we want to go through all the clients randomly so that a person at the end of allClients doesn't awlays get dropped if bandwidth is a problem
        List<int> allkeys = Enumerable.ToList(allClients.Keys);
        Shuffle(allkeys);

        foreach (int currclient in allkeys)
        {
            if (who == SendDataFlags.PLAYERS && allClients[currclient].starting_position == "Spectator")
            { continue; }

            if (who == SendDataFlags.SPECTATORS && allClients[currclient].starting_position != "Spectator")
            { continue; }

            if ( GLOBALS.ENABLE_UDP_STATS && netmonitor_header != null)
            {
                string key = netmonitor_header + "_" + currclient + "_in";
                if ( netmonitor.ContainsKey(key) )
                {   netmonitor[key] += 1;  }
                else
                {   netmonitor.Add(key, 1); }
            }

            try
            {
                udpSemaphore.WaitOne();
                bytes_sent = m_udpClient.Send(messageBytes, messageBytes.Length, allClients[currclient].endpoint);
                sent_data_count += bytes_sent;
                datacounter += bytes_sent;
                if (GLOBALS.NETSTATS > 0) { stats_outgoing[statkey] += bytes_sent; }

                if (GLOBALS.ENABLE_UDP_STATS && netmonitor_header != null)
                {
                    string key = netmonitor_header + "_" + currclient + "_out";
                    if (netmonitor.ContainsKey(key))
                    { netmonitor[key] += 1; }
                    else
                    { netmonitor.Add(key, 1); }
                }
                
            }
            catch (Exception t)
            {
                MyUtils.LogMessageToFile("exeption when sending to player" + allClients[currclient].name + " : " + t.ToString(), true);
                allClients.Remove(currclient);
            }
            finally
            {
                udpSemaphore.Release();
            }

            // Check for data rate exceeded and go to sleep if so
            // Check if sleep is required 
            if (datacounter > MAX_BYTES)
            {
                UDPSendSleep(UDP_SLEEP);
                ResetDataCounter(true);
            }
            else
            {
                ResetDataCounter();
            }
        }


        return true;
    }

    // Sends all UDP packets that haven't been confirmed to clients
    // Update the tracked packet id once per frame

    public bool SendFlagsUDPTracked(string messageraw, int Compression = GLOBALS.PACKET_COMPRESSION, SendDataFlags who = SendDataFlags.EVERYONE, int receiver_id = -1)
    {
        // Will use SEPARATOR3 for seperating tracked packets
        // However, we want to go through all the clients randomly so that a person at the end of allClients doesn't awlays get dropped if bandwidth is a problem
        List<int> allkeys;
        
        // If we're sending to 1 person, then just populate the list with 1 person
        if( who == SendDataFlags.TARGETID)
        {
            allkeys = new List<int>();
            allkeys.Add(receiver_id);
        }
        else
        {
            allkeys = Enumerable.ToList(allClients.Keys);
            Shuffle(allkeys);
        }

        foreach (int currclient in allkeys)
        {
            // If we are sending to a group of people, then skip the ones we are not sending it to
            if (who == SendDataFlags.PLAYERS && allClients[currclient].starting_position == "Spectator")
            { continue; }

            if (who == SendDataFlags.SPECTATORS && allClients[currclient].starting_position != "Spectator")
            { continue; }

            Client theclient = allClients[currclient];

            // Increment message counter
            theclient.curr_tracked_packet += 1;

            // Create the message
            theclient.server_sent_packets[theclient.curr_tracked_packet] = theclient.curr_tracked_packet.ToString() + GLOBALS.SEPARATOR2 + theclient.curr_tracked_packet.ToString() + GLOBALS.SEPARATOR2 + messageraw;

            // Go through the list and send all packets not sent previously
            // The start of message needs to indicate this is a Flag packet
            StringBuilder outmessage = new StringBuilder();
            outmessage.Append(GLOBALS.HEADER_FLAGS + GLOBALS.SEPARATOR1 );

            // Next add all the individual messages
            bool firstdone = false;
            int i = theclient.client_confirmed_this_msg_id+1;
            if ((theclient.curr_tracked_packet - i ) > GLOBALS.MAX_TRACKED_PACKETS) { i = theclient.curr_tracked_packet - GLOBALS.MAX_TRACKED_PACKETS + 1; }
            if (i> theclient.curr_tracked_packet) { i = theclient.curr_tracked_packet;  } // If some corruption happened the curr_tracked_packet is higher , then still end the latest message

            // Go through all the packets
            for ( ; i <= theclient.curr_tracked_packet; i++)
            {
                // Make sure the packet exists
                if (!theclient.server_sent_packets.ContainsKey(i)) { continue; }

                // Append SEPARATOR if required
                if ( firstdone)  { outmessage.Append(GLOBALS.SEPARATOR3); }
                else            {  firstdone = true;  }
                
                // Add packet
                outmessage.Append(theclient.server_sent_packets[i] + GLOBALS.SEPARATOR2 + "TRACKED_FLAG_ID" + GLOBALS.SEPARATOR2 + theclient.server_confirmed_this_msg_id);            
            }

            // Add the client message we last received
            SendUdp(outmessage.ToString(), theclient.endpoint, Compression);

            // Clean up old messages
            int oldest_packet_needed = theclient.curr_tracked_packet - GLOBALS.MAX_TRACKED_PACKETS + 1;
            List<int> currlist_packetids = theclient.server_sent_packets.Keys.ToList();

            foreach (int currkey in currlist_packetids)
            {
                if ( currkey < oldest_packet_needed)
                {
                    theclient.server_sent_packets.Remove(currkey);
                }
            }
        }

        return true;
    }

    private void DEBUG_print_flags_udp(int currclient, string message)
    {
        
        MyUtils.LogMessageToFile("SC: Client last received: client_confirmed_this_msg_id = " + allClients[currclient].client_confirmed_this_msg_id, true);
        MyUtils.LogMessageToFile("SC: Server current id: curr_tracked_packet = " + allClients[currclient].curr_tracked_packet, true);
        MyUtils.LogMessageToFile("CS: Server last received: server_confirmed_this_msg_id = " + allClients[currclient].server_confirmed_this_msg_id, true);
        MyUtils.LogMessageToFile("Fifo buffer size: server_sent_packets = " + allClients[currclient].server_sent_packets.Count, true);
        MyUtils.LogMessageToFile("  curr_tracked_packet = " + allClients[currclient].curr_tracked_packet, true);

        string[] message_without_header = message.Split(GLOBALS.SEPARATOR1);
        string[] message_top = message_without_header[1].Split(GLOBALS.SEPARATOR3);

        for( int i = 0; i < message_top.Length; i++)
        {
            string[] message_inside = message_top[i].Split(GLOBALS.SEPARATOR2);

            bool failed = false;
            int id;
            if ( !int.TryParse(message_inside[0], out id))
            {
                failed = true;
            }
      
            MyUtils.LogMessageToFile("    MSG ID: " + id, true);

            for (int j = 1; j < message_inside.Length;)
            {
                MyUtils.LogMessageToFile("       KEY[" + message_inside[j++] + "]=" + message_inside[j++], true);
            }

        }
    }


    // Randomize a list
    private static System.Random rng = new System.Random();

    public static void Shuffle(IList<int> list)
    {
        int n = list.Count;
        while (n > 1)
        {
            n--;
            int k = rng.Next(n + 1);
            int value = list[k];
            list[k] = list[n];
            list[n] = value;
        }
    }


    // SendServerInfoUdp:
    // iSends a server info packet
    public bool SendServerInfoUdp(string messageraw, IPEndPoint destination )
    {
        byte[] messageBytes = Encoding.UTF8.GetBytes(GLOBALS.GETSERVERINFO + GLOBALS.SEPARATOR1 + messageraw);
   
        try
        {
            udpSemaphore.WaitOne();
            m_udpClient.Send(messageBytes, messageBytes.Length, destination);

        }
        catch (Exception t)
        {
            MyUtils.LogMessageToFile("exeption when sending server info!", true);
            return false;
        }
        finally
        {
            udpSemaphore.Release();
        }

        return true;
    }
}