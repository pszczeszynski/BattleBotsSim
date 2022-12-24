using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;
using System.IO;
using UnityEngine.SceneManagement;
using Ionic.Zlib;
using UnityEngine.XR;


public class ClientLow : MonoBehaviour
{
    /// There is only one ClientLow in the game, so it's a singeltonf
    [NonSerialized]
    public static ClientLow instance;

    private bool DEBUG = false;

    public bool VR_ENABLED;
    public Scorekeeper scorer;

    Dictionary<String, int> netmonitor = new Dictionary<String, int>();
    string netmonitor_header;

    GameObject top_canvas;

    ApplicationManager application_manager;

    // If found a valid position on a server, set this to true
    bool found_position = false;
    bool holding_mode = false;

    // Frame-rate counter
    public int framecount = 0;
    public int FPS = 0;

    // *************************************************************************
    // ************* Game Object + Scene Manipulation Functions        *********
    // *************************************************************************
    private GameObject big_message = null;

    private Dictionary<int, GameObject> allFieldElements = new Dictionary<int, GameObject>();

    void OnLevelFinishedLoading(Scene scene, LoadSceneMode mode)
    {
        if (scene.name == "fieldElements")
        {   elements_load = true; }

        if (scene.name == "field")
        { 
            field_load = true;
            
            // The field scene wil ldefine the ambient lighting
            SceneManager.SetActiveScene(scene);
        }

        if (scene.name == "Scoring")
        { scorer_load = true; }

        if (scene.name == "MultiPlayer_gui")
        { gui_load = true; }

        if (elements_load && field_load && gui_load && scorer_load && !configuration_done)
        {
            messageLog = GameObject.Find("MessageLogText");

            GameObject details_go = GameObject.Find("DetailsOverlay");
            if (details_go)
            {
                detailsText = details_go.GetComponent<Text>();
            }

            Tournament_msg = GameObject.Find("TournamentMSG");
            Tournament_top = GameObject.Find("Tournament");
            Tournament_ready = Tournament_top.transform.Find("Ready").gameObject;

            Tournament_top.SetActive(false);

            // Find all field elements and put the in our dictionary using their index number
            allFieldElements.Clear();
            GameObject[] allelements;
            allelements = GameObject.FindGameObjectsWithTag("GameElement");

            foreach (GameObject currobj in allelements)
            {
                gameElement currelement = currobj.GetComponent<gameElement>();

                if (allFieldElements.ContainsKey(currelement.id)) 
                {
                    Debug.Log("Field element " + currelement.id + " is not unique id."); 
                }
                else 
                {
                    allFieldElements.Add(currelement.id, currobj);
                }
            }

            // Turn off all physics in game elements and field elements

            ConfigureElements();

            scorer = GameObject.Find("Scorekeeper").GetComponent<Scorekeeper>();
            scorer_overlays = GameObject.Find("ScorerOverlays").transform;

            // Find the canvas for hud identifications
            top_canvas = GameObject.Find("Canvas");
            big_message = GameObject.Find("BIGMESSAGE");
            if( big_message) { big_message.SetActive(false); }


            // Prepare scorer
            scorer.shadowmode = true;
            scorer.ScorerReset();
            configuration_done = true;

            // Prepare overlasys
            PrepOverlays();

            // Set the camera quality level
            Camera[] allcameras = Resources.FindObjectsOfTypeAll<Camera>();

            foreach (Camera currcamera in allcameras)
            {
                MyUtils.SetCameraQualityLevel(currcamera.gameObject);
            }

            MyUtils.QualityLevel_AdjustObjects();
           
            ourgamesettings = FindObjectOfType<GameSettings>();

            if (ourgamesettings) { ourgamesettings.Init(); }

            // Get our application manager
            application_manager = FindObjectOfType<ApplicationManager>();
        }
    }

    private void ConfigureElements()
    {
        // Turn off box collider and rigid body. Interpolation will be done later
        Dictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;

        foreach (GameObject currobj in allelements)
        {
            // Turn off collider
            BoxCollider currcollider = currobj.GetComponent<BoxCollider>();
            if (currcollider != null) { currcollider.enabled = false; }

            // If there are joints of any type, remove them
            Joint myjoint = currobj.GetComponent<Joint>();
            if( myjoint ) { Destroy(myjoint);  }

            Rigidbody currrigid = currobj.GetComponent<Rigidbody>();
            if (currrigid != null) {
                currrigid.detectCollisions = false;
                currrigid.isKinematic = true; // What we used to do, but this still causes issues
                Destroy(currrigid);
            }
        }

        // Field should also have all things turned off
        GameObject[] allobjects;
        allobjects = GameObject.FindGameObjectsWithTag("FieldStructure");

        foreach (GameObject currobj in allobjects)
        {
            BoxCollider currcollider = currobj.GetComponent<BoxCollider>();
            Rigidbody currrigid = currobj.GetComponent<Rigidbody>();

            if (currcollider != null) { currcollider.enabled = false; }
            if (currrigid != null) {
                currrigid.detectCollisions = false;
                currrigid.isKinematic = true;
                Destroy(currrigid);
            }
        }
    }

    private void TurnOnInterpolation()
    {
        // Turn on interpolation of field elements only after connection established
        Dictionary<int, GameObject>.ValueCollection allelements = allFieldElements.Values;

        // However, if there is no interpolation, add it in
        foreach (GameObject currobj in allelements)
        {
            interpolation currinterpolation = currobj.GetComponent<interpolation>() as interpolation;
            if( currinterpolation == null )
            {
                currinterpolation = currobj.AddComponent<interpolation>();            
            }
            currinterpolation.enabled = true;
        }

        interpolation_on = true;
    }

    private void TurnOnInterpolationInObject(GameObject inobject)
    {
        // Iterate through every child of the object
        for (int j = 0; j < inobject.transform.childCount; j++)
        {
            // Get the interpolation, create it if it doesn't exist
            interpolation currint = inobject.transform.GetChild(j).GetComponent<interpolation>();
            if (currint == null)
            {
                currint = inobject.transform.GetChild(j).gameObject.AddComponent<interpolation>();
            }
            currint.enabled = true;

            // If there is a rigidbody,make it kinematic. Can't delete because joints may be dependent.
            Rigidbody myrb = inobject.transform.GetChild(j).GetComponent<Rigidbody>();

            if(myrb)
            {
                // Make it kinematic
                myrb.detectCollisions = false;
                myrb.isKinematic = true;     
            }
        }

        // Don't forget the parent
        interpolation currintp = inobject.GetComponent<interpolation>();
        if (currintp == null)
        {
            currintp = inobject.AddComponent<interpolation>();
        }
        currintp.enabled = true;

        // If there is a rigidbody, turn it off and/or delete it
        Rigidbody rb = inobject.GetComponent<Rigidbody>();

        if (rb)
        {
            // Make it kinematic
            rb.detectCollisions = false;
            rb.isKinematic = true;
        }
    }

    // Update the score field
    private GameObject redtextobj = null;
    private GameObject bluetextobj = null;
    private GameObject field_redscore = null;
    private GameObject field_bluescore = null;

    public void UpdateScore()
    {
        // Get the red score
        if (redtextobj == null)
        { 
            redtextobj = GameObject.Find("REDSCORE");
            field_redscore = GameObject.Find("FIELD_RED");

            // Make sure we found the score
            if (redtextobj == null)
            { return; }
        }

        // Get the blue score
        if (bluetextobj == null)
        { 
            bluetextobj = GameObject.Find("BLUESCORE");
            field_bluescore = GameObject.Find("FIELD_BLUE");
            
            // Make sure we found the score
            if (bluetextobj == null)
            { return; }
        }

        string redscore = "";
        string bluescore = "";

        if (!serverFlags.TryGetValue("REDSCORE", out redscore))
        { return; }

        if (!serverFlags.TryGetValue("BLUESCORE", out bluescore))
        { return; }

        redtextobj.GetComponent<Text>().text = redscore;
        bluetextobj.GetComponent<Text>().text = bluescore;
        if (field_redscore) { field_redscore.GetComponent<TMPro.TextMeshPro>().text = redscore; }
        if (field_bluescore) { field_bluescore.GetComponent<TMPro.TextMeshPro>().text = bluescore; }

        scorer.ReceiveServerData(serverFlags);

    }

    // *************************************************************************
    // ************* LOGS, Status Messages                             *********
    // *************************************************************************


    // ************** User Error Message Line *********************
    private GameObject messageLog;  // Master message log line to copy


    class LogLine
    {
        public GameObject TextLine;
        public long time_of_message;
    };

    private List<LogLine> allmessages = new List<LogLine>();

    public void ShowMessage(string message)
    {
        // Make sure there is a messageLog found
        if (!messageLog) { return; }

        // Create a copy
        LogLine newline = new LogLine();
        newline.TextLine = Instantiate(messageLog, messageLog.transform.parent, false);
        newline.time_of_message = GetTimeMillis();


        // Set the message
        Text textobj = newline.TextLine.GetComponent<Text>();
        textobj.text = message;
        textobj.enabled = true;
        Canvas.ForceUpdateCanvases(); 
        float lineheight = textobj.cachedTextGenerator.lineCount * 25f;

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


    private void UpdateMessage()
    {
        for(int i = allmessages.Count - 1; i >=0; i--)
        {
            LogLine curr_line = allmessages[i];

            // Get rid of message if period is exceeded
            long time_on_screen = GetTimeMillis() - curr_line.time_of_message;
            if ((time_on_screen > GLOBALS.MESSAGE_DISPLAY_PERIOD) && ((allmessages.Count - i) > GLOBALS.MESSAGES_TO_KEEP))
            {
                // Remove from list if it exceeds last stored meesage count
                allmessages.RemoveAt(i);

                // Destroy object
                Destroy( curr_line.TextLine);
                continue;
            }

            // Otherwise start fading it once the time is at 90%
            Text textobj = curr_line.TextLine.GetComponent<Text>();

            if (textobj == null)
            { return; }

            float alpha = 1f;
            textobj.enabled = true;

            if (time_on_screen > 0.9 * GLOBALS.MESSAGE_DISPLAY_PERIOD)
            {
                alpha = (GLOBALS.MESSAGE_DISPLAY_PERIOD - time_on_screen) / (0.1f * GLOBALS.MESSAGE_DISPLAY_PERIOD);
                if (alpha < 0)
                { 
                    alpha = 0;
                    textobj.enabled = false;
                }
                else
                {
                    textobj.enabled = true;
                }
            }
            Color newcolor = new Color(textobj.color.r, textobj.color.g, textobj.color.b, alpha);

            textobj.color = newcolor;
        }
    }

    public void ResetChatCounter()
    {
        for (int i = 0; i < allmessages.Count; i++)
        {
            if((allmessages.Count-i) > GLOBALS.MESSAGES_TO_KEEP) { continue; }

            LogLine curr_line = allmessages[i];
            curr_line.time_of_message = GetTimeMillis();
        }
    }
    // *************************************************************************
    // ************* PROGRAM FLOW CONTROL (Start, Stop, Update, etc..) *********
    // *************************************************************************

    Thread thread_incoming;
    private bool first_time_game_settings = true;

    private bool ClientStart()
    { 
        MyUtils.LogMessageToFile("starting client udp...", false);

        if( ! startUdp() ) { return false;  }

        MyUtils.LogMessageToFile("OK, started udp.",false);

        // Clear globals that may have been populated
        GLOBALS.client_names.Clear();
        GLOBALS.client_ids.Clear();

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

            if (thread_incoming.IsAlive)    { MyUtils.LogMessageToFile("udp thread is alive.", false); }
            else                            { MyUtils.LogMessageToFile("udp thread is NOT alive.... yet...", true); }
        }
        else if (GLOBALS.UDP_ALGORITHM == 2) // 100% serial
        {
            // nothing to do
        }

        instance = this;
        return true;    
    }

    void OnApplicationQuit()
    {
        killme = true;
    }

    static private bool killme = false;

    // OnDisable: close log files
    private void OnDisable()
    {
        killme = true;
        configuration_done = false;

        // Stop the asyncrhonous task
        if (thread_incoming != null)
        {
            thread_incoming.Abort();
        }

        // End the client
        if (m_udpClient != null)
        {
            // Close the udp client
            m_udpClient.Close();
            m_udpClient.Dispose();
            m_udpClient = null;
        }

        // Close any open files
        MyUtils.CloseScorefiles();

        // Clear any recording
        MyUtils.PB_ClearRecording();

        SceneManager.sceneLoaded -= OnLevelFinishedLoading;

        // Unmark client mode
        GLOBALS.CLIENT_MODE = false;
        GLOBALS.topclient = null;

        // Make sure to restore bhelp setting
        GLOBALS.FORCE_OLD_BHELP = false;

        // Clear variables we may have populated
        GLOBALS.client_names.Clear();
        GLOBALS.client_ids.Clear();
    }

    static bool field_load = false;
    static bool elements_load = false;
    static bool scorer_load = false;
    static public bool configuration_done = false;
    static bool gui_load = false;


    // OnEnable: Open log files
    private void OnEnable()
    {
        GLOBALS.LOGS_PATH = ((Application.isEditor) ? "." : Application.persistentDataPath) + Path.DirectorySeparatorChar.ToString() + "logs";

        field_load = false;
        elements_load = false;
        scorer_load = false;
        configuration_done = false;
        gui_load = false;
        connection_state = ConnectionStates.NOTSTARTED;

        // Disable Physics - Do we need it for OVR??? not really..
        Physics.autoSimulation = false;

        // Load field elements
        // OnLevelFinished will trigger when this has finished loading, then we can initialize more data types
        SceneManager.sceneLoaded += OnLevelFinishedLoading;
        SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/fieldElements", LoadSceneMode.Additive);

        // Load GUI Scene
        SceneManager.LoadScene("Scenes/MultiPlayer_gui", LoadSceneMode.Additive);

        // Load field
        SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/field", LoadSceneMode.Additive);

        // Load scoring 
        // SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/Scoring", LoadSceneMode.Additive);

        killme = false;

        // Mark we are in client mode
        GLOBALS.CLIENT_MODE = true;
        GLOBALS.topclient = this;
        GLOBALS.FORCE_OLD_BHELP = false;

        // Clear variables that we want to start clean
        GLOBALS.client_names.Clear();
        GLOBALS.client_ids.Clear();
    }

    private void Start()
    {

    }
    
    void FixedUpdate()
    {

    }

    // Update interval time tracking
    long lastSendingTime = 0;
    bool second_load = false;
    long lastFrameCount = 0;

    public int playback_index = 0;

    void Update()
    {
        playback_index = MyUtils.playback_index;

        // Re-init status variables as required
        players_changed = false;

        // Do FPS Calculation
        framecount += 1;

        // Update data rate every 250ms
        if (MyUtils.GetTimeMillis() - lastFrameCount >= 250)
        {
            FPS = (int) (framecount * 1000 / (MyUtils.GetTimeMillis() - lastFrameCount));
            lastFrameCount = MyUtils.GetTimeMillis();
            framecount = 0;
        }

        // if this is a serial communication method, do it here
        // **** receive any messages ****
        if (!playbackmode && (GLOBALS.UDP_ALGORITHM == 2))
        {
            DataReceiveSerial();
        }

        //* Debug messages
        if (DEBUG)
        {
           /* for (int i = allmessages.Count - 1; i >= 0; i--)
            {
                LogLine curr_line = allmessages[i];
                // Remove from list
                allmessages.RemoveAt(i);
                // Destroy object
                Destroy(curr_line.TextLine);
            }

            string test = "";
            foreach (string key in clientFlags.Keys)
            {
                test = test + key + " = " + clientFlags[key] + "\n";
            }
            ShowMessage(test);
            */
        }

        if (!second_load && gui_load)
        {
            second_load = true;
            // Load scoring elements
            SceneManager.LoadScene("Scenes/" + GLOBALS.GAME + "/Scoring", LoadSceneMode.Additive);
        }


        // Do mouse selection if we are a spectator
        if ((myPosition == "Spectator") || (myPosition.Length < 1) || playbackmode)
        {
            ProcessMouse();
        }

        // Check Connetion(s)
        if (!playbackmode)
        {
            MonitorConnection();
            UpdatePacketLoss();


            // If we lost our connection, then we need to exit
            if ((connection_state == ConnectionStates.LOST) && !killme && !GLOBALS.now_playing)
            {
                ShowMessage("LOST Connection to server!!!");
                MyUtils.LogMessageToFile("Lost connection to server...", true);
                found_position = false;

                connection_state = ConnectionStates.NOTSTARTED;

                ResetAllStates();
                // Will NOT reload scene
                // Best to re-load this scene    
                // killme = true;
                // SceneManager.LoadScene(SceneManager.GetActiveScene().name);         
                return;
            }


            // ********* INCOMING MESSAGE PROCESSING *************
            // Process any received messages as soon as they arrive    

            allReceivedDataSemaphore.WaitOne();
            try
            {
                // If marked for the end, exit
                if (killme) { return; }

                if (allReceivedData.Count > 0)
                {
                    // Process all the messages
                    onReceivedData(ref allReceivedData);//do this from the main
                }
            }
            catch (Exception e)
            {
                MyUtils.LogMessageToFile("Exception occured during update onReceivedData " + e, true);
            }
            finally
            {
                // Clear all the messages
                allReceivedData.Clear();
                allReceivedDataSemaphore.Release();
            }

            GLOBALS.time_after_data_received = MyUtils.GetTimeSinceStart();
        }

        // **** IF activelly playing data, do playback
        if( GLOBALS.now_playing )
        {
            if (GLOBALS.now_paused)
            {
                MyUtils.PB_UpdateDuringPause();
            }
            
            ProcessPlayback();
        }


        // Uodate the following on the update rate
        // Get curr_time in millisensoncds
        long curr_time = GetTimeMillis();
        long elapsed_time = curr_time - lastSendingTime;

        if (elapsed_time < GLOBALS.CLIENT_SEND_UPDATE_DELAY) { return; }
        lastSendingTime = curr_time;

        // Update timer and score
        UpdateScore();

        // Deal with keyboard actions
        DoKeyboardStuff();

        // Update Log Status Files
        MyUtils.DoScoringFiles(serverFlags);

        // Process any scoring file commands if required
        ProcessScoringFileCommands();

        // Do overlays
        ApplyOverlays();

        // If playback mode, process our avatar keys
        if( playbackmode )
        {
            if (use_spec_myRobot || (myRobot_working == spec_myRobot))
            {
                spec_myRobot.updateGamepadVars();
            }
        }

        UpdateMessage();

        // ********* SEND OUTGOING MESSAGES *************
        // Only send update if enough time passed. In essense we're generating our own fixedupate
        // May consider moving into fixed update at some point

        if (connection_state != ConnectionStates.CONNECTED)
        {
            return;
        }

        // Process server flags first
        // ProcessServerFlags(); // They are now processed as they are received

        // Mark that we are updating
        SendMyInputs();
        SendFlags();

    }

    public RobotInterface3D highlited_robot;
    public RobotInterface3D selected_robot;

    private void ProcessMouse()
    {
        if( !main_camera ) { return; }

        Camera thecamera = main_camera.GetComponent<Camera>();
        Ray camera_ray = thecamera.ScreenPointToRay(Input.mousePosition);
      
        RaycastHit hitInfo = new RaycastHit();
        bool hit = Physics.Raycast(camera_ray, out hitInfo, 1000f, 1 << 23, QueryTriggerInteraction.Collide);

        // *************************
        // Highlite robot under mouse
        if (hit)
        {
            Transform hit_transform = hitInfo.transform;

            // See if this is a robot
            RobotInterface3D newbot = hitInfo.transform.GetComponentInParent<RobotInterface3D>();

            // If bot found, deal with it
            if (newbot && (newbot != highlited_robot) )
            {
                if (highlited_robot) { highlited_robot.Highlite(false); }
                highlited_robot = newbot;
                highlited_robot.Highlite(true);
            }
        }
        else
        {
            if (highlited_robot)
            {
                highlited_robot.Highlite(false);
                highlited_robot =  null;
            }
        }

        // *****************************
        // Process left mouse click down event: select robot
        if( !application_manager.IsPointerOverActive() && Input.GetMouseButtonDown(0))
        {
            // Deselect old robot
            if(selected_robot) {  selected_robot.Select(false); }
            selected_robot = highlited_robot;
            if( selected_robot){  selected_robot.Select(true);  }

            // Deal with administrator menu
            UpdateAdminPopupMenu();
        }

        
    }

    // Linked popupmenu
    GameObject popup_menu;
    void UpdateAdminPopupMenu()
    {
        // Find the popup menu
        if( !popup_menu)
        {
            popup_menu = MyUtils.FindGlobal("AdminPopUp");
            if( !popup_menu) { return; }
        }

        if (selected_robot)
        {
            popup_menu.SetActive(true);
        }
        else
        {
            popup_menu.SetActive(false);
        }
    }

    public void CB_Referee(string what_to_do)
    {
        if( !selected_robot || selected_robot.deleted )
        { return;  }

        switch( what_to_do)
        {
            case "kick":
                // Kick Player
                SendChat("/SERVER KICKID=" + selected_robot.myRobotID.id);                
                break;

            case "warn":
                // Warn player
                SendChat("/SERVER MESSAGE=" + selected_robot.myRobotID.id + "=<size=35><color=red>*****</color> WARNING GIVEN <color=red>*****</color></size>");
                break;

            case "reset":
                // Reset player
                SendChat("/SERVER RESET=" + selected_robot.myRobotID.id);
                break;

            case "5s":
                // Start a pin circle on player
                SendChat("/SERVER ROBOTCOUNTER=" + selected_robot.myRobotID.id + "=" + "5");
                break;

            case "clear":
                // Clear Robot circle
                SendChat("/SERVER ROBOTCOUNTERRESET=" + selected_robot.myRobotID.id );
                break;


        }
    }

    private void ProcessScoringFileCommands()
    {
        // See if there are any commands waiting
        List<string> commands = MyUtils.GetScoringFilesCommand();

        if( commands == null ) { return; }

        // Send them to chat
        foreach(string currcommand in commands)
        {
            SendChat(currcommand);
        }

    }
    private void ProcessPlayback()
    {
        Saved_Data currdata = MyUtils.PB_GetNext();
        while(currdata != null)
        {
            ProcessData(currdata.data, currdata.timestamp, true);
            currdata = MyUtils.PB_GetNext();
        }
    }

    public bool IsPlaybackDataEndOfMatch(Saved_Data currdata)
    {
        // Make sure some data exists
        if( currdata.data.Length < 1) { return false; }

        // Check if it is flags
        if(currdata.data[0] != GLOBALS.HEADER_FLAGS) { return false; }

        // Check if it contains the end-of-match flag
        string[] all_flag_messages = currdata.data[1].Split(GLOBALS.SEPARATOR3);
        int message_id;
        int message_id2;

        for (int i = 0; i < all_flag_messages.Length; i++)
        {
            // Split the message to the key, value pair
            string[] id_key_value = all_flag_messages[i].Split(GLOBALS.SEPARATOR2);

            // serverFlagsCopy = id_key_value; 
            if (id_key_value.Length < 4) { continue; } // Make sure there are at least 4 items ( 0 = message id, 1= message id, 2 = first key, 3 = first value
            message_id = int.Parse(id_key_value[0]);
            message_id2 = int.Parse(id_key_value[1]);
            if (message_id != message_id2) { continue; } // Checked id wasn't corrupted

            // Go through all the keys and add them
            for (int j = 2; j < id_key_value.Length;)
            {
                string key = id_key_value[j++];
                string value = id_key_value[j++];

                // We will mark the end of a match with the fireworks command being sent
                if(key == "FIREWORKS") { return true; } 
            }
        }

        return false;
    }

    public bool IsPlaybackDataStartOfMatch(Saved_Data currdata)
    {
        // Make sure some data exists
        if (currdata.data.Length < 1) { return false; }

        // Check if it is flags
        if (currdata.data[0] != GLOBALS.HEADER_FLAGS) { return false; }

        // Check if it contains the end-of-match flag
        string[] all_flag_messages = currdata.data[1].Split(GLOBALS.SEPARATOR3);
        int message_id;
        int message_id2;

        for (int i = 0; i < all_flag_messages.Length; i++)
        {
            // Split the message to the key, value pair
            string[] id_key_value = all_flag_messages[i].Split(GLOBALS.SEPARATOR2);

            // serverFlagsCopy = id_key_value; 
            if (id_key_value.Length < 4) { continue; } // Make sure there are at least 4 items ( 0 = message id, 1= message id, 2 = first key, 3 = first value
            message_id = int.Parse(id_key_value[0]);
            message_id2 = int.Parse(id_key_value[1]);
            if (message_id != message_id2) { continue; } // Checked id wasn't corrupted

            // Go through all the keys and add them
            for (int j = 2; j < id_key_value.Length;)
            {
                string key = id_key_value[j++];
                string value = id_key_value[j++];

                // We will mark the start of a match with the countdown
                if (key == "COUNTDOWN") { return true; }
            }
        }

        return false;
    }

    // Update camera to new positions
    private void LateUpdate()
    {
        UpdateTrackingCamera();
    }


    public static long GetTimeMillis()
    {
        return DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
    }

    // *************************************************************************
    // ************* HEALTH MONITORING OF OUR CONNECTION ***********************
    // *************************************************************************


    // Checks to see if we are still connected. If we don't receive any server communication
    // for timeout period, we consider ourselves disconnected
    private string connectString = "";
    private long time_last_packet = -1;
    private bool interpolation_on = false;
    public enum ConnectionStates
    {
        NOTSTARTED=0,
        CONNECTING,  
        CONNECTED,   // Connected but a player hasn't foudn a valid positio nyet
        LOST
    }
    public ConnectionStates connection_state = ConnectionStates.NOTSTARTED;

    // Task that monitors connection, also retries establishing server connecton
    private long time_last_sent = -1;
    private long time_started = -1;
    private long time_last_count_check = -1;

    private void MonitorConnection()
    {
        // Don't monitor if in playback mode
        if( playbackmode ) { return;  }

        // Initialize time started to determine if we have a server connection timeout
        if (time_started == -1)
        { time_started = GetTimeMillis(); }

        if ((connection_state != ConnectionStates.LOST) &&
            (connection_state != ConnectionStates.NOTSTARTED))
        {
            long curr_time = GetTimeMillis();

            // If we are trying to establish a server connection, do so
            if (connection_state == ConnectionStates.CONNECTING)
            {
                // See if we received back a packet, if so we are connected
                if (time_last_packet > 0)
                {
                    connection_state = ConnectionStates.CONNECTED;
                }
                else
                // Re-send connection request if timeout occured
                if (curr_time - time_last_sent >= GLOBALS.CLIENT_CONNECT_RETRY_TIME)
                {
                    // send the message to the server 
                    sendUdpData(connectString);
                    time_last_sent = curr_time;
                }

            }

            // See if timeout occured
            if (((time_last_packet != -1) && (curr_time - time_last_packet >= GLOBALS.CLIENT_DISCONNECT_TIMEOUT))
                ||
                ((time_last_packet == -1) && (curr_time - time_started >= GLOBALS.CLIENT_DISCONNECT_TIMEOUT)))
            {
                connection_state = ConnectionStates.LOST;
                return;
            }

            // Debugging packet rate information
            if (GLOBALS.ENABLE_UDP_STATS && (curr_time - time_last_count_check > GLOBALS.SERVER_MESSAGE_COUNT_TIME))
            {
                string message = "";
                if (netmonitor != null)
                {
                    foreach (KeyValuePair<string, int> curr in netmonitor)
                    {
                        message += "\n" + curr.Key + "=" + ((float)curr.Value / (float)(curr_time - time_last_count_check) * 1000f);
                    }

                    ShowMessage(message);
                    netmonitor.Clear();
                }
                time_last_count_check = curr_time;
            }
        }
    }
    // *************************************************************************
    // ************* GUI Related Callbacks               ***********************
    // *************************************************************************
    // *************************************************************************

    // called when player hits connect
    public string myStartfield = "";
    public bool Connect(string ipfield, string portfield, string pName, string passfield, string startfield)
    {
        // Reset all state variables in this class to prepare for new connection
        ResetAllStates();

        // Check if name is entered
        if (pName == "")
        {
            ShowMessage("You must enter a name!");
            return false;
        }

        // Check if we're already trying to connect (someone hit Connect twice??)
        if (connection_state != ConnectionStates.NOTSTARTED) { return true; }

        ourPlayerName = pName;

        // Check if ip field needs to be reset
        if (ipfield != "")
        {
            serverIP = ipfield;
        }

        int finaludpport = 0;

        // Check if port field needs to be reset
        if (portfield.Length > 0 && Int32.TryParse(portfield, out finaludpport ))
        {
            serverPORT = finaludpport;
        }

        // Create a UDP connection to server
        if (!ClientStart())
        {
            ShowMessage("Unable to establish a UDP connection to server!");
            return false;
        }

        // Clear connection variables
        time_last_packet = -1;  // Marks that we haven't received any packets yet
        interpolation_on = false; // Mark that interpolation is still off, to be done once we get a connection
        first_time_game_settings = true;

        // Check if we have a license for our robot model
        if (!LicenseData.CheckRobotIsUnlocked(scorer.CorrectRobotChoice(GLOBALS.RobotModel), GLOBALS.robotskins))
        {
            GLOBALS.robotskins = "Default";
        }

        string RobotModel = ((startfield == "Spectator") || (startfield == "Admin")) ? "AvatarSpectator" : GLOBALS.RobotModel; // Robot name needs to be different from position or else GameObject.Find screws up...

        // Request to connect
        // Include:
        // Player name, password, start-location, etc...
        connectString = GLOBALS.HEADER_IN_NEWPLAYER + GLOBALS.SEPARATOR1 + 
                        pName + GLOBALS.SEPARATOR1 + 
                        passfield + GLOBALS.SEPARATOR1 + 
                        startfield + GLOBALS.SEPARATOR1 + 
                        RobotModel + GLOBALS.SEPARATOR1 +
                        GLOBALS.GAME + GLOBALS.SEPARATOR1 +
                        Application.version + GLOBALS.SEPARATOR1 +
                        GLOBALS.DriveTrain + GLOBALS.SEPARATOR1 +
                        GLOBALS.speed + GLOBALS.SEPARATOR1 +
                        GLOBALS.acceleration + GLOBALS.SEPARATOR1 +
                        GLOBALS.weight + GLOBALS.SEPARATOR1 +
                        GLOBALS.turning_scaler + GLOBALS.SEPARATOR1 +
                        GLOBALS.fieldcentric + GLOBALS.SEPARATOR1 +
                        GLOBALS.activebreaking + GLOBALS.SEPARATOR1 +
                        GLOBALS.tankcontrol + GLOBALS.SEPARATOR1 +
                        GLOBALS.skins + GLOBALS.SEPARATOR1 +
                        GLOBALS.robotskins + GLOBALS.SEPARATOR1
                        ;

        // Set connection_state
        connection_state = ConnectionStates.CONNECTING;


        // Do first iteration of connection monitoring: it will send the first connection request
        time_started = -1;
        MonitorConnection();

        // ***** Can't set camera here since server may overide
        // Set the camera position/rotation
        // SetCamera(startfield);

        // Record our starfield for admin checking
        myStartfield = startfield;

        return true;
    }

    public bool playbackmode = false;
    string oldPlayerName;
    public void InitPlaybackMode()
    {
        playbackmode = true;

        // Need to set all the cameras and ourselves as spectator
        oldPlayerName = ourPlayerName;
        ourPlayerName = "Spectator";
        SpawnPlayer("Spectator", "-1", "AvatarSpectator", "Spectator");

    }

    // Do stuff when playback is stopped
    public void Playback_Stopped()
    {
        // Restore our player name
        ourPlayerName = oldPlayerName;

        // Stop ALL objects
        foreach( interpolation currobj in FindObjectsOfType<interpolation>())
        {
            currobj.StopMovement();
        }

        // Main thing is to force camera to change back
        if (myPosition != "Spectator")
        {
            myRobot_working = myRobot_saved;
            DoKeyboardStuff(true);
        }
    }

    // Process key events that single player deals with
    private int spectator_curr_camera_player = 0;  // the player position (elementAt()) in player dictionary. -1 = Main camera, 0 = 1st item. 

    bool key_space_last = false;
    private void DoKeyboardStuff(bool force_camera_change = false)
    {
        if (!force_camera_change && GLOBALS.keyboard_inuse) { return; }  // exit if someone else has keyboard focus


        //  Was space-bar pressed? (NOTE: the Input.GetKeyDown method was falsly missing events - maybe some frames were being cut short and not getting here? Dunno....
        bool toggle_camera = (Input.GetKey(KeyCode.Space) || GLOBALS.JoystickMap["Jcontrolls_camera"].GetButton() ) && !key_space_last;
        key_space_last = Input.GetKey(KeyCode.Space) || GLOBALS.JoystickMap["Jcontrolls_camera"].GetButton();

        // Check whether to cycle camera
        if (force_camera_change || toggle_camera)
        {
            // Exit if valid cameras aren't present
            if ((main_camera == null) || (player_camera == null)) { return; }

            // Disable spectator inputs - this specator is only used in playback mode
            if (spec_myRobot) { 
                use_spec_myRobot = false;
                spec_myRobot.disable_motion = true;
            }

            // If we are a sepctator, we will cycle camera through all the players as well as the main camera
            // However if a player is selected, then this will cycle between main camera and that selected player
            if ((myPosition == "Spectator") || GLOBALS.now_playing)
            {
                // Too bad dictionary doesn't allow access by index #... thus we have to
                // create a list of keys before accessing them by index
                List<int> keys = new List<int>(players.Keys);

                // If we have a selected robot, assign it to the next cycle view, otherwise return to main camera if already assigned
                // Ignore if no selected robot
                if (selected_robot && selected_robot.myRobotID)
                {
                    if( keys.Contains(selected_robot.myRobotID.id) )
                    {
                        int selected_index = keys.IndexOf(selected_robot.myRobotID.id);
                        if(spectator_curr_camera_player == selected_index )
                        {
                            spectator_curr_camera_player = -2;
                        }
                        else
                        {
                            spectator_curr_camera_player = selected_index;
                        }
                    }
                }
                else
                {
                    // Increment spectatorindex to next person
                    spectator_curr_camera_player += 1;
                }



                // First check that the id exists, if not, set the spectator back to main camera
                if ( players.Count < (spectator_curr_camera_player+1) )
                {
                    if (spectator_cam2 || GLOBALS.now_playing)
                    {
                        spectator_curr_camera_player = -2;
                    }
                    else
                    {
                        spectator_curr_camera_player = -1; // Set to -1 do transfer to main camera, set to 0 just to keep cycling through players.
                                                           // Since we are a player, we do not need to set the camera to -1, but it does add the extra camera angle to the cycling
                                                           // Also, as of now, we aren't a body to this position
                    }
                }

                // Next, get the correct camera assigned
                // If currcamera == -2 and we are in now_playing mode (and are not a spectator), then go to else condition and deal with spectator camera
                if ((spectator_curr_camera_player < 0) && ((myPosition == "Spectator") || (spectator_curr_camera_player != -2)))
                {
                    GLOBALS.camera_follows = false;

                    if (spectator_cam2 && (spectator_curr_camera_player == -2))
                    {
                        main_camera.transform.position = spectator_cam2.transform.position;
                        main_camera.transform.rotation = spectator_cam2.transform.rotation;
                    }
                    else
                    {
                        main_camera.transform.position = spectator_cam1.transform.position;
                        main_camera.transform.rotation = spectator_cam1.transform.rotation;
                    }

                    myRobot_working = null;
                }
                else
                {
                    // Make sure it will re-create the tracker 
                    GLOBALS.camera_follows = true;
                    robot_ref = null;

                    // Extract the next item in the list of keys.

                    // If this is us, then set camera back to spectator
                    //if(keys[spectator_curr_camera_player] == ourClientId)
                    //{
                    //    GLOBALS.camera_follows = false;
                    //    main_camera.transform.position = player_camera.transform.position;
                    //    main_camera.transform.rotation = player_camera.transform.rotation;
                    //    myRobot = null;
                    //    return;
                    //}

                    // Assign spectator camera in this special case
                    if (spectator_curr_camera_player < 0)
                    {
                        SpawnSpectator();
                        use_spec_myRobot = true;
                        spec_myRobot.disable_motion = false;
                    }
                    else
                    {
                        // Get the players RobotInterface3D into myRobot which updateTrackingCamera 
                        myRobot_working = players[keys[spectator_curr_camera_player]].robot;

                        // If the current robot is the spectator, then enable control
                        if( myRobot_working == spec_myRobot)
                        {
                            spec_myRobot.disable_motion = false;
                        }
                    }

                    if (mycameratracker)
                    {
                        mycameratracker.transform.parent = null;
                        Destroy(mycameratracker);
                        mycameratracker = null;
                    }

                    // Reset main camera
                    main_camera.transform.rotation = Quaternion.identity;
                }

                // Correct for worldscale
                Vector3 newpos = main_camera.transform.position;
                newpos /= GLOBALS.worldscale / 2f;
                main_camera.transform.position = newpos;

                return;
            }
        
            // Non spectator camera...
            // Check that the working and saved copies are the same
            // Need this when we come out of playback mode
            if( myRobot_working != myRobot_saved)
            {
                myRobot_working = myRobot_saved;
            }

            GLOBALS.camera_follows = !GLOBALS.camera_follows;

            if (scorer)
            {
                scorer.OnCameraViewChanged(); // Let scorer know we are attempting a camera view change
            }

            DoCameraViewChanged();
        }
     
    }

    public void DoCameraViewChanged()
    {
        // Reset camera to starting position
        if (!GLOBALS.camera_follows)
        {
            main_camera.transform.position = player_camera.transform.position;
            main_camera.transform.rotation = player_camera.transform.rotation;
            if (vr_camera)
            {
                vr_camera.transform.position = vr_starting_pos;
                vr_camera.transform.rotation = vr_starting_rot;
            }

            // Correct for worldscale
            MyUtils.AdjustCameraForScale(main_camera, player_camera.transform);

        }
        else /* if (robot_ref != null) */
        {
            // Make sure it will re-create the tracker 
            robot_ref = null;
            if (mycameratracker)
            {
                mycameratracker.transform.parent = null;
                Destroy(mycameratracker);
            }
            mycameratracker = null;
        }
    }


    // Set the camera position/rotation
    GameObject player_camera; // Set to spectator camera if we are a spectator
    GameObject spectator_cam1; // Set to secondary spectator cam is present
    GameObject spectator_cam2; // Set to secondary spectator cam is present

    public GameObject main_camera;
    private GameObject vr_camera;
    private Vector3 vr_starting_pos;
    private Quaternion vr_starting_rot;


    private void SetCamera(string RobotPosition)
    {
        // Set the camera position/rotation
        player_camera = GameObject.Find(RobotPosition + " Cam");
        if (player_camera == null)
        {
            player_camera = GameObject.Find("Spectator Cam");            
        }

        //if( RobotPosition == "Spectator")
        spectator_cam1 = GameObject.Find("Spectator Cam");
        spectator_cam2 = GameObject.Find("Spectator Cam 2");

        if (!main_camera)
        {
            ApplicationManager mymanager = GameObject.FindObjectOfType<ApplicationManager>();
            if (mymanager)
            {
                main_camera = mymanager.MainCamera;
            }
        }

        if (MyUtils.XR_isPresent())
        {
            if (!vr_camera)
            {
                ApplicationManager mymanager = GameObject.FindObjectOfType<ApplicationManager>();
                if (mymanager)
                {
                    vr_camera = mymanager.VRCamera;
                }
            }
        }
        else
        {
            vr_camera = null;
        }



        // vr_camera = main_camera;
        GameObject vr_camera_scaling = GameObject.Find("OVRCameraScaling");

        MyUtils.SetCameraQualityLevel(main_camera);


        // Set non-VR camera
        if ((main_camera != null) && (player_camera != null))
        {
            // Turn of orbit script
            cameraOrbit orbitscript = main_camera.GetComponent<cameraOrbit>();
            if (orbitscript != null)
            { orbitscript.enabled = false; }

            // Set view point to target
            main_camera.transform.position = player_camera.transform.position;
            main_camera.transform.rotation = player_camera.transform.rotation;

            // Scale positions, fov based on world scaler
            // Our unity world is 2X real-world, so divide world-scale by 2
            // Y (height) just directly scales up

            MyUtils.AdjustCameraForScale(main_camera, player_camera.transform);

            //
            //Vector3 newpos = main_camera.transform.position;
            //newpos /= GLOBALS.worldscale / 2f;
            // main_camera.transform.position = newpos;

            // Field of view is a bit more complicated: need to calculate the angle difference required to maintain the same scope of objects..
            // Did a curve fit to approximate good choices near nominal setting
            //float newfov = 60f * Mathf.Pow(2f / GLOBALS.worldscale, -1.15f); // sets fov = 60 for worldscale = 2
            //main_camera.GetComponent<Camera>().fieldOfView = newfov;

        }

        // Set VR camera
        if ((vr_camera != null ) && (player_camera != null))
        {
            // Set proper scaling
            vr_camera_scaling.transform.localScale = (new Vector3(1f, 1f, 1f)) * 2f / GLOBALS.worldscale;

            // Set view point to target with a y of 0 
            Vector3 vr_cam_pos = player_camera.transform.position;
            vr_cam_pos.y = 0;
            vr_camera.transform.position = vr_cam_pos;


            Quaternion camera_rotation = player_camera.transform.rotation;
            Vector3 euler = camera_rotation.eulerAngles;
            euler.x = 0;
            euler.z = 0;
            camera_rotation.eulerAngles = euler;

            vr_camera.transform.rotation = camera_rotation;

            // Set floor height in the tracking space
            Vector3 vr_yheightpos = vr_camera_scaling.transform.localPosition;
            vr_yheightpos.y = MyUtils.GetFloorYPos();
            vr_camera_scaling.transform.localPosition = vr_yheightpos;

            // vr_camera.transform.position *= GLOBALS.worldscale / 2f;

            vr_starting_pos = vr_camera.transform.position;
            vr_starting_rot = vr_camera.transform.rotation;

            /*
            // Set view point to target
            // Correct position for floor-standing person
            Vector3 camera_pos = player_camera.transform.position;
            camera_pos.y = MyUtils.GetFloorYPos();
            vr_camera.transform.position = camera_pos;


            // Correct the rotation to just look straight
            Quaternion camera_rotation = player_camera.transform.rotation;
            Vector3 euler = camera_rotation.eulerAngles;
            euler.x = 0;
            euler.z = 0;
            camera_rotation.eulerAngles = euler;

            vr_camera.transform.rotation = camera_rotation;
            vr_camera.transform.position *= GLOBALS.worldscale / 2f;

            // Fix the y position
            Vector3 campos = vr_camera.transform.position;
            campos.y = MyUtils.GetFloorYPos();
            vr_camera.transform.position = campos;
            vr_camera_scaling.transform.localScale = (new Vector3(1f, 1f, 1f)) * 2f / GLOBALS.worldscale;

            vr_starting_pos = vr_camera.transform.position;
            vr_starting_rot = vr_camera.transform.rotation;
            */
        }
    }

    // For camera tracking, ref positions for camera and robot to calculate offsets
    private Transform robot_ref = null;
    private Transform camera_ref = null;
    private GameObject mycameratracker = null; // use for calculating new camera offset


    // Updates tracking camera if applicable
    private void UpdateTrackingCamera()
    {
        RobotInterface3D myRobot_curr = (use_spec_myRobot) ? spec_myRobot : myRobot_working;

        if (!GLOBALS.camera_follows || !main_camera  || !myRobot_curr || GLOBALS.CAMERA_COUNTDOWN_CONTROL) { return; }

        if (holding_mode && (myPosition != "Spectator"))
        {
            if (player_camera && player_camera.transform && main_camera.transform)
            {
                main_camera.transform.position = player_camera.transform.position;
                main_camera.transform.rotation = player_camera.transform.rotation;
                robot_ref = null;
            }

            return;
        }

        // I am spoectator if I am a spectator and not following another bot
        bool i_am_spectator_bot = (myPosition == "Spectator") && !(GLOBALS.camera_follows && myRobot_working.myRobotID);

        // Initialize new camera tracker if it doesn't exist
        if (!robot_ref)
        {
            GameObject robotref = GameObject.Find("robot_ref");
            GameObject cameraref = GameObject.Find("camera_ref");

            if( !robotref || !cameraref ) { return; }

            robot_ref = robotref.transform;
            camera_ref = cameraref.transform;

            if( !robot_ref || !camera_ref ) { return; }

            mycameratracker = new GameObject();

            // The camera_ref and robot_ref are based on 0 rotation blocks and +z defines the robot forward movement.
            if (!i_am_spectator_bot)
            {
                // No longer do worldscale adjuststments here
                // Vector3 camera_pos_offset = (camera_ref.position - robot_ref.position) * 2f / GLOBALS.worldscale;
                Vector3 camera_pos_offset = (camera_ref.position - robot_ref.position);
                mycameratracker.transform.position = camera_pos_offset;
                Quaternion camera_angle = mycameratracker.transform.rotation;
                camera_angle.eulerAngles = (camera_ref.rotation.eulerAngles - robot_ref.rotation.eulerAngles);

                // Do some tilt correction based on scaling
                //Vector3 euler_angles = camera_angle.eulerAngles;
                //euler_angles.x -= (GLOBALS.worldscale - 2f) * 5f;
                // camera_angle.eulerAngles = euler_angles;
                mycameratracker.transform.rotation = camera_angle;

                // Now correct it for the z-axis/x-axis definition differences
                Vector3 ref_point = new Vector3(0f, 0f, 0f);
                Vector3 axis_ref = new Vector3(0f, 1f, 0f);
                mycameratracker.transform.RotateAround(ref_point, axis_ref, 90f);
            }
            else
            {
                mycameratracker.transform.localRotation = Quaternion.Euler(0, 90f, 0);
            }
            Quaternion final_camera_angle = mycameratracker.transform.rotation;

            // Now parent the tracker to the body, (which will apply body rotations to it)
            // Need to save local positions/orientations and re-apply after parenting (parenting changes it all)
            Vector3 localpos = mycameratracker.transform.localPosition;
            Vector3 localscale = mycameratracker.transform.localScale;
            Quaternion localrot = mycameratracker.transform.localRotation;

            // Now set the parent which screws everything up
            if (myRobot_curr.rb_body == null) { return; }
            mycameratracker.transform.SetParent(myRobot_curr.rb_body.transform, false);

            // Ok now reset lcoals
            mycameratracker.transform.localPosition = localpos;
            mycameratracker.transform.localScale = localscale;
            mycameratracker.transform.localRotation = localrot;

            // Correct main camera to the new x,z angles
            main_camera.transform.rotation = localrot; // Setting camera rotation to the tracker's original local (un-rotated) to initialize correct x,z ang;es

            // Problem is if body wasn't level, then we just applied body non-level rotation to the camera, but we only want to apply Y axis rotation... Therefore reset x,z rotations to original
            // Copy over the orientation (required to set the z,x rotations which won't be updated after this
            // But we do need to rome any robot tilt that we dont want
            /* camera_angle = mycameratracker.transform.rotation;

              // Reset angles x,z
              Vector3 eulers = camera_angle.eulerAngles;
              eulers.x = final_camera_angle.eulerAngles.x;
              eulers.z = final_camera_angle.eulerAngles.z;
              camera_angle.eulerAngles = eulers;
              main_camera.transform.rotation = camera_angle;
            */
            if (vr_camera && vr_camera.transform)
            {
                vr_camera.transform.rotation = mycameratracker.transform.rotation;
            }
        }

        if( !mycameratracker || !mycameratracker.transform ) { return; }

        // Now copy transform info over to main camera
        Vector3 temp_pos = mycameratracker.transform.position;

        // Fix Y position, but not for spectator when viewing himself
        if (!i_am_spectator_bot) {
            // temp_pos.y = camera_ref.position.y * (1f + (2f - GLOBALS.worldscale) * 0.22f);  // Fix the y position
            temp_pos.y = camera_ref.position.y;
        } // Fix the y position

        Vector3 old_pos = main_camera.transform.position;

        float averaging = GLOBALS.CAMERA_AVERAGING;


        main_camera.transform.position = old_pos + (temp_pos-old_pos)/ averaging;

        // But rotation we only want to change Y
        Vector3 camera_rotation = mycameratracker.transform.rotation.eulerAngles;

        // But don't limit for Spectator
        if (!i_am_spectator_bot)
        {
            camera_rotation.x = main_camera.transform.rotation.eulerAngles.x;
            camera_rotation.z = main_camera.transform.rotation.eulerAngles.z;
        }


        Quaternion old_rot = main_camera.transform.rotation;
        Quaternion new_rot = Quaternion.Euler(camera_rotation);


        /*
        // Get the difference in angle
        Quaternion difference_angle = Quaternion.Euler(camera_rotation) * Quaternion.Inverse(temp_quat);

        // Do angle averaging: divide angle by half
        float angle;
        Vector3 axis = new Vector3(0,1f,0);
        difference_angle.ToAngleAxis(out angle, out axis);
        angle = (float) MyUtils.AngleWrap(angle);
        angle *= 1f;
        

        // Calculate final angle
        main_camera.transform.rotation = Quaternion.AngleAxis(angle, axis) * Quaternion.Euler(camera_rotation);
        */
        // Average in angle
        if (averaging <= 1.1f)
        {
            main_camera.transform.rotation = new_rot;
        }
        else
        {
            main_camera.transform.rotation = Quaternion.Lerp(old_rot, new_rot, 1f / averaging);
        }

        // Adjust for worldscale
        MyUtils.AdjustCameraForScale(main_camera, main_camera.transform, myRobot_curr.rb_body.transform);

        if (vr_camera && vr_camera.transform)
        {
            temp_pos.y = MyUtils.GetFloorYPos(); // Camera references feet, thus lower it to feet position
            vr_camera.transform.position = temp_pos;
            vr_camera.transform.rotation = new_rot;
        }
    }


    private Dropdown flagrequest_menu;
    public void FlagRequest(Dropdown menu)
    {
        // Save the menu for resetting later
        flagrequest_menu = menu;

        // Make sure we are connected 
        if( ourClientId == -1 )
        {
            FlagReset();
            return;
        }

        switch (menu.value)
        {
            case 0:
                // This is a nothing condition, thus do nothing
                return;
            case 1:
                // Request from server to restart-all
                if (!holding_mode)
                {
                    clientFlags["RESTARTALL"] = "1";
                }
                return;

            case 2:
                // Request from server to reset position
                RequestPosReset();
                return;
        }
    }

    // RESETMYPOS request
    public bool RequestPosReset()
    {
        // Request from server to reset position
        if (!holding_mode)
        {
            clientFlags["RESETMYPOS"] = "1";
            return true;
        }

        return false;
    }

    // Reset the dropdown menu flag request (if present)
    private void FlagReset()
    {
        if (flagrequest_menu == null) { return; }

        flagrequest_menu.value = 0;
        flagrequest_menu.itemText.text = flagrequest_menu.options[0].text;

    }

    // Returns -1 if not connected, 0 if it's trying to connect, 1 if it's connected
    public int GetConnectionStatus()
    {
 
        switch (connection_state)
        {
            case ConnectionStates.LOST:
            case ConnectionStates.NOTSTARTED:
                return -1;
            case ConnectionStates.CONNECTING:
                return 0;
            case ConnectionStates.CONNECTED:
                if( found_position) { return 1;  }
                else { return 0; }
            default:
                return -1;
        }
    }

    // private int message_id = 0;
    public void SendChat(string msg)
    {
        // Process chat meesages for commands
        if (msg.StartsWith("/SET ") )
        {
            string substring = msg.Substring(5);
            string[] split = substring.Split('=');

            // Clean up split[0]
            split[0] = split[0].TrimEnd();

            if( split[0] == "OUTPUT_SCORE_FILES")
            {
                GLOBALS.OUTPUT_SCORING_FILES = true;
                if( split.Length > 1)
                {
                    MyUtils.status_file_dir = split[1];
                }
                return;
            }
            if (split[0] == "TURN_OFF_HUD")
            {
                if (top_canvas) { top_canvas.SetActive(false); }
                return;
            }
            if (split[0] == "TURN_ON_HUD")
            {
                if (top_canvas) { top_canvas.SetActive(true); }
                return;
            }
            if (split[0] == "FIREWORKS_BLUE")
            {
                scorer.StartFireworks(false);
                return;
            }
            if (split[0] == "FIREWORKS_RED")
            {
                scorer.StartFireworks(true);
                return;
            }
            if (split[0] == "SHOW_VS")
            {
                scorer.ShowGameStartOverlayNoAnimation(true);
                return;
            }
            if (split[0] == "HIDE_VS")
            {
                scorer.ShowGameStartOverlayNoAnimation(false);
                return;
            }
            if (split[0] == "HIDE_CHAMPS")
            {
                DisableChampsMode();
                return;
            }

        }
        else
        {
            clientFlags["CHAT"] = msg;
            // message_id += 1;
        }
    }

    // Turns off the HUD display (press ~ - chat to re-engage)
    public void Toggle_HUD()
    {
        if (top_canvas) { top_canvas.SetActive(!top_canvas.activeSelf); }

        // Turn off server window if preset
        // Turn off server info window if present
        GameObject serverinfowindow = GameObject.Find("ServerInfoWindow");
        if (serverinfowindow)
        {
            serverinfowindow.SetActive(false);
        }
    }

    // OVERLAYS

    // Prep Overlays: after all levels are loaded, finds any necessary compontents
    public GameObject name_parent;
    public Transform name_rl;
    public Transform name_rc;
    public Transform name_rr;
    public Transform name_bl;
    public Transform name_bc;
    public Transform name_br;
    public Transform scorer_overlays;

    public void PrepOverlays()
    {
        // Find the parent ovjects (is active hence findable)
        name_parent = GameObject.Find("/Canvas2/Names");
        if( ! name_parent ) { return;  }

        // Find disabled children
        name_rl = name_parent.transform.Find("RL");
        name_rc = name_parent.transform.Find("RC");
        name_rr = name_parent.transform.Find("RR");
        name_bl = name_parent.transform.Find("BL");
        name_bc = name_parent.transform.Find("BC");
        name_br = name_parent.transform.Find("BR");
    }

    public int overlay_mode = 0; // 0 = off, >0 is some version of overlay
    private bool old_keys_combo_overlay = false;

    public int overlay_details = 0;  // 0 = off, 1 = FPS, 2 = score-details, 3 = FPS + score details
    private bool old_keys_combo_overlay2 = false;

    private Text detailsText = null;

    private string old_overlay_string = "";

    public void ApplyOverlays()
    {
        // Make sure there is a main camera
        if (main_camera == null) { return; }
        if (name_parent == null) { return; }

        // ****** DETECT NAME OVERLAY
        bool keys_combo_overlay = Input.GetKey(KeyCode.Alpha1) && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift));


        // Check keycode for name overlays
        if (!GLOBALS.keyboard_inuse && keys_combo_overlay && !old_keys_combo_overlay)
        {
            overlay_mode++;

            // If we reached the end of al lthe overlay modes, then turn it off
            if (overlay_mode > 2)
            { overlay_mode = 0; }
        }

        old_keys_combo_overlay = keys_combo_overlay;

        // ****** DETECT FPS + score details overlay
        keys_combo_overlay = Input.GetKey(KeyCode.Alpha2) && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift));

        // Check key-code for FPS overlay and score details
        if (!GLOBALS.keyboard_inuse && keys_combo_overlay && !old_keys_combo_overlay2)
        {
            overlay_details++;

            // If we reached the end of al lthe overlay modes, then turn it off
            if (overlay_details > 3)
            { overlay_details = 0; }
        }

        old_keys_combo_overlay2 = keys_combo_overlay;


        // *********** APPLY OVERLAY names
        // First disable all overlays
        name_rl.gameObject.SetActive(false);
        name_rc.gameObject.SetActive(false);
        name_rr.gameObject.SetActive(false);
        name_bl.gameObject.SetActive(false);
        name_bc.gameObject.SetActive(false);
        name_br.gameObject.SetActive(false);

        if (overlay_mode != 0)
        {

            // Now go through all people and find their positions
            foreach (Player currplayer in players.Values)
            {
                // Drop is a spectator
                if (!currplayer.robot || currplayer.robot.isSpectator) { continue; }

                // See if it's in the field of view
                if (currplayer.robot.rb_body)
                {

                    switch (currplayer.robot.myRobotID.starting_pos)
                    {
                        case "Red Left":
                            SetOverlayNamePos(currplayer, name_rl, 0);
                            break;

                        case "Red Center":
                            SetOverlayNamePos(currplayer, name_rc, 2);
                            break;

                        case "Red Right":
                            SetOverlayNamePos(currplayer, name_rr, 4);
                            break;

                        case "Blue Left":
                            SetOverlayNamePos(currplayer, name_bl, 1);
                            break;

                        case "Blue Center":
                            SetOverlayNamePos(currplayer, name_bc, 3);
                            break;

                        case "Blue Right":
                            SetOverlayNamePos(currplayer, name_br, 5);
                            break;

                    }
                }
            }
        }


        // *********** APPLY OVERLAY FPS + score details
        if( !detailsText) { return; } // Make sure a text object was found

        if( detailsText.text.Length > 0 )  // Clear existing string
        {
            detailsText.text = "";
        }

        // Add overlay info
        if (overlay_details != 0)
        {
            // Add FPS
            if( (overlay_details == 1) || (overlay_details == 3))
            {
                detailsText.text += "FPS=" + FPS;
            }

            // Add score_files
            if ((overlay_details == 2) || (overlay_details == 3))
            {
                string details = "";
                foreach (string key in MyUtils.score_details.Keys)
                {
                    // Do everything except OPR, save that for last
                    if( key == "OPR") { continue; }

                    if( GLOBALS.ScorefilesDescription.ContainsKey(key) )
                    {
                        details = GLOBALS.ScorefilesDescription[key];
                    }
                    else
                    {
                        details = key;
                    }

                    detailsText.text += "\n" + details + "=" + MyUtils.score_details[key];
                }

                // Now deal with OPR
                if(MyUtils.score_details.ContainsKey("OPR"))
                {
                    detailsText.text += "\n" + GLOBALS.ScorefilesDescription["OPR"] + "     \n" + MyUtils.score_details["OPR"];

                }

            }
        }

        // Add any scorekeeper overlays
        // Intended give some robot settings for the current robot
        // These will be referenced to the bottom Left
        if( myRobot_working && scorer_overlays && myRobot_working.myRobotID)
        {         
            int id = myRobot_working.myRobotID.id;
            String new_overlay_string = scorer.GetOverlaysString(id);

            // Update overlays if they need to change
            if (! old_overlay_string.Equals(new_overlay_string) )
            {
                // Replace old overlays if a change occured
                foreach (Transform child in scorer_overlays.transform)
                {
                    Destroy(child.gameObject);
                }

                Transform scorekeeper_overlays = scorer.GetOverlays(id, scorer_overlays);

             /*
                if (scorekeeper_overlays)
                {
                    Vector3 oldpos = scorekeeper_overlays.localPosition;
                    Quaternion oldrot = scorekeeper_overlays.localRotation;
                    scorekeeper_overlays.SetParent(scorer_overlays);
                    scorekeeper_overlays.localPosition = oldpos;
                    scorekeeper_overlays.localRotation = oldrot;
                }
                */
            }
            old_overlay_string = new_overlay_string;
        }
    }


    public Vector3 screen_pos;
    private void SetOverlayNamePos(Player currplayer, Transform name_tr, float pos_offset )
    {
        name_tr.GetComponent<TMPro.TextMeshPro>().text = currplayer.playerName;

        Vector3 namepos = name_tr.localPosition;
        screen_pos = main_camera.GetComponent<Camera>().WorldToScreenPoint(currplayer.robot.rb_body.transform.position);
        
        // If this is behind us, then dont show it
        if( screen_pos.z < 0) { return; }

        // Translate it to a percentage of screen x,y
        float x_percentage = (screen_pos.x - main_camera.GetComponent<Camera>().pixelWidth / 2f) / ((float) main_camera.GetComponent<Camera>().pixelWidth);
        float y_percentage = (screen_pos.y - main_camera.GetComponent<Camera>().pixelHeight / 2f) / ((float)main_camera.GetComponent<Camera>().pixelHeight);
        
        namepos.x = x_percentage * name_parent.GetComponent<RectTransform>().rect.width;

        // If we enable y location as well, then add this in


        if (overlay_mode == 1) { namepos.y = name_parent.GetComponent<RectTransform>().rect.height / 2f - GLOBALS.OVERLAY_NAME_Y_OFFSET - pos_offset * GLOBALS.OVERLAY_NAME_Y_INCREMENT; }
        else { namepos.y = y_percentage * name_parent.GetComponent<RectTransform>().rect.height - 10f; }

        name_tr.localPosition = namepos;

        name_tr.gameObject.SetActive(true);
    }

    // *************************************************************************
    // ************* Outgoing Messages Generation        ***********************
    // *************************************************************************
    // *************************************************************************

    // List of all variables/flags we keep track of
    private Dictionary<string, string> clientFlags = new Dictionary<string, string>();

    // Destination IP
    public static string serverIP = "127.0.0.1";
    public static int serverPORT = GLOBALS.UDP_PORT;

    // Our name to help identify our unique ID
    // TBD: May want to rely on server-client direct message to get ID and use this
    // only as a backfall (if UDP packet didn't arrive).
    public static string ourPlayerName = "newbie"; //our name

    private void SendMyInputs()
    {
        // Get the interface object from the robot
        if( myPlayer == null ) { return; }

        if (myRobot_working == null) { return; }

        // Inputs if in playback mode or spectator
        if (GLOBALS.now_playing) { 
            if( use_spec_myRobot || (myRobot_working == spec_myRobot))
            {
                spec_myRobot.updateGamepadVars();
            }
            return;
        }

        // don't override flipsy's gamepad varaibles since it is controlled externally
        // flipsy will decide when to call this
        if (!(myRobot_working is Robot_BB_Flipsy))
        {
            myRobot_working.updateGamepadVars();
        }

        if ( myPosition == "Spectator" ) { return; }

        // Update the game variables

        //Send our inputs
        string m = GLOBALS.HEADER_IN_INPUTS + GLOBALS.SEPARATOR1

            // Basic buttons
            + MyUtils.BoolToString(myRobot_working.gamepad1_a) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_b) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_x) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_y) + GLOBALS.SEPARATOR2

            // Basic Movement
            + myRobot_working.gamepad1_right_stick_y.ToString("0.###") + GLOBALS.SEPARATOR2
            + myRobot_working.gamepad1_right_stick_x.ToString("0.###") + GLOBALS.SEPARATOR2
            + myRobot_working.gamepad1_left_stick_x.ToString("0.###") + GLOBALS.SEPARATOR2
            + myRobot_working.gamepad1_left_stick_y.ToString("0.###") + GLOBALS.SEPARATOR2

            // DPAD
            + MyUtils.BoolToString(myRobot_working.gamepad1_dpad_down) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_dpad_up) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_dpad_left) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_dpad_right) + GLOBALS.SEPARATOR2

            // Others
            + MyUtils.BoolToString(myRobot_working.gamepad1_right_bumper) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_left_bumper) + GLOBALS.SEPARATOR2
            + myRobot_working.gamepad1_left_trigger.ToString("0.###") + GLOBALS.SEPARATOR2
            + myRobot_working.gamepad1_right_trigger.ToString("0.###") + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_stop) + GLOBALS.SEPARATOR2
            + MyUtils.BoolToString(myRobot_working.gamepad1_restart) + GLOBALS.SEPARATOR2
            ;
        sendUdpData(m);

        // Process non-standard requests
        
        // Check for character reset request: RESETMYPOS
        if (myRobot_working.gamepad1_reset_changed && myRobot_working.gamepad1_reset)
        {
            RequestPosReset();
        }
    }

    private long lastFlagTime = 0;

    private void SendFlags()
    {
        long currtime = MyUtils.GetTimeMillis();
        // Update data rate every 250ms
        if( currtime - lastFlagTime < GLOBALS.CLIENT_FLAGS_UPDATE_DELAY)
        {
            return;
        }

        lastFlagTime = currtime;


        // If no flags, send blank
        if (clientFlags.Count < 1)
        {
            clientFlags["NotUsed"] = "1";
        }

        // Don't send flags if ID not established 
        if( ourClientId < 0) { return;  }

        // Don't send flags if in playback mode
        // But still need to ping server so no disconnection
        if( GLOBALS.now_playing ) {

            SendFlagsUDPTracked("NotUsed" +GLOBALS.SEPARATOR2 + "1");
            return; 
        }

        string m = "";

        // Assumed there are less than 4k bytes of data to be sent here, otherwise issues may arise
        bool firstline = true;
        foreach (String currkey in clientFlags.Keys)
        {
            if( !firstline )
            {
                m += GLOBALS.SEPARATOR2;
            }
            firstline = false;
            m += currkey + GLOBALS.SEPARATOR2 + clientFlags[currkey];
        }

        SendFlagsUDPTracked(m);

        // Clear flags with no handshaking
        clientFlags.Remove("CHAT");
        clientFlags.Remove("RESTARTALL");
        clientFlags.Remove("RESETMYPOS");
        clientFlags.Remove("GameSettings");

        FlagReset();
        
    }


    // *************************************************************************
    // ************* INCOMING MESSAGE PROCESSING         ***********************
    // *************************************************************************
    // *************************************************************************

    // Buffer of all incoming packets
    private List<byte[]> allReceivedData = new List<byte[]>();

    // List of all variables/flags server has sent over
    public Dictionary<string, string> serverFlags = new Dictionary<string, string>();
    // public string[] serverFlagsCopy;

    // Semaphore to control access to buffer
    private Semaphore allReceivedDataSemaphore = new Semaphore(1, 1);

    public class Player
    {
        public string playerName;
        public string skins;
        public string robot_skins;
        public string model;
        public string position;
        public GameObject avatar;
        public RobotInterface3D robot;
        public int connectionId;
        public bool isRed;
        public bool isSpectator;
        public int messageId; // stores the last message id
        
    }

    /// <summary>
    /// All the players that are currently in the game
    /// </summary>
    public Dictionary<int, Player> players = new Dictionary<int, Player>();
    public bool players_changed = false; // For external application manager to know this changed in this update

    public int latest_message_id = 0;
    public int last_checked_message_id=0;
    public int received_msg_count = 0;
    public long last_update_time = 0;

    public void UpdatePacketLoss()
    {
        // Make sure enough time expired
        long delta_time = GetTimeMillis() - last_update_time;

        if( delta_time < GLOBALS.PACKET_LOSS_UPDATE_TIME)
        {
            return;
        }

        last_update_time = GetTimeMillis();

        // Make sure some packets were received
        if ((latest_message_id-last_checked_message_id) <= 0 )
        {
            clientFlags["PL"] = "0";
            return;
        }

        // Calculate packet loss
        int packetloss = (int) (100f*(1f - (float)(received_msg_count) / ((float)(latest_message_id - last_checked_message_id))));
        clientFlags["PL"] = packetloss.ToString();

        last_checked_message_id = latest_message_id;
        received_msg_count = 0;
    }

    public void onReceivedData(ref List<byte[]> messages)
    {
        // Sanity check
        if( messages.Count < 1) { return;  }

        // Will only process data that are equal to or greater than the latest message id received in each category

        // Assume 5 packets for elements, 1 for flags, 2 per driver for player updates = 8 = ~14 packets per update... assume some out of order, thus
        // limit looking back to 35 messages
        // If combined message setting is on, this will be reduced
        for(int thisMessage = messages.Count - 35; thisMessage < messages.Count; thisMessage++)//go forwards through the messages
        {
            if( thisMessage < 0 )
            { thisMessage = 0; }

            byte[] message = messages[thisMessage];
            List<byte[]> extracted_data = new List<byte[]>();

            // Split the message up into the top components
            if( ! MyUtils.ExtractMessageHeader(message, extracted_data))
            {
                MyUtils.LogMessageToFile("OnReceiveData split failed on _. " + message, true);
                continue;
            }


            // Initial message split into following:
            // <PASSCODE>|<COMPRESSION#>|<CLIENT ID/Time Stamp or -1 for server>|<DATA LENGTH>|<DATA>

            // ************ VERIFY PASSCODE **********************
        
            if (! Encoding.UTF8.GetString(extracted_data[0]).Equals(GLOBALS.PASSCODE))
            {
                MyUtils.LogMessageToFile("OnReceiveData passcode failed. " + message, true);
                continue;
            }//needs to match passcode

            // ************ VERIFY LENGTH **********************
            int length = 0;
            if(! int.TryParse( Encoding.UTF8.GetString(extracted_data[3]), out length) )
            {
                MyUtils.LogMessageToFile("OnReceiveData extraction of length failed. " + message, true);
                continue;
            }

            if( length != extracted_data[4].Length)
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

            // Get timestamp
            int timestamp = -1;
            if( !int.TryParse(Encoding.UTF8.GetString(extracted_data[2]), out timestamp) )
            {
                timestamp = -1;
            }


            // ************ Process Message **********************
            string messageOnly = MyUtils.DecompressMessage(extracted_data[4], compression);

            // MyUtils.LogMessageToFile("MESSAGE: " + messageOnly, true);

            // Check what we are sending
            string[] splitData = messageOnly.Split(GLOBALS.SEPARATOR1);
            if(splitData.Length < 2)
            {
                MyUtils.LogMessageToFile("OnReceiveData split failed on " + GLOBALS.SEPARATOR1 + " " + messageOnly, true);
                continue;
            }//don't go on if the split is bad

            // Will do packet-aging inside each function, not here
            // MyUtils.LogMessageToFile("OnReceiveData type = " + splitData[0], true);
            MyUtils.PB_RecordData(splitData, timestamp);

            // If not in playback mode, then process data
            if (!GLOBALS.now_playing)
            {
                ProcessData(splitData, timestamp);
            }
        }
    }

    // Processes the incoming data
    private void ProcessData(string[] splitData, int timestamp, bool playback_mode = false)
    {
        switch (splitData[0])
        {        
            case GLOBALS.HEADER_NEWPLAYER: // Don't do anything with this
                break;

            case GLOBALS.HEADER_PLAYER_POS:
                found_position = true;
                OnPlayerLocations(splitData, timestamp, playback_mode);
                break;

            case GLOBALS.HEADER_FIELDELEMENTS:
                updateElementLocations(splitData, timestamp, playback_mode);
                break;

            case GLOBALS.HEADER_PLAYERS:
                onCurrentPlayers(splitData, playback_mode);
                break;
            case GLOBALS.HEADER_ERROR:
                onServerError(splitData, playback_mode);
                break;
            case GLOBALS.HEADER_FLAGS:
                OnServerFlags(splitData, playback_mode);
                break;
            default:
                MyUtils.LogMessageToFile("OnReceiveData invalid HEADER:" + splitData[0], true);
                break;
        }
    }



    private int id_current_players = -1;  // Aging of packets for player updates

    private void onCurrentPlayers(string[] data, bool playback_mode = false)
    {
        // Sanity check on data size
        if (data.Length < 4)//only spawn the others if there are others to spawn XD
        {
            MyUtils.LogMessageToFile("onCurrentPlayers data length < 4 at " + data.Length, true);
            return;
        }
        // 1st and second positions were Type of message and message count/id..
        int curr_message_id = -1;
        if (!Int32.TryParse(data[1], out curr_message_id))
        {
            MyUtils.LogMessageToFile("onCurrentPlayers invalid packet age:" + data[1], true);
            return;
        }

        int curr_message_id2 = -1;
        if (!Int32.TryParse(data[2], out curr_message_id2))
        {
            MyUtils.LogMessageToFile("onCurrentPlayers invalid packet age:" + data[2], true);
            return;
        }

        // Drop invalid id
        if(curr_message_id != curr_message_id2) { return; }

        // Drop old messages
        if (!playback_mode)
        {
            if (id_current_players > curr_message_id)
            {
                MyUtils.LogMessageToFile("onCurrentPlayers dropped old message.", true);
                return;
            }

            id_current_players = curr_message_id;
        }

        // Make a copy of all the players
        List<Player> player_copy = new List<Player>( players.Values);

        // I have a problem that I'm not sure why it occurs: if we are spectator, then all robots that are spwaned before this client have 
        // incorrect Body orientation relative to related siblings. Quick fix: make a list of people that need to be spawned and spawn them only after we are spawned
        List<string[]> users_to_spawn = new List<string[]>();

        // Go through all the player list
        // Every player in our playerlist should be present in the current message
        // This requires all players to be present in a single message
        for (int i = 3; i < data.Length; i++)
        {
            //check if we have already spawned the player
            bool shouldSpawn = true;
            
            // Extract the player information, but make sure it meets bare minimum
            string[] currdata = data[i].Split(GLOBALS.SEPARATOR2);
            if( currdata.Length < 4) { continue; }

            // But only if there are players left in our list
            if (player_copy.Count > 0)
            {
                for (int currindex = player_copy.Count - 1; currindex >= 0; currindex--)
                {
                    // Extract cnnID, if can't then skip the player
                    int cnnID = -1;
                    if( ! int.TryParse(currdata[1], out cnnID) )
                    {
                        continue;
                    }
                   

                    if (player_copy[currindex].connectionId == cnnID )
                    {
                        // In case the same cnnID is used for another player, make sure the name also matches
                        if( GLOBALS.client_names.ContainsKey(cnnID) && GLOBALS.client_names[cnnID] != currdata[0])
                        {
                            PlayerDisconnected(cnnID);
                            continue;
                        }

                        shouldSpawn = false;

                        // Update robot states
                        if (player_copy[currindex].robot)
                        {
                            player_copy[currindex].robot.SetStates(currdata[4]);
                        }
                     
                        // Remove player
                        player_copy.RemoveAt(currindex);
                        break;
                    }
                }
            }

            // Spawn ourselves if we came up
            if (shouldSpawn)
            {
                // If this is us, spawn us
                if (ourPlayerName.Equals(currdata[0]))
                {
                    SpawnPlayer(currdata[0], currdata[1], currdata[2], currdata[3], (currdata.Length>5) ? currdata[5] : "0", (currdata.Length>6) ? currdata[6] : "");
                }
                else
                {
                    users_to_spawn.Add(new string[] { currdata[0], currdata[1], currdata[2], currdata[3], (currdata.Length > 5) ? currdata[5] : "0", (currdata.Length > 6) ? currdata[6] : "" });
                }
            }
        }

        // Now spawn any players that still need spawning
        for( int i = 0; i < users_to_spawn.Count; i++)
        {
            SpawnPlayer(users_to_spawn[i][0], users_to_spawn[i][1], users_to_spawn[i][2], users_to_spawn[i][3], users_to_spawn[i][4], users_to_spawn[i][5]);
        }

        // Remove extra players
        for (int currindex = player_copy.Count - 1; currindex >= 0; currindex--)
        {
            // But if we are in playback mode and this is our id, then dont
            if( playbackmode && player_copy[currindex].connectionId == ourClientId )
            { continue; }

            PlayerDisconnected( player_copy[currindex].connectionId);
        }
    }


    private void updateElementLocations(string[] locations, int timestamp = -1, bool playback_mode = false)
    {
        // Turn on interpolation if haven't done so already
        if (!interpolation_on)
        {
            TurnOnInterpolation();
        }

        // 1st and second positions were Type of message and message count/id..
        int curr_message_id = -1;
        if (!Int32.TryParse(locations[1], out curr_message_id))
        {
            MyUtils.LogMessageToFile("UpdateElementLocations invalid packet age:" + locations[1], true);
            return;
        }

        int curr_message_id2 = -1;
        if (!Int32.TryParse(locations[2], out curr_message_id2))
        {
            MyUtils.LogMessageToFile("UpdateElementLocations invalid packet age:" + locations[2], true);
            return;
        }

        // Drop invalid data
        if( curr_message_id != curr_message_id2) { return; } 

        if (!playback_mode && (curr_message_id > latest_message_id))
        {
            latest_message_id = curr_message_id;
        }

        // Assume packet is good, decrement later if bad
        if (!playback_mode) { received_msg_count++; }

        bool decrement_msg_count = false; // if Any of the elements have already been updated, count this is a bad out-of-order packet

        // Element locations can be split into multiple packets: do we update the elements only once all have been received,
        // or update them as the data is coming? If we update them as the data is received, what about receiving older and
        // newer packets out of order for different elements?
        // 

        // Iterate through all the elements
        for (int i = 3; i < locations.Length ; i++)
        {
            // Extract the information
            // The data should be as follows:   element ID, element type, transform xyz, rotation xyzw
            string[] currData = locations[i].Split(GLOBALS.SEPARATOR2);

            // Make sure empty data lines aren't causing errors
            if( currData.Length < 1) { continue; }
            if( currData[0].Length < 1) { continue;  }

            int index = 0;
            int elementID = 0;
            string elementType;
            float trans_x, trans_y, trans_z, rot_x, rot_y, rot_z;

            try
            {
                elementID = int.Parse(currData[index++]);

                // Check if this object was already updated
                if (!playback_mode)
                {
                    if (allFieldElements[elementID].GetComponent<gameElement>().lastupdateID > curr_message_id)
                    {
                        decrement_msg_count = true;
                        continue;
                    }

                    allFieldElements[elementID].GetComponent<gameElement>().lastupdateID = curr_message_id;
                }

                elementType = currData[index++];

                if (allFieldElements[elementID].GetComponent<BandwidthHelper>())
                {
                    index += allFieldElements[elementID].GetComponent<BandwidthHelper>().Set(currData, index, timestamp);
                }
                else
                {
                    trans_x = float.Parse(currData[index++]);
                    trans_y = float.Parse(currData[index++]);
                    trans_z = float.Parse(currData[index++]);
                    rot_x = float.Parse(currData[index++]);
                    rot_y = float.Parse(currData[index++]);
                    rot_z = float.Parse(currData[index++]);
                    allFieldElements[elementID].GetComponent<interpolation>().SetPosition(new Vector3(trans_x, trans_y, trans_z), timestamp);
                    allFieldElements[elementID].GetComponent<interpolation>().SetRotation(new Vector3(rot_x, rot_y, rot_z), timestamp);
                }
               
            }
            catch (Exception e)
            {
                //this could happen if the parsing doesn't work
                MyUtils.LogMessageToFile("updateElementLocations exception! " + e.ToString(), true);
                continue;
            }
    
        }

        // If any of the elements were already updated with a future packet, don't count this packet
        if(!playback_mode && decrement_msg_count) { received_msg_count--; }
    }

    private void OnPlayerLocations(string[] data, int timestamp = -1, bool playback_mode = false)
    {
        // 1st and second positions were Type of message and message count/id..
        
        int curr_message_id = -1;
        if (!Int32.TryParse(data[1], out curr_message_id))
        {
            MyUtils.LogMessageToFile("OnPlayerLocations invalid packet age:" + data[1], true);
            return;
        }

        int curr_message_id2 = -1;
        if (!Int32.TryParse(data[2], out curr_message_id2))
        {
            MyUtils.LogMessageToFile("OnPlayerLocations invalid packet age:" + data[2], true);
            return;
        }

        // Drop bad packets
        if( curr_message_id != curr_message_id2) { return; }
       
        if ( !playback_mode && (latest_message_id < curr_message_id) )
        {
            latest_message_id = curr_message_id;
        }

        // Assume this is a good message... if it isn't, we'll decrement later
        if (!playback_mode) { received_msg_count++; }

        try
        {
            netmonitor_header = GLOBALS.HEADER_PLAYER_POS;

            // The rest is the packet content
            for (int i = 3; i < data.Length; i++)
            {
                // Make sure valid data contained
                if (  !data[i].Contains(GLOBALS.SEPARATOR2.ToString()) )
                {
                    continue;
                }

                string[] d = data[i].Split(GLOBALS.SEPARATOR2);
                int id = Int32.Parse( d[0]);
                int robotnamelength = Int32.Parse(d[1]);
                int istart = 2; // Starting index in array that contains element data

         
                if (players.ContainsKey(id))
                {
                    if (!playback_mode && GLOBALS.ENABLE_UDP_STATS)
                    {
                        String key = netmonitor_header + "_" + id + "_in";
                        if (netmonitor.ContainsKey(key))
                        { netmonitor[key] += 1; }
                        else
                        { netmonitor.Add(key, 1); }
                    }

                    // Compare packet age, if it's old, we should abonden it
                    if (!playback_mode && (players[id].messageId > curr_message_id))
                    {
                        received_msg_count--;
                        return;
                    }

                    players[id].messageId = curr_message_id;

                    int childcount = players[id].avatar.transform.childCount;
                    for (int j = istart; j < d.Length - 1; )
                    {
                        try
                        {
                            int index = int.Parse(d[j]);

                            // Make sure index does not exceed childcount
                            if (index >= childcount)
                            {
                                MyUtils.LogMessageToFile("OnPlayerLocations index > childcount: " + index.ToString() + ">" + childcount.ToString(), true);
                                break;
                            }

                            // Differentiate between parent and child. Parent set global positions/orientation so we don't have to worry if it's part
                            // of some other hierarchy..
                            if (index < 0)
                            {
                                players[id].avatar.transform.GetComponent<interpolation>().SetPosition(
                                                                   new Vector3(float.Parse(d[j + 1]), float.Parse(d[j + 2]), float.Parse(d[j + 3])), timestamp);

                                players[id].avatar.transform.GetComponent<interpolation>().SetRotation(
                                    new Vector3(float.Parse(d[j + 4]), float.Parse(d[j + 5]), float.Parse(d[j + 6])), timestamp);

                                j += 7;

                            }
                            else
                            {
                                // If it has a BandwidthHelper class, then use it
                                if (players[id].avatar.transform.GetChild(index).GetComponent<BandwidthHelper>())
                                {
                                    j = players[id].avatar.transform.GetChild(index).GetComponent<BandwidthHelper>().Set(d,j+1, timestamp);
                                }
                                else
                                {
                                    players[id].avatar.transform.GetChild(index).GetComponent<interpolation>().SetLocalPosition(
                                        new Vector3(float.Parse(d[j + 1]), float.Parse(d[j + 2]), float.Parse(d[j + 3])), timestamp);

                                    players[id].avatar.transform.GetChild(index).GetComponent<interpolation>().SetLocalRotation(
                                        new Vector3(float.Parse(d[j + 4]), float.Parse(d[j + 5]), float.Parse(d[j + 6])), timestamp);

                                    j += 7;
                                }
                            }
                        }
                        catch (Exception e)
                        {
                            MyUtils.LogMessageToFile("Error in OnPlayerLocations: " + e, true);
                            return;
                        }
                    }

                    if (!playback_mode && GLOBALS.ENABLE_UDP_STATS)
                    {
                        String key = netmonitor_header + "_" + id + "_out";
                        if (netmonitor.ContainsKey(key))
                        { netmonitor[key] += 1; }
                        else
                        { netmonitor.Add(key, 1); }
                    }
                }
                else
                {
                    // Common to get a playerlocations update before the player was updated.. thus ignore this for now
                    // MyUtils.LogMessageToFile("OnPlayerLocations: ID " + id + " not present. Myid = " + ourClientId, true);
                }
            }
        }
        catch (Exception e)
        {
            MyUtils.LogMessageToFile("Client OnPlayerLocations exception: " + e, true);
        }
    }

    private int id_server_error = -1;

    private void onServerError(string[] data, bool playback_mode = false)
    {
        // 1st and second positions were Type of message and message count/id..
        int curr_message_id = -1;
        if (!Int32.TryParse(data[1], out curr_message_id))
        {
            MyUtils.LogMessageToFile("onServerError invalid packet age:" + data[1], true);
            return;
        }

        if (!playback_mode)
        {
            if (id_server_error > curr_message_id) { return; }
            id_server_error = curr_message_id;
        }

        // Don't care about showing older packets....
        if (data.Length >= 4)
        { 
            ShowMessage(data[3]);
        }
    }


    // Process client flags
    private int client_confirmed_this_msg_id = 0;
    private void OnServerFlags(string[] splitData, bool playback_mode = false)
    {
        serverFlags.Clear();

        // splitData[0] defines this is a server-flag
        // splitData[1] should contain all the flag messages, each one seperated by SEPERATOR3
        // Each message has a key=value seperated by SEPERATOR2
        // The oldest message is first, the newest last
        string[] all_flag_messages = splitData[1].Split(GLOBALS.SEPARATOR3);
        int message_id;
        int message_id2;

        for (int i = 0;  i < all_flag_messages.Length; i++)
        {
            // Split the message to the key, value pair
            string[] id_key_value = all_flag_messages[i].Split(GLOBALS.SEPARATOR2);
            // serverFlagsCopy = id_key_value; 
            if (id_key_value.Length < 4) { continue; } // Make sure there are at least 4 items ( 0 = message id, 1= message id, 2 = first key, 3 = first value
            message_id = int.Parse(id_key_value[0]);
            message_id2 = int.Parse(id_key_value[1]);
            if( message_id != message_id2) { continue; } // Checked id wasn't corrupted

            if (!playback_mode)
            {
                if (message_id <= client_confirmed_this_msg_id) { 
                    // If the difference is too large, we may have  corruption. We then want to re-initizalize it
                    if(message_id + (2*GLOBALS.MAX_TRACKED_PACKETS) < client_confirmed_this_msg_id)
                    {
                        client_confirmed_this_msg_id = message_id;
                    }
                    
                    continue;                                                       
                } // Don't process server message sthat we already have
                else { client_confirmed_this_msg_id = message_id; }           // Messages are sent in chronological order (oldest first)
            }

            // Go through all the keys and add them
            for (int j = 2; j < id_key_value.Length;)
            {
                string key = id_key_value[j++];
                string value = id_key_value[j++];
                serverFlags[key] = value;    
            }

            // Record the last message server received
            if (!playback_mode && serverFlags.ContainsKey("TRACKED_FLAG_ID"))
            {
                server_confirmed_this_msg_id = int.Parse(serverFlags["TRACKED_FLAG_ID"]);
            }

            // Do we need to process Flags after every received message???? Yes
            ProcessServerFlags();
        }

    }

    // Is set to true if this user name is designated as admin or startposition is admin
    public bool IsAdmin = false;

    
    // ********** sound stuff **************


    private void soundStart(string message)
    {
        // Extract the sound info
        // there may be multiple entries
        string[] all_sounds = message.Split(GLOBALS.SEPARATOR4);

        for( int i = 0; i<all_sounds.Length; i++)
        {
            // Extract current sounds
            string[] sound_info = all_sounds[i].Split(GLOBALS.SEPARATOR5);
            if( sound_info.Length < 4) { continue; }


            int id = int.Parse(sound_info[0]);
            string name = sound_info[1];
            float crossfade = float.Parse(sound_info[2]);
            float volume = float.Parse(sound_info[3]);

            // Now play the sound
            // If this is a positive number, then the sound is a field element
            if (id > 0)
            {
                allFieldElements[id].GetComponent<AudioManager>().Play(name, crossfade, volume, true);
                continue;
            }

            id *= -1;

            // Otherwsie this is a robot cnnID
            if( ! players.ContainsKey(id)) { continue; }

            GameObject avatar = players[id].avatar;
            if (!avatar) { continue; }

            AudioManager audiomanager = avatar.GetComponentInChildren<AudioManager>();
            if (!audiomanager) { continue; }

            audiomanager.Play(name, crossfade, volume, true);
        }
    }

    private void soundStop(string message)
    {
        // Extract the sound info
        // there may be multiple entries
        string[] all_sounds = message.Split(GLOBALS.SEPARATOR4);

        for (int i = 0; i < all_sounds.Length; i++)
        {
            // Extract current sounds
            string[] sound_info = all_sounds[i].Split(GLOBALS.SEPARATOR5);
            if (sound_info.Length < 3) { continue; }


            int id = int.Parse(sound_info[0]);
            string name = sound_info[1];
            float crossfade = float.Parse(sound_info[2]);

            // Now play the sound
            // If this is a positive number, then the sound is a field element
            if (id > 0)
            {
                allFieldElements[id].GetComponent<AudioManager>().Stop(name, crossfade, true);
                continue;
            }

            id *= -1;

            // Otherwsie this is a robot cnnID
            if (!players.ContainsKey(id)) { continue; }

            // Stop the sound, but the player may have been disabled by its animation, thus be gentle here
            GameObject avatar = players[id].avatar;
            if ( !avatar) { continue; }

            AudioManager audiomanager = avatar.GetComponentInChildren<AudioManager>();
            if( !audiomanager) { continue; }

            audiomanager.Stop(name, crossfade, true);
        }
    }


    //*************************************
    private GameObject myPlayer = null;
    private RobotInterface3D myRobot_working = null;
    private RobotInterface3D myRobot_saved = null;
    public string myPosition = "";

    private void SpawnPlayer(string playerName, string cnnIDraw, string robotmodel, string RobotPosition, string skins = "0", string robot_skin = "")
    {
        // Get cnnId
        int cnnId = -1;
        if (!Int32.TryParse(cnnIDraw, out cnnId))
        {
            MyUtils.LogMessageToFile("SpawnPlayer had invalid cnnIDraw " + cnnIDraw, true);
        }
        
        // If player position is Spectator, then only spawn it if its us
        if (( RobotPosition == "Spectator") && !ourPlayerName.Equals(playerName))
        { return; }

        // Load the player robot
        GameObject go = null;

        // Instantiate it in the same position as server in case relative algorithms require it
        GameObject starting_position = GameObject.Find(RobotPosition);
        Vector3 pos = Vector3.zero;
        Quaternion rot = Quaternion.identity;

        if (starting_position)
        {
            pos = starting_position.transform.position;
            rot = starting_position.transform.rotation;
        }

        try
        {
            go = MyUtils.InstantiateRobot(robotmodel, pos, rot, skins, robot_skin);
        }
        catch (Exception e)
        {
            ShowMessage("Missing robot model for player " + playerName + ". Can't show player!");
            return;
        }

        // Enable any existing interpolations
        interpolation[] allinter = go.GetComponentsInChildren<interpolation>();
        foreach (interpolation currobj in allinter)
        {
            if (currobj != null) { currobj.enabled = true; }
        }


        Player p = new Player
        {
            avatar = go,
            robot = go.GetComponent<RobotInterface3D>(),
            playerName = playerName,
            model = robotmodel,
            position = RobotPosition,
            skins = skins,
            robot_skins = robot_skin,
            connectionId = cnnId,
            messageId = 0,
            isRed = (RobotPosition.StartsWith("Red")) ? true : false,
            isSpectator = (RobotPosition == "Spectator") ? true : false
        };

        // Set player name and other robot stuff
        if (p.robot)
        {
            p.robot.SetName(playerName);
            p.robot.SetKinematic(true);

            // turn off real-robot stuff in robot 
            if (RobotPosition == "Spectator")
            {
                p.robot.isSpectator = true;             
                p.robot.Initialize();
                spec_myRobot = (Robot_Spectator) p.robot;
            }
            else
            // Stuff we should only be doing if we have a real robot
            {
                // Turn on interpolation in the robot 
                TurnOnInterpolationInObject(go);

                RobotID newid = go.AddComponent<RobotID>() as RobotID;
                newid.id = cnnId;
                newid.starting_pos = RobotPosition;

                p.robot.Initialize();

                // Set its color
                p.robot.SetColorFromPosition(RobotPosition);

                GLOBALS.client_names[cnnId] = playerName;
                GLOBALS.client_ids[playerName] = cnnId;
            }
        }

        players.Add(cnnId, p);
        players_changed = true;

        //is this ours?
        if (ourPlayerName.Equals(playerName))
        {
            ourClientId = cnnId; // Record our ID to identify our sent messages
            myPlayer = go;
            myPosition = RobotPosition;
            myRobot_working = p.robot; // This is the working copy of our robot - can change when we cycle through views
            myRobot_saved = p.robot; // This is the saved copy so we can always restore this view
            SetCamera(RobotPosition);

            if (p.robot)
            {
                GLOBALS.I_AM_SPECTATOR = p.robot.isSpectator;
                if (p.robot.myRobotID)
                {
                    GLOBALS.I_AM_RED = p.robot.myRobotID.is_red;
                }
            }
        }

        // Get it's BodyColliders and add a rigid body (to allow highliting/selecting of bots) 
        BoxCollider[] allboxes = go.GetComponentsInChildren<BoxCollider>();
        foreach (BoxCollider currbox in allboxes)
        {
            if (currbox.gameObject.layer == GLOBALS.LAYER_RobotBoundry)
            {
                // give it a rigid body
                Rigidbody newbody = currbox.gameObject.AddComponent<Rigidbody>();
                newbody.isKinematic = true;
                newbody.collisionDetectionMode = CollisionDetectionMode.Discrete;
                newbody.detectCollisions = true;
                newbody.WakeUp();
            }
        }

        // Update scorer so it can track robot penalties
        scorer.FieldChanged();
    }

    // Spectator details
    private GameObject spec_myPlayer = null;
    private Robot_Spectator spec_myRobot = null;
    private bool use_spec_myRobot = false;

    private void SpawnSpectator()
    {
        if (spec_myRobot) { return; }

        // Load the player robot
        GameObject go;

        // Instantiate it in the same position as server in case relative algorithms require it
        GameObject starting_position = GameObject.Find("Spectator");
        Vector3 pos = Vector3.zero;
        Quaternion rot = Quaternion.identity;

        if (starting_position)
        {
            pos = starting_position.transform.position;
            rot = starting_position.transform.rotation;
        }

        try
        {
            go = MyUtils.InstantiateRobot("AvatarSpectator", pos, rot);
        }
        catch (Exception e)
        {
            ShowMessage("Missing spectaqtor robot model!");
            return;
        }

        // Enable any existing interpolations
        interpolation[] allinter = go.GetComponentsInChildren<interpolation>();
        foreach (interpolation currobj in allinter)
        {
            if (currobj != null) { currobj.enabled = true; }
        }


        // turn off real-robot stuff in robot 
        Robot_Spectator ri3d = go.GetComponent<Robot_Spectator>();
        ri3d.isSpectator = true;
        ri3d.SetKinematic(true);
        ri3d.Initialize();

        // Save results for later access
        spec_myPlayer = go;
        spec_myRobot = ri3d;
    }

    public bool isSpectator()
    {
        if( myPosition == "Spectator") { return true; }
        return false;
    }


    private void PlayerDisconnected(int cnnId)
    {
        if (players[cnnId].robot)
        {
            players[cnnId].robot.deleted = true;
        }

        Destroy(players[cnnId].avatar);
        players[cnnId].robot = null;
        players.Remove(cnnId);

        if (GLOBALS.client_names.ContainsKey(cnnId))
        {
            string playername = GLOBALS.client_names[cnnId];
            GLOBALS.client_names.Remove(cnnId);

            if(GLOBALS.client_ids.ContainsKey(playername))
            {
                GLOBALS.client_ids.Remove(playername);
            }
        }

        players_changed = true;
    }

    private GameSettings ourgamesettings;
    private IEnumerator autosave_enumerator;

    private IEnumerator FinishAutosave()
    {
        while( MyUtils.PB_AutoSaveInProgress())
        {
            MyUtils.PB_AutoSaveToFile();
            yield return null;
        }

        autosave_enumerator = null;
        yield break;
    }

    private float ignore_TM_ANIM2_time = 0f;

    private void ProcessServerFlags()
    {
        // First see if there is a response to one of our flags
        if (!playbackmode && (ourClientId == -1)) { return; }

        // Remove any big server messages
        big_message.SetActive(false);

        holding_mode = false;
        IsAdmin = (myStartfield == "Admin");

        // Go through all the server flags
        foreach (String currkey in serverFlags.Keys)
        {
            switch (currkey)
            {
                case "RESTARTALL":
                    clientFlags.Clear();
                    FlagReset();
                    ShowMessage("Field reset by " + serverFlags[currkey]);
                    break;

                case "TM_STATE":
                    ProcessTournamentMode();
                    break;

                case "CHAT":
                    // No hand-shaking for chat, thus just display message
                    ShowMessage(serverFlags[currkey]);
                    break;

                case "SOUND_PLAY":
                    soundStart(serverFlags[currkey]);
                    break;

                case "SOUND_STOP":
                    soundStop(serverFlags[currkey]);                   
                    break;

                case "COUNTDOWN":
                    scorer.StartCountdown();
                    break;

                case "TM_ANIM1": // This is the active form, if we see it then ignore TM_ANIM2 for 1s
                    
                    if (serverFlags[currkey][0] == '1')
                    {
                        scorer.ShowGameStartOverlay(true);
                        ignore_TM_ANIM2_time = Time.time;                        
                    }
                    else
                    {
                        scorer.ShowGameStartOverlay(false);
                    }
                                       
                    break;

                case "TM_ANIM2": // this is passive form (can be overriden), and only really used to turn off overlay;
                    if (Time.time - ignore_TM_ANIM2_time < 2f) // Ignore if we marked this for ignore
                    {
                        break;
                    }
                    
                    if (serverFlags[currkey][0] == '1')
                    {
                        scorer.ShowGameStartOverlay(true, true); // should never occur
                    }
                    else
                    {
                        scorer.ShowGameStartOverlay(false, true);
                    }
                    break;

                case "FIREWORKS":
                    scorer.StartFireworks(serverFlags["FIREWORKS"]=="RED");
                    // Autosave if applicable
                    MyUtils.PB_AutoSaveToFile();
                    autosave_enumerator = FinishAutosave();
                    StartCoroutine(autosave_enumerator);
                    break;

                case "HOLDING":
                    if (myPosition != "Spectator")
                    {
                        big_message.GetComponent<Text>().text = "WAITING FOR TOURNAMENT TO START...";
                        big_message.SetActive(true);
                    }
                    holding_mode = true;
                    break;
                case "ADMIN":
                    if( int.Parse(serverFlags["ADMIN"]) == ourClientId )
                    {
                        IsAdmin = true;
                    }
                    else
                    {
                        IsAdmin = (myStartfield == "Admin");
                    }

                    break;
                case "GameSettings":
                    // Find game settings
                    if( !ourgamesettings)
                    {
                        ourgamesettings = FindObjectOfType<GameSettings>();
                    }

                    if (ourgamesettings)
                    {
                        ourgamesettings.SetString(serverFlags[currkey]);
                    }
                    
                    // If this is the first time game-settings have been synced, then give scorer a change to reset
                    if( first_time_game_settings)
                    {
                        scorer.ScorerReset();
                        first_time_game_settings = false;
                    }
                    break;

                case "SCORER":
                    // Call the scorer interrupt response
                    scorer.OnScorerInterrupt(serverFlags[currkey]);
                    break;

                default:  // Default case: do nothing, probably belongs to some other code/function
                    break;
            }
        }

        // Process Champs 
        ToggleChampsMode();

        // Remove any processed keys
        serverFlags.Remove("RESTARTALL");
        serverFlags.Remove("CHAT");
        serverFlags.Remove("SOUND_PLAY");
        serverFlags.Remove("SOUND_STOP");
        serverFlags.Remove("COUNTDOWN");
        serverFlags.Remove("FIREWORKS");
        serverFlags.Remove("GameSettings");
        serverFlags.Remove("SCORER");
    }


    public GameObject Tournament_top;
    public GameObject Tournament_msg;
    public GameObject Tournament_ready;

    private void ProcessTournamentMode()
    {
        string tm_state = "NO";
        string tm_msg = "";

        if( ! serverFlags.TryGetValue("TM_STATE", out tm_state) )
        {
            Tournament_top.SetActive(false);
            return;
        }

        serverFlags.TryGetValue("TM_MSG", out tm_msg);

        // Set the message
        Text textobj = Tournament_msg.GetComponent<Text>();

        if (textobj == null)
        { return; }

        textobj.text = tm_msg;
        textobj.enabled = true;

        switch( tm_state)
        {
            case "NO":
                Tournament_top.SetActive(false);
                // Nothing to do
                return;

            case "WAITING":
                // Make sure ready button is enabled
                Tournament_ready.SetActive(true);

                if(Tournament_ready.GetComponent<Toggle>().isOn)
                {
                    clientFlags["READY"] = "READY";
                }
                else
                {
                    clientFlags["READY"] = "";
                }

                break;

            case "COUNTDOWN":
            case "RUNNING":
            case "END":
            default:
                // Reset ready flag
                clientFlags["READY"] = "";
                Tournament_ready.GetComponent<Toggle>().isOn = false;
                Tournament_ready.SetActive(false);
                break;
        }

        // Make sure Tournament mode is active
        Tournament_top.SetActive(true);
    }

    GameObject ChampsObject;
    bool no_champs_exists = false;

    public void DisableChampsMode()
    {
        if( ChampsObject)
        {
            ChampsObject.SetActive(false);
        }

        no_champs_exists = true;
    }

    public void ToggleChampsMode()
    {
        // If there is not champs object, exit
        if(no_champs_exists) { return; }

        // See if there is a champs section and if there is, enable it
        if(serverFlags.ContainsKey("CHAMPS") )
        {
            // If ChampsObject does not exist, find it
            if( !ChampsObject )
            {
                ChampsInit[] champsobjects = Resources.FindObjectsOfTypeAll<ChampsInit>();
                if (champsobjects.Length < 1) {
                    no_champs_exists = true; // Don't check again
                    return; 
                }

                ChampsObject = champsobjects[0].gameObject;
            }

            // If not enabled, enable it
            if( !ChampsObject.activeSelf)
            {
                ChampsObject.SetActive(true);

                // Set text
                TMPro.TextMeshPro[] usertexts = ChampsObject.GetComponentsInChildren<TMPro.TextMeshPro>();

                foreach (TMPro.TextMeshPro currtext in usertexts)
                {
                    if (currtext.name.StartsWith("UserText"))
                    {
                        currtext.text = serverFlags["CHAMPS"];
                    }
                }
            }

        }
        else
        {
            // Otherwise disable it
            if( ChampsObject && ChampsObject.activeSelf )
            {
                ChampsObject.SetActive(false);
            }
        }
    }

    // **************************************************************
    // **************************************************************
    // ****************RAW UDP related section **********************
    // **************************************************************
    // **************************************************************

    // Semaphore for permitting only 1 instance to use udpClient
    private Semaphore udpSemaphore = new Semaphore(1, 1);

    // Count of how many errors we printed: tries to limit it to prevent spamming logfile
    private int datareceivederrors = 0;

    // The single udpClient instance used for all communication
    private UdpClient m_udpClient;

    // The server's endpoint information for sending data
    private IPEndPoint receiverEP;

    // Our unique ID as reported from server. Need it for tagging messages
    private int ourClientId = -1;


    // ************ UDP Algorithm 0: udpClient asyncrhonous receive
    // used udpClient asynch receive to process incoming packets

    private void DataReceived(IAsyncResult ar)
    {
         // If marked for end, abort
         if( killme || connection_state == ConnectionStates.LOST)
         { return; }
        
         // Finish reading rest of data
        UdpClient currentclient = (UdpClient)ar.AsyncState;
        Byte[] receivedBytes;
        IPEndPoint receiverEP = new IPEndPoint(IPAddress.Any, 0);

        try
        {
            // Since we may be receiving binary data, we will not convert to "ASCII"
            receivedBytes = currentclient.EndReceive(ar, ref receiverEP);

            // should never fault past this point, but don't want to execute it if above fails
            if (allReceivedDataSemaphore.WaitOne())
            {
                allReceivedData.Add(receivedBytes);
                allReceivedDataSemaphore.Release();
            }
        }
        catch (Exception e)
        {
            if (datareceivederrors < 10)
            { MyUtils.LogMessageToFile("DR Exception: " + e.ToString(), false); }
            datareceivederrors++;
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

    private void DataReceive( )
    {
        // Keep receiving new messages until we have a problem
        // We are using 2 sempahores: messages and udp
        // to make sure we don't have an interlock issues, lets split the code so only one 
        // is required at any given time
        while (!killme && connection_state != ConnectionStates.LOST)
        {
            int sleep_duration = 0;
            Byte[] receivedBytes;

            // First deal with the udpClient data (and semaphore)
            try
            {
                // Do check if read is possible
                udpSemaphore.WaitOne();
                if(m_udpClient.Available < 1)
                {
                    // If no data available, then might as well sleep for a little while
                    sleep_duration = 1;
                    continue; 
                }

                receivedBytes = m_udpClient.Receive(ref receiverEP);
            }
            catch (Exception e)
            {
                MyUtils.LogMessageToFile("DR GLOBAL udpClient Exception: " + e.ToString(), false);
                ShowMessage("DR GLOBAL udpClient Exception: " + e.ToString());

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

            // Mark time of the packet so monitor knows connection is alive
            time_last_packet = GetTimeMillis();
            allReceivedDataSemaphore.WaitOne();
            allReceivedData.Add(receivedBytes);
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

        if( m_udpClient == null) { return; }

        // get any data that is present
        while (m_udpClient.Available > 10)
        {
            IPEndPoint incomingEP;
            incomingEP = new IPEndPoint(IPAddress.Any, 0);

            try
            {
                Byte[] rawbytes = m_udpClient.Receive(ref incomingEP);

                // Add to our queue of messages to be processed
                allReceivedData.Add(rawbytes);

                time_last_packet = GetTimeMillis();
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


    // Sends the UDP message
    public void sendUdpData(string messageraw)
    {
        // Use compression if enabled
        byte[] message = MyUtils.CompressMessage(messageraw);

        string header = GLOBALS.PASSCODE + GLOBALS.SEPARATOR1 + 
                        GLOBALS.PACKET_COMPRESSION.ToString() + GLOBALS.SEPARATOR1 + 
                        ourClientId + GLOBALS.SEPARATOR1 + 
                        message.Length.ToString() + GLOBALS.SEPARATOR1;

        byte[] messageBytes = MyUtils.CombineByteArrays(Encoding.UTF8.GetBytes(header), message);

        try
        {
            udpSemaphore.WaitOne();
            m_udpClient.Send(messageBytes, messageBytes.Length); // Returns bytes sent... could check if all was sent but I'm not going to do anything with that info so wont bother
        }
        catch (Exception e)
        {
            MyUtils.LogMessageToFile("exception in sending!! " + e, true);
            
            // Nothing to do but hope next message gets through
        }
        finally
        {
            udpSemaphore.Release();
        }

    }

    // Sends all UDP packets that haven't been confirmed to server
    public int curr_tracked_packet = 1;
    public int server_confirmed_this_msg_id = -1;
    private Dictionary<int, string> client_sent_packets = new Dictionary<int, string>();
    public bool SendFlagsUDPTracked(string messageraw)
    {
        // Add it to our list of 
        // Send packet ID twice so if it get corrupter we won't be screwed.
        curr_tracked_packet += 1;
        client_sent_packets[curr_tracked_packet] = curr_tracked_packet.ToString() + GLOBALS.SEPARATOR2 + curr_tracked_packet.ToString() + GLOBALS.SEPARATOR2 + messageraw;

        // Will use SEPERATOR3 for seperating tracked packets

        // If this is the first time server is sent a message, make it's oldest ID equal to the current one
        if (server_confirmed_this_msg_id < 0) { server_confirmed_this_msg_id = curr_tracked_packet - 1; }

        // Go through the list and send all packets not sent previously
        // The start of message needs to indicate this is a Flag packet
        StringBuilder outmessage = new StringBuilder();
        outmessage.Append(GLOBALS.HEADER_FLAGS + GLOBALS.SEPARATOR1);

        // Next add al lthe individual messages
        bool firstdone = false;
        int i = server_confirmed_this_msg_id + 1;
        if ((curr_tracked_packet - i) > GLOBALS.MAX_TRACKED_PACKETS) { i = curr_tracked_packet - GLOBALS.MAX_TRACKED_PACKETS + 1; }

        for (; i <= curr_tracked_packet; i++)
        {
            // Append seperator if required
            if (firstdone) { outmessage.Append(GLOBALS.SEPARATOR3); }
            else { firstdone = true; }

            // Add packet
            outmessage.Append(client_sent_packets[i] + GLOBALS.SEPARATOR2 + "TRACKED_FLAG_ID" + GLOBALS.SEPARATOR2 + client_confirmed_this_msg_id);
        }

        sendUdpData(outmessage.ToString());
        // DEBUG_print_flags_udp(outmessage.ToString());

        // Clean out old messages
        int oldest_required = server_confirmed_this_msg_id+1; // Increment to represent first packet needed to be stored
        if ((curr_tracked_packet - oldest_required) > GLOBALS.MAX_TRACKED_PACKETS) { oldest_required = curr_tracked_packet - GLOBALS.MAX_TRACKED_PACKETS + 1; }

        // Clean up all messages no longer needed
        for (int j = oldest_required - 1; j > 0; i--)
        {
            if (client_sent_packets.ContainsKey(j)) { client_sent_packets.Remove(j); }
            else { break; }
        }

        return true;
    }


    private void DEBUG_print_flags_udp(string message)
    {
        MyUtils.LogMessageToFile("CS: Server last received: server_confirmed_this_msg_id = " + server_confirmed_this_msg_id, true);
        MyUtils.LogMessageToFile("CS: Client current id: curr_tracked_packet = " + curr_tracked_packet, true);
        MyUtils.LogMessageToFile("SC: Client last received: client_confirmed_this_msg_id = " + client_confirmed_this_msg_id, true);
        MyUtils.LogMessageToFile("Fifo buffer size: server_sent_packets = " + client_sent_packets.Count, true);
        MyUtils.LogMessageToFile("  curr_tracked_packet = " + curr_tracked_packet, true);

        string[] message_without_header = message.Split(GLOBALS.SEPARATOR1);
        string[] message_top = message_without_header[1].Split(GLOBALS.SEPARATOR3);

        for (int i = 0; i < message_top.Length; i++)
        {
            string[] message_inside = message_top[i].Split(GLOBALS.SEPARATOR2);

            bool failed = false;
            int id;
            if (!int.TryParse(message_inside[0], out id))
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


    // Starts UDP: Creates client, starts asyncrhonous sections, etc...
    private bool startUdp()
    {
        m_udpClient = new UdpClient();

        // UDP is a connectionless protocoll, so there is no way of knowing if a server received our message.
        // Will therefore want to add some handshaking to see if we are able to talk to the server...
        try
        {
            receiverEP = new IPEndPoint(IPAddress.Parse(serverIP), serverPORT); // endpoint where server is listening
            m_udpClient.Connect(receiverEP);
        }
        catch (Exception e)
        {
            ShowMessage("ERROR: " + e);
            MyUtils.LogMessageToFile("UDP Start Error: " + e, true);
            return false;
        }

        // Don't allow fragmentation: the name of the property is BACKWARDS to how you set it? 
        // Maybe fragmentation is o.k. (meaning don't drop fragmented packets)... rely on ip layer 3 to re-assemble.
        m_udpClient.DontFragment = false;

        return true;
    }

    // ********************************************************
    //   OTHER UTILITIES
    // *********************************************************


    private void ResetAllStates() // Resets variables before new server connection
    {
        // reset all counters and variables
        latest_message_id = 0;
        last_checked_message_id = 0;
        received_msg_count = 0;
        last_update_time = 0;
        id_current_players = -1;
        id_server_error = -1;
        client_confirmed_this_msg_id = 0;
        found_position = false;
        holding_mode = false;
        connection_state = ConnectionStates.NOTSTARTED;
        time_started = -1;
        ourClientId = -1;
        curr_tracked_packet = 1;
        server_confirmed_this_msg_id = -1;
        myStartfield = "";

        // Reset more complex structure
        clientFlags.Clear();
        allReceivedData.Clear();
        serverFlags.Clear();
        client_sent_packets.Clear();


        // Close any active score files
        MyUtils.CloseScorefiles(); 

        // If there are any robots instantiated, clear them
        if ( players.Count > 0 )
        {
            foreach (int currplayer in players.Keys)
            {
                if(players[currplayer].robot) { players[currplayer].robot.deleted = true; }
                Destroy(players[currplayer].avatar);
            }
            players.Clear();
            players_changed = true;
        }


        // Clear myself
        if( myPlayer != null )
        {
            if (myRobot_working) { myRobot_working.deleted = true; }
            Destroy( myPlayer);
            myPlayer = null;
        }

        myRobot_working = null;

        if(m_udpClient != null)
        {
            m_udpClient.Close();
            m_udpClient.Dispose();
            m_udpClient = null;
        }

        // Reset all gameelements memory of last updates
        foreach( GameObject currelement in allFieldElements.Values)
        {
            if( currelement && currelement.GetComponent<gameElement>())
            {
                currelement.GetComponent<gameElement>().lastupdateID = -1;
            }
        }

        // Clear any other potential variables
        GLOBALS.client_names.Clear();
        GLOBALS.client_ids.Clear();

    }

    // Changes the server's settings if applicable
    public void ChangeGameSettings(string newsettings)
    {
        // Only change it if we're the admin
        if( !IsAdmin ) { return; }

        // Otherwise set the correct key to let 
        clientFlags["GameSettings"] = newsettings;
    }

}




