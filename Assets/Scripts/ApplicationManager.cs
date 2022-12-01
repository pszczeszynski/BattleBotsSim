using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using UnityEngine.XR;
using System;
using System.Collections.Generic;
using SimpleFileBrowser;
public class ApplicationManager : MonoBehaviour
{
    GameObject gui_panel;
    Animator gui_animator;

    public GameObject MainCamera;
    public GameObject VRCamera;
    public Camera VRtracking;
    public GameObject EventSystem;
    public GameObject ServerOptions;
    public GameObject chat_text;

    public Toggle toggle_tmode;
    public Toggle toggle_startwhenready;
    public Toggle toggle_holding;
    public InputField Tminplayers;
    public Dropdown dropdown_gameoption;
    public InputField inputfield_password;
    public InputField inputfield_updaterate;
    public InputField inputfield_maxdata;
    public InputField inputfield_admin;
    public TMPro.TextMeshProUGUI scoreadj_red_placeholder;
    public TMPro.TextMeshProUGUI scoreadj_blue_placeholder;

    public GameObject admin_menu;
    public GameObject admin_popup;

    public GameObject admin_button;
    public GameObject options_button;
    public Dropdown admin_kickplayer;

    public Toggle playback_rec;
    public Toggle playback_pause;
    public Toggle playback_play;
    public Toggle playback_save;
    public Toggle playback_stop;
    public Slider playback_slider;
    public TMPro.TMP_Dropdown playback_speed;
    public Text playback_autosave;

    private bool close_client_menu = false;

    bool initialized = false;

    public int pointer_over_active = 0;

    private void Awake()
    {
        // In windows, there is no stadout to the console, thus re-direct it here
#if UNITY_STANDALONE_WIN
        // Go into console mode if in windows
        if (GLOBALS.HEADLESS_MODE && (GLOBALS.win_console == null))
        {
            System.Console.WriteLine("**** STARTING CONSOLE FOR WINDOWS *****");
            GLOBALS.win_console = new Windows.ConsoleWindow();
            GLOBALS.win_console.Initialize();
            System.Console.WriteLine("**** CONSOLE FOR WINDOWS STARTED *****");
        }
#endif

    }

    private void Start()
    {
        // Init settings panel if it exists
        if (settings_panel)
        {
            Transform t_nameinpuit = settings_panel.transform.Find("NameInput");
            if(t_nameinpuit)
            {
                t_nameinpuit.GetComponent<InputField>().text = GLOBALS.default_player_name;
            }
        }

        // Initialize playback buttons
        PB_StopPressed(true);

        GLOBALS.now_playing = false;
        GLOBALS.now_paused= false;
        GLOBALS.now_recording = false;
        MyUtils.PB_ChangeSpeed(1f);

        // Init server settings if presetn
        if( ServerOptions)
        {
            ServerOptionsChange();
        }
    }

    bool old_admin_enable = false;

    public void Update()
    {
        // If client is connected succesfully, then monitor conenction
        if (close_client_menu)
        {
            if( GLOBALS.topclient==null)
            {
                return;
            }

            // If connection lost, re-pop up menu
            if (!GLOBALS.topclient.playbackmode && GLOBALS.topclient.GetConnectionStatus() == -1)
            {
                close_client_menu = false;

                // Make panel manager turn off window
                PanelManager manager = GameObject.Find("MenuManager").GetComponent<PanelManager>();
                if (manager == null) { return; }

                manager.OnEnable();
                return;
            }

            if(GLOBALS.topclient.IsAdmin)
            {
                admin_button.SetActive(true);
                options_button.SetActive(true);
            }
            else
            {
                admin_button.SetActive(false);
                admin_menu.SetActive(false);
                options_button.SetActive(false);
            }

            scoreadj_red_placeholder.text = MyUtils.GetRedAdj();
            scoreadj_blue_placeholder.text = MyUtils.GetBlueAdj();
        }


        // Deal with chat window
        if (chat_text) { UpdateChat(); }

        // Update Admin menus if required
        if (admin_menu)
        {
            if (admin_menu.activeSelf && ( GLOBALS.topclient.players_changed || !old_admin_enable))
            {
                // Update list of players
                admin_kickplayer.ClearOptions();
                
                List<string> newoptions = new List<string>();

                // Add title entry
                newoptions.Add("Kick Player");
                admin_kickplayer.AddOptions(newoptions);
                newoptions.Clear();

                // Add individual players
                foreach (int id in GLOBALS.topclient.players.Keys)
                {
                    newoptions.Add(id + ":" + GLOBALS.topclient.players[id].playerName);
                }

                admin_kickplayer.AddOptions(newoptions);
            }

            old_admin_enable = admin_menu.activeSelf;
        }

        // Deal with recording buttons
        if( GLOBALS.now_recording)
        {
            playback_slider.minValue = MyUtils.PB_GetStartTime();
            playback_slider.maxValue = MyUtils.PB_GetEndTime();
            playback_slider.value = playback_slider.minValue;

            if( GLOBALS.autosave_recordings)
            {
                if (!playback_autosave.gameObject.activeSelf) { playback_autosave.gameObject.SetActive(true); }
                Color curr_color = playback_autosave.color;
                curr_color.a = MyUtils.PB_AutoSavePercentage();
                playback_autosave.color = curr_color;
            }
            else
            {
                if (playback_autosave.gameObject.activeSelf) { playback_autosave.gameObject.SetActive(false); }
            }
        }

        // Deal with recording buttons
        if (GLOBALS.now_playing && !GLOBALS.now_paused)
        {
            PB_ChangeSlider(MyUtils.PB_GetCurrentTime());

            // If we reached the end, stop
            if (MyUtils.PB_ReachedEnd())
            {
                PB_StopPressed(true);
            }
        }

        // Deal with save button
        if( playback_save )
        {
            if (MyUtils.recorded_data.Count > 0)
            {
                if (!playback_save.gameObject.activeSelf)
                {
                    playback_save.gameObject.SetActive(true);
                    playback_save.interactable = true;
                }
            }
            else
            {
                if (playback_save.gameObject.activeSelf)
                {
                    playback_save.gameObject.SetActive(false);
                    playback_save.interactable = false;
                }
            }
        }

    }


    public void LateUpdate()
    {
        if (!initialized)
        {
            if (VRCamera)
            {
                // if in VR mode, enable VR camera, keep regular camera 
                if (MyUtils.XR_isPresent())
                {
                    // MainCamera.SetActive(false);
                    VRCamera.SetActive(true);
                    // EventSystem.GetComponent<StandaloneInputModule>().enabled = false;
                    // EventSystem.GetComponent<OVRInputModule>().enabled = true;
                    // mainCanvas.GetComponent<Canvas>().worldCamera = VRtracking;

                }
                else
                {

                    VRCamera.SetActive(false);
                    // EventSystem.GetComponent<StandaloneInputModule>().enabled = true;
                    // EventSystem.GetComponent<OVRInputModule>().enabled = false;
                    // mainCanvas.GetComponent<Canvas>().worldCamera = MainCamera.GetComponent<Camera>();
                    // MainCamera.SetActive(true);

                }
            }

            initialized = true;

            // If server is not found and this is headless mode, start server
            // If this is headless mode, start server
            if (GLOBALS.HEADLESS_MODE && !GLOBALS.topserver)
            {
                Console.Out.WriteLine("Loading Server....");
                LoadSceneDelayed("Scenes/server");
            }
        }
        
    }
    // On enable sequence will not guarantee canvas is all defined before we are
    // thus will move init to first update
    public void OnEnable()
    {
        // Clean up old scenes
        Resources.UnloadUnusedAssets();
    }

    public void Quit()
    {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
#else
		Application.Quit();
#endif
    }

    public void ShowGameOptions()
    {
        // See if there is a GameMenu object
        var alloptions = Resources.FindObjectsOfTypeAll<GameMenu>();
        if( alloptions.Length <=0 ) { return; }

        foreach (GameMenu currmenu in alloptions)
        {
            currmenu.gameObject.SetActive(true);
        }
    }


    public void LoadScene(string name)
    {
        SceneManager.LoadScene(name);
    }

    public IEnumerator DelaySeconds(float seconds, string name)
    {
        yield return new WaitForSecondsRealtime(seconds);
        LoadScene(name);
    }

    // Delay scene loading
    public void LoadSceneDelayed(string name)
    {
        StartCoroutine(DelaySeconds(0.65f, name));
    }

    public void EnableWindow(string name)
    {

        gui_panel = GameObject.Find(name);
        if (gui_panel == null) { return; }

        gui_animator = gui_panel.GetComponent<Animator>();

        gui_animator.SetBool("Open", true);
    }

    public GameObject settings_panel;
    
    public void ConnectButtonPressed()
    {
        if (GLOBALS.topclient == null) { return; }

        // Get the IP field
        InputField ipfield = settings_panel.transform.Find("IPInput").GetComponent<InputField>();
        if (ipfield == null) { return; }

        // Get the Port field
        InputField portfield = settings_panel.transform.Find("PORTInput").GetComponent<InputField>();
        if (portfield == null) { return; }

        // Get the Name field
        InputField namefield = settings_panel.transform.Find("NameInput").GetComponent<InputField>();
        if (namefield == null) { return; }

        // Save the namefield
        GLOBALS.default_player_name = namefield.text;
        Settings.SavePrefs();

        // Get the password
        InputField passfield = settings_panel.transform.Find("PasswordInput").GetComponent<InputField>();
        if (passfield == null) { return; }

        // Get the start position
        Dropdown startfield = settings_panel.transform.Find("StartPosInput").GetComponent<Dropdown>();
        if (startfield == null) { return; }

        if (!GLOBALS.topclient.Connect(ipfield.text, portfield.text, namefield.text, passfield.text, startfield.options[startfield.value].text))
        { return; }

        // Make panel manager turn off window
        PanelManager manager = GameObject.Find("MenuManager").GetComponent<PanelManager>();
        if (manager == null) { return; }

        manager.CloseCurrent();
        close_client_menu = true;

        // Turn off chat
        chat_text.GetComponent<InputField>().readOnly = false;
        chat_text.GetComponent<InputField>().text = "";
        chat_text.SetActive(false);

    }

    private bool filebrowser_initialized = false;
    public void LoadButtonPressed()
    {

        // Set exclude files
        // FileBrowser.SetExcludedExtensions(".lnk", ".tmp", ".zip", ".rar", ".exe");

        // If already open then quit.
        if( FileBrowser.IsOpen) { return;  }

        // Coroutine 
        StartCoroutine(ShowLoadDialogCoroutine());

    }

    IEnumerator ShowLoadDialogCoroutine()
    {
        // Set filter to be .xrc files only
        FileBrowser.SetFilters(true, new FileBrowser.Filter("xRC Data-Set", ".xrc"));
        FileBrowser.SetDefaultFilter(".xrc");

        // Show a load file dialog and wait for a response from user
        // Load file/folder: both, Allow multiple selection: true
        // Initial path: default (Documents), Initial filename: empty
        // Title: "Load File", Submit button text: "Load"
        yield return FileBrowser.WaitForLoadDialog(FileBrowser.PickMode.Files, false, null, null, "Load Files and Folders", "Load xRC Data");

        // Dialog is closed
        // Print whether the user has selected some files/folders or cancelled the operation (FileBrowser.Success)
        Debug.Log(FileBrowser.Success);

        if (FileBrowser.Success)
        {
            if (GLOBALS.topclient == null) { yield break; }

            if (FileBrowser.Result.Length != 1)
            {
                GLOBALS.topclient.ShowMessage("Did not chose 1 file...");
                yield break;
               
            }

            if( !MyUtils.PB_LoadFromFile(FileBrowser.Result[0]))
            {
                GLOBALS.topclient.ShowMessage("Failed to load " + FileBrowser.Result[0]);
                yield break;
            }

            // Make panel manager turn off window
            PanelManager manager = GameObject.Find("MenuManager").GetComponent<PanelManager>();
            if (manager == null) { yield break; }

            manager.CloseCurrent();
            close_client_menu = true;

            // Turn off chat
            chat_text.GetComponent<InputField>().readOnly = false;
            chat_text.GetComponent<InputField>().text = "";
            chat_text.SetActive(false);

            // Tell client to be in playback mode
            GLOBALS.topclient.InitPlaybackMode();

            // Set all the playback buttons

            // Initialize playback buttons
            GLOBALS.now_playing = false;
            GLOBALS.now_paused = false;
            GLOBALS.now_recording = false;


            // Make sure button exists (application manager is used in places where this is not the case)
            if (!playback_stop) { yield break; }

            playback_stop.gameObject.SetActive(true);
            playback_stop.interactable = true;
            playback_stop.isOn = true;


            PB_StopPressed(true);



        }
    }

    IEnumerator ShowSaveDialogCoroutine()
    {
        // Set filter to be .xrc files only
        FileBrowser.SetFilters(true, new FileBrowser.Filter("xRC Data-Set", ".xrc"));
        FileBrowser.SetDefaultFilter(".xrc");

        // Show a load file dialog and wait for a response from user
        // Load file/folder: both, Allow multiple selection: true
        // Initial path: default (Documents), Initial filename: empty
        // Title: "Load File", Submit button text: "Load"
        yield return FileBrowser.WaitForSaveDialog(FileBrowser.PickMode.Files, false, null, null, "Save xRC Data", "Save");

        // Dialog is closed
        // Print whether the user has selected some files/folders or cancelled the operation (FileBrowser.Success)
        Debug.Log(FileBrowser.Success);

        if (FileBrowser.Success)
        {
            if (GLOBALS.topclient == null) { yield break; }

            if (FileBrowser.Result.Length != 1)
            {
                GLOBALS.topclient.ShowMessage("Did not chose 1 file...");
                yield break;

            }

            GLOBALS.autosave_recordings = FileBrowser.AutoSave;

            if ( MyUtils.PB_SaveToFile(FileBrowser.Result[0]) )
            {
                string message = "File save =  " + FileBrowser.Result[0];

                if (GLOBALS.autosave_recordings)
                {
                    GLOBALS.autosave_filename = FileBrowser.Result[0];
                    message += ", Auto-Save started.";
                }

                GLOBALS.topclient.ShowMessage(message);
            }
            else
            {
                string message = "Failed to save to file " + FileBrowser.Result[0];
                if(GLOBALS.autosave_recordings)
                {
                    message += ", Auto-Save disabled.";
                    GLOBALS.autosave_recordings = false;
                }
                GLOBALS.topclient.ShowMessage(message);

            }
            

            // Print paths of the selected files (FileBrowser.Result) (null, if FileBrowser.Success is false)

            // Read the bytes of the first file via FileBrowserHelpers
            // Contrary to File.ReadAllBytes, this function works on Android 10+, as well
            // byte[] bytes = FileBrowserHelpers.ReadBytesFromFile(FileBrowser.Result[0]);
        }
    }


    public void StartButtonPressed()
    {
        if (GLOBALS.topserver == null) { return; }

        GLOBALS.topserver.top_application_manager = this;

        // Get the Port field
        InputField portfield = settings_panel.transform.Find("PORTInput").GetComponent<InputField>();
        if (portfield == null) { return; }

        // Get the password
        InputField passfield = settings_panel.transform.Find("PasswordInput").GetComponent<InputField>();
        if (passfield == null) { return; }

        // Get the max spectators
        InputField spectators = settings_panel.transform.Find("SPECTATORSInput").GetComponent<InputField>();
        if (spectators == null) { return; }

        // Get the Torunament Mode
        //Toggle tournamentmode = settings_panel.transform.Find("TournamentToggle").GetComponent<Toggle>();
        //if (tournamentmode == null) { return; }

        // Get the comment field
        InputField comment = settings_panel.transform.Find("CommentInput").GetComponent<InputField>();
        if (comment == null) { return; }

        // Get the Register state
        Toggle register = settings_panel.transform.Find("RegisterToggle").GetComponent<Toggle>();
        if( register==null) { return; }

        // Get the router port
        InputField routerport = settings_panel.transform.Find("REGPORTInput").GetComponent<InputField>();
        if (routerport == null) { return; }

        if (routerport.text.Length < 1 )
        { routerport.text = GLOBALS.UDP_PORT.ToString();  }

        if (portfield.text.Length < 1)
        { portfield.text = GLOBALS.UDP_PORT.ToString(); }

        if (spectators.text.Length < 1)
        { spectators.text = "4"; }

        GLOBALS.topserver.ServerStart(int.Parse(portfield.text), passfield.text, int.Parse(spectators.text), comment.text,
            int.Parse(routerport.text), register.isOn, false);

        // Turn off chat
        chat_text.GetComponent<InputField>().readOnly = false;
        chat_text.GetComponent<InputField>().text = "";
        chat_text.SetActive(false);

        // Update Server Options with latest data
        if( ServerOptions == null ) { return;  }
        toggle_tmode.isOn = false;
        toggle_startwhenready.isOn = true;
        toggle_holding.isOn = false;
        inputfield_password.text = passfield.text;
        inputfield_updaterate.text = GLOBALS.SERVER_SEND_UPDATE_DELAY.ToString();
        inputfield_maxdata.text = GLOBALS.UDP_MAX_BYTES_IN_MS.ToString();
        // Make panel manager turn off window
        PanelManager manager = GameObject.Find("MenuManager").GetComponent<PanelManager>();
        if (manager == null) { return; }

        manager.CloseCurrent();

        

    }


    public GameObject RequestsDropdown;
    public GameObject PositionDropdown;

    // Callback for the Request drop-down menu
    public void MultiPlayerRequestCB()
    {
        if (GLOBALS.topclient == null)
        {
            return;
        }

        // Get the value
        Dropdown dropdownscript = RequestsDropdown.GetComponent<Dropdown>();
        if (dropdownscript == null) { return; }

        GLOBALS.topclient.FlagRequest(dropdownscript);
    }


    private SinglePlayer singleplayer;

    public void SinglePlayerRequestCB()
    {
        if (singleplayer == null)
        {
            singleplayer = GameObject.Find("SinglePlayerScript").GetComponent<SinglePlayer>();
        }

        // Make sure we found client
        if (singleplayer == null) { return; }

        // Get the value
        Dropdown dropdownscript = RequestsDropdown.GetComponent<Dropdown>();
        if (dropdownscript == null) { return; }

        singleplayer.FlagRequest(dropdownscript);
    }

    public void SinglePlayerSetPos()
    {
        if (singleplayer == null)
        {
            singleplayer = GameObject.Find("SinglePlayerScript").GetComponent<SinglePlayer>();
        }

        // Make sure we found client
        if (singleplayer == null) { return; }

        // Get the value
        Dropdown playerpos = PositionDropdown.GetComponent<Dropdown>();
        if (playerpos == null) { return; }

        singleplayer.SetPosition(playerpos.options[playerpos.value].text);

    }

    public void TimerButtonPressed()
    {
        GameObject.Find("Scorekeeper").GetComponent<Scorekeeper>().OnTimerClick();
    }

    public void TimerReset()
    {
        GameObject.Find("Scorekeeper").GetComponent<Scorekeeper>().OnTimerReset();
    }

    public void SendChat()
    {
        GLOBALS.keyboard_inuse = false;

        // Don't do anything if escape was pressed
        if (Input.GetKeyDown(KeyCode.Escape))
        { return; }


        if (GLOBALS.topclient)
        {
            GLOBALS.topclient.SendChat(chat_text.GetComponent<InputField>().text );
        }
        else if (GLOBALS.topserver)
        {
            GLOBALS.topserver.SendChat(chat_text.GetComponent<InputField>().text );
        }

        chat_text.GetComponent<InputField>().text = "";
        chat_text.SetActive(false);

    }

    private void UpdateChat()
    {
        // Toggle information screen
        if (Input.GetKeyDown(KeyCode.BackQuote))
        {
            if (!chat_text.activeSelf)
            {
                chat_text.SetActive(true);
                if( chat_text.transform.parent) // Make sure the parent is also enabled in case the canvas was turned off
                {
                    chat_text.transform.parent.gameObject.SetActive(true);

                    // Turn off server info window if present
                    GameObject serverinfowindow = GameObject.Find("ServerInfoWindow");
                    if (serverinfowindow)
                    {
                        serverinfowindow.SetActive(false);
                    }
                }
                chat_text.GetComponent<InputField>().ActivateInputField();
                chat_text.GetComponent<InputField>().Select();
                GLOBALS.keyboard_inuse = true;

            }
            else
            {
                GLOBALS.keyboard_inuse = false;
            }
        }

        if (Input.GetKeyDown(KeyCode.Escape))
        {
            chat_text.GetComponent<InputField>().text = "";
            chat_text.GetComponent<InputField>().DeactivateInputField();
            chat_text.SetActive(false);
            GLOBALS.keyboard_inuse = false;
        }

        // Prevent chat messages from fading if chat is enabled
        if(chat_text.activeSelf)
        {
            if (GLOBALS.topclient)
            {
                GLOBALS.topclient.ResetChatCounter();
            }
            else if (GLOBALS.topserver)
            {
                GLOBALS.topserver.ResetChatCounter();
            }
        }
    }

    // Called when there are any changes in the server options menu
    public void ServerOptionsChange( )
    {

        // Make sure we found client
        if (GLOBALS.topserver == null) { return; }

        // Shouldn't be doing anything here in headless mode
        if( GLOBALS.HEADLESS_MODE) { return; }

        GLOBALS.topserver.top_application_manager = this;


        // Set server settings
        GLOBALS.topserver.tournament_mode = toggle_tmode.isOn;
        GLOBALS.topserver.start_when_ready = toggle_startwhenready.isOn;
        GLOBALS.topserver.holding_mode = toggle_holding.isOn;
        GLOBALS.game_option = dropdown_gameoption.value + 1;
        GLOBALS.topserver.PASSWORD = inputfield_password.text;
        GLOBALS.topserver.ADMIN = inputfield_admin.text;

        long longvalue;
        if( long.TryParse(inputfield_updaterate.text, out longvalue))
        {
            GLOBALS.topserver.UPDATE_DELAY = longvalue;
        }
        if (long.TryParse(inputfield_maxdata.text, out longvalue))
        {
            GLOBALS.topserver.MAX_BYTES = longvalue;
        }
        if (long.TryParse(Tminplayers.text, out longvalue))
        {
            if( longvalue > GLOBALS.PlayerCount)
            {
                longvalue = GLOBALS.PlayerCount;
                Tminplayers.text = longvalue.ToString();
            }
            if( longvalue < 1)
            {
                longvalue = 1;
                Tminplayers.text = longvalue.ToString();
            }
            GLOBALS.TMinPlayers = (int) longvalue;
        }
    }


    // If the server itself changes a menu setting, then server can call this function to update menu
    public void UpdateMenuesToServerSettings()
    {
        toggle_tmode.isOn = GLOBALS.topserver.tournament_mode;
        toggle_startwhenready.isOn = GLOBALS.topserver.start_when_ready;
        Tminplayers.text = GLOBALS.TMinPlayers.ToString();
        toggle_holding.isOn = GLOBALS.topserver.holding_mode;
        dropdown_gameoption.value = GLOBALS.game_option - 1;
        inputfield_password.text = GLOBALS.topserver.PASSWORD;
        inputfield_admin.text = GLOBALS.topserver.ADMIN;
        Tminplayers.text = GLOBALS.PlayerCount.ToString();
    }

    public void RestartGame()
    {
        GLOBALS.topserver.ServerMenu_RestartLevel();
    }

    public void DebugChange(string number_in)
    {
       // server = GameObject.Find("Server").GetComponent<ServerLow>();
       // if (server == null) { return; }

       // server.UDP_SLEEP = float.Parse(number_in);
    }

    // *************************************************
    // ADMIN Function

    public void ADMIN_KickAll()
    {
        if( !GLOBALS.topclient) { return;  }
        GLOBALS.topclient.SendChat("/SERVER KICKALL");
    }

    public void ADMIN_KickPlayer(int index)
    {
        if (!GLOBALS.topclient) { return; }

        // If this is index=0, then don't do anything
        if( index==0) { return;  }

        // Get the name
        string playername_raw = admin_kickplayer.options[index].text;

        string[] playerdata = playername_raw.Split(':');

        // If we don't have 2 items, quit
        if( playerdata.Length < 2) { return; }

        // Kick the player
        GLOBALS.topclient.SendChat("/SERVER KICKID=" + playerdata[0]);
    }

    public void ADMIN_StartGame()
    {
        if (!GLOBALS.topclient) { return; }
        GLOBALS.topclient.SendChat("/SERVER RESTART");
    }

    public void ADMIN_StopGame()
    {
        if (!GLOBALS.topclient) { return; }
        GLOBALS.topclient.SendChat("/SERVER STOP");
    }

    public void ADMIN_SetPassword(Text newpassword)
    {
        if (!GLOBALS.topclient) { return; }

        GLOBALS.topclient.SendChat("/SERVER PASSWORD=" + newpassword.text);
    }

    public void ADMIN_ExitAdmin()
    {
        if (!GLOBALS.topclient) { return; }
        if (!admin_menu) { return; }

        admin_menu.SetActive(false);
    }

    public void ADMIN_ToggleHUD()
    {
        if (!GLOBALS.topclient) { return; }

        GLOBALS.topclient.Toggle_HUD();
    }

    public void ADMIN_EnterAdmin()
    {
        if (!GLOBALS.topclient) { return; }
        if (!admin_menu) { return; }

        admin_menu.SetActive(true);
    }

    public void ADMIN_AdjRED(TMPro.TMP_InputField newvalue)
    {
        if (!GLOBALS.topclient) { return; }
        GLOBALS.topclient.SendChat("/SERVER REDADJ=" + newvalue.text);

        // Clear the text
        newvalue.text = "";
    }

    public void ADMIN_AdjBLUE(TMPro.TMP_InputField newvalue)
    {
        if (!GLOBALS.topclient) { return; }
        GLOBALS.topclient.SendChat("/SERVER BLUEADJ=" + newvalue.text);


        // Clear the text
        newvalue.text = "";
    }

    public void ADMIN_LockKeyboard(bool lockstate)
    {
        GLOBALS.keyboard_inuse = lockstate;
    }

    public void ADMIN_SetOutputScoreFiles(TMPro.TMP_InputField location)
    {
        GLOBALS.OUTPUT_SCORING_FILES = true;
        MyUtils.status_file_dir = location.text;
    }


    public void Set_AutomationFiles(TMPro.TMP_InputField location)
    {
        GLOBALS.AUTOMATION_FILES = true;
        GLOBALS.AUTOMATION_DIR = location.text;
        Settings.SavePrefs();
        MyUtils.ResetAutoFiles();
    }

    public void Set_AutomationState(bool enabled_auto)
    {
        GLOBALS.AUTOMATION_FILES = enabled_auto;
        if( GLOBALS.topsingleplayer && GLOBALS.topsingleplayer.scorer)
        {
            GLOBALS.topsingleplayer.scorer.clean_run = false;
            GLOBALS.topsingleplayer.InitAutoFiles();
            MyUtils.ResetAutoFiles();
        }
    }


    public void PB_SavePressed(bool state)
    {
        if (state) { playback_save.isOn = false; }

        // If this was a script releasing it, do nothing
        if (!state) { return; }

        // If file browser already open, don't do anything
        if (FileBrowser.IsOpen) { return; }

        // Coroutine example
        StartCoroutine(ShowSaveDialogCoroutine());

    }

    public void PB_RecordPressed(bool state)
    {
        // If already recording, do nothing
        if (GLOBALS.now_recording) { return; }

        // If this was a script releasing it, do nothing
        if( !state ) { return;  }

        // Clear old data
        MyUtils.PB_ClearRecording();

        // Set buttons states
        playback_rec.interactable = false;
        playback_rec.isOn = true;

        playback_stop.gameObject.SetActive(true);
        playback_stop.interactable = true;
        playback_stop.isOn = false;

        playback_play.gameObject.SetActive(false);
        playback_play.interactable = false;
        playback_play.isOn = false;

        playback_pause.gameObject.SetActive(false);
        playback_pause.interactable = false;
        playback_pause.isOn = false;

        playback_slider.gameObject.SetActive(false);

        playback_speed.gameObject.SetActive(false);
        playback_speed.interactable = false;

        GLOBALS.now_recording = true;
    }

    public void PB_StopPressed(bool state)
    {
        // Make sure button exists (application manager is used in places where this is not the case)
        if (!playback_stop) { return; }

        // Make sure this was pressed on not a script releasing it
        if (!state) { return; }

        playback_stop.gameObject.SetActive(true);
        playback_stop.interactable = false;
        playback_stop.isOn = false;

        // If already recording, do nothing
        // if (!GLOBALS.now_recording && !GLOBALS.now_playing) { return; }

        // Set buttons states
        playback_rec.gameObject.SetActive(true);
        playback_rec.interactable = true;
        playback_rec.isOn = false;

        playback_pause.gameObject.SetActive(false);
        playback_pause.interactable = false;
        playback_pause.isOn = false;

        playback_speed.gameObject.SetActive(false);
        playback_speed.interactable = false;


        if (MyUtils.PB_GetStartTime() > 0)
        {
            playback_play.gameObject.SetActive(true);
            playback_play.interactable = true;
            playback_play.isOn = false;

            playback_slider.gameObject.SetActive(true);
            playback_slider.interactable = true;
            playback_slider.minValue = MyUtils.PB_GetStartTime();
            playback_slider.maxValue = MyUtils.PB_GetEndTime();

            if ((playback_slider.value < playback_slider.minValue) ||
                (playback_slider.value > playback_slider.maxValue))
            {
                PB_ChangeSlider(playback_slider.minValue);
            }
        }
        else
        {
            playback_play.gameObject.SetActive(false);
            playback_play.interactable = false;
            playback_play.isOn = false;


            playback_slider.gameObject.SetActive(false);
        }

        GLOBALS.now_recording = false;

        if (GLOBALS.now_playing)
        {
            GLOBALS.now_playing = false;
            GLOBALS.now_paused = false;
            if (GLOBALS.topclient)
            {
                GLOBALS.topclient.Playback_Stopped();
            }
        }
    }

    public void PB_PlayPressed(bool state)
    {
        // If this was deselected (by scripts), dont do anything
        if( !state ) { return; }

        /// Calculate the starting position
        float start_time = playback_slider.value;

        // If the slider is at the end, 
        // then stop everything
        if ((long)(start_time + 1f) >= MyUtils.PB_GetEndTime())
        {
            PB_StopPressed(true);
            return;
        }

        // Start playback
        if (!MyUtils.PB_StartPlayback((long)start_time) )
        {
            PB_StopPressed(true);
            return;
        }

        // Set globals
        GLOBALS.now_recording = false;
        GLOBALS.now_playing = true;
        GLOBALS.now_paused = false;

        // Set buttons
        playback_play.gameObject.SetActive(true);
        playback_play.interactable = false;
        playback_play.isOn = true;

        playback_stop.gameObject.SetActive(true);
        playback_stop.interactable = true;
        playback_stop.isOn = false;

        playback_rec.gameObject.SetActive(false);
        playback_rec.interactable = false;
        playback_rec.isOn = false;

        playback_pause.gameObject.SetActive(true);
        playback_pause.interactable = true;
        playback_pause.isOn = false;

        playback_speed.gameObject.SetActive(true);
        playback_speed.interactable = true;
    }

    public void PB_PausedPressed(bool state)
    {
        // If we are not playing, don't do anything
        if( !GLOBALS.now_playing)
        {
            playback_pause.gameObject.SetActive(false);
            playback_pause.interactable = false;
            playback_pause.isOn = false;
            return;
        }

        // If this was selected, then set pause mode on
        if (state) {
            GLOBALS.now_paused = true;    
            return; 
        }

        GLOBALS.now_paused = false;
    }

    bool slider_changed_by_script = false;
    public void PB_SliderChanged(float value)
    {
        // exit if recording on this is a script changing the value
        if (GLOBALS.now_recording || slider_changed_by_script)
        {
            return;
        }

        // If not playing then play
        if (!GLOBALS.now_playing)
        {
            PB_PlayPressed(true);
        }

        // Deselect pause so you can see the slider change values
        //if (GLOBALS.now_paused)
        //{
        //    PB_PausedPressed(false);
        //}

        // Set the state to the current value
        MyUtils.PB_ChangeTime((long) value);
    }

    public void PB_ChangeSlider(float new_value)
    {
        slider_changed_by_script = true;
        playback_slider.value = new_value;
        slider_changed_by_script = false;
    }

    public void PB_ChangeSpeed(int value)
    {
        float speed;

        switch( value ) 
        {
            case (0): speed = 0.25f; break;
            case (1): speed = 0.5f; break;
            case (2): speed = 1f; break;
            case (3): speed = 2f; break;
            case (4): speed = 4f; break;
            default:  speed = 1f; break;
        };

        MyUtils.PB_ChangeSpeed(speed);
    }

    public void CB_Referee(string msg)
    {
        if( GLOBALS.topclient)
        {
            GLOBALS.topclient.CB_Referee(msg);
        }
    }

    public void PointerEnteredActive()
    {
        pointer_over_active += 1;
    }

    public void PointerExitedActive()
    {
        pointer_over_active -= 1;
    }

    public bool IsPointerOverActive()
    {
        return pointer_over_active > 0;
    }
}
