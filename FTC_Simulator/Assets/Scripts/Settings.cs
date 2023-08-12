using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using Object = UnityEngine.Object;




//*************** GLOBAL SETTINGS *********************
//
public class keyinfo
{
    public string key;
    public string details;

    public keyinfo( string inkey, string info)
    {
        key = inkey;
        details = info;
    }

    public string GetString()
    {
        return key + "$!" + details;
    }

    public void FromString(string input)
    {
        string[] splits = { "$!" };

        string[] datain = input.Split(splits, StringSplitOptions.None);

        if (datain.Length < 2)
        { return; }

        key = datain[0];
        details = datain[1];        
    }
}


// THIS NAMESPACE APPLIES TO BOTH CLIENTS AND SERVER
public static class GLOBALS
{
    public static string TWO_LETTER_LANGUAGE = "en";

    public static bool FORCE_REAL = false;
    public static bool FORCE_OLD_BHELP = false;

    public static string VERSION = "v9.0d"; // Version of game
    public static bool HEADLESS_MODE = false; // If true, we are running a server only

    public static bool CLIENT_MODE = false; // If multiplayer mode, this tells everyone that this is just a client
    public static ClientLow topclient = null;

    public static bool SINGLEPLAYER_MODE = false; // If in singleplayer mode
    public static SinglePlayer topsingleplayer = null;

    public static bool SERVER_MODE = false;   // If this is a server
    public static ServerLow topserver = null;

    public static bool SERVER_DEBUG = false; // Used it to turn some debug features on/off that may no longer be in  there
    public static int NETSTATS = 0;
    public static bool AUDIO = true;        // Enables audio
    public static bool ROBOTAUDIO = true;   // Enables Robot sounds effect 
    public static float VOLUME = 0.5f;      // Global volume

    // Logs/ debugging/ etc.. 
    public static bool ENABLE_LOGS = true;
    public static string LOGS_PATH = "." + Path.DirectorySeparatorChar.ToString()  + "logs";  // Put logs in the current directory. This is updated during OnStart from Client and Server functions
    //public static string LOGS_PATH =  "."  + "\\logs";  // Put logs in the current directory
    public static bool USE_STANDARD_LOG = true; // Forces use of standard logging instead of custom log path
    
    // 



    public static bool ENABLE_UDP_STATS = false;
    public static bool UDP_LOGGING = false; // dump UDP data for current places of interest... currently only flag processing has stuff here.
    public static bool OUTPUT_SCORING_FILES = false; // Outputs scoring details to multiple files for OBS (open broadcast streaming)
    public static bool AUTOMATION_FILES = false; // Outputs automation details
    public static string AUTOMATION_DIR = Path.DirectorySeparatorChar.ToString() + "tmp" + Path.DirectorySeparatorChar.ToString() + "xRCsim";

    //#if UNITY_STANDALONE_WIN
    public static Windows.ConsoleWindow win_console = null;
                                                          
//#endif

    // Server Outgoing packets, Client Incoming
    public static int UDP_PORT = 1446;
    public const string PASSCODE = "9p0a";
    public const string GETSERVERINFO = "11115";
    public const string PACKETTYPE_ZIP = "Z";
    public const string PACKETTYPE_TXT = "T";
    public const string HEADER_PLAYERS = "PLYRS";
    public const string HEADER_NEWPLAYER = "NEW";
    public const string HEADER_PLAYER_POS = "4.5";    // Needs to be made of only bit-packing approved characters
    public const string HEADER_FIELDELEMENTS = "3.2"; // Needs to be made of only bit-packing approved characters
    public const string HEADER_ERROR = "ERR";
    public const string HEADER_FLAGS = "FLG"; // both server + client use this
    public const int PACKET_COMPRESSION = 1;    // 0 = none, 1 = 1.67X fast zip, 2 = zlib (3X Reduction but slow),  3 = bitpacking (2X reduction, ultra-fast)
                                                // Bit-packing can only be used on char-reduced inputs
    public const bool SHOW_COMPRESSION_TESTCASE = false; // Shows a test-case that looks at packet compression info
    public const int MAX_TRACKED_PACKETS = 30;    // Max number of packets for Flags to keep in history
    public const long PACKET_LOSS_UPDATE_TIME = 500; // Number of ms to average packet loss over
    public const long FLAG_UPDATE_TIME = 100; // ms delay between flag update time

    public static bool now_recording = false; // Turns on recording of incoming data
    public static bool now_playing = false; // Tturns on playing back data
    public static bool now_paused = false;
    public static float playback_speed = 1f;     // The speed to playback with
    public static int PB_BUFFER_DURATION = 5; // minutes of buffer time to record. Set to 5m
    public static bool autosave_recordings = false;
    public static string autosave_filename = "";


    // Packet Compression setting: for now forcing compression=3 for player-positions and field-elements.
    // All other packets use above setting.

    public const char SEPARATOR1 = (char)17;   // Restriction needed for compression to be optimal 
    public const char SEPARATOR2 = (char)18;   // Restriction needed for compression to be optimal 
    public const char SEPARATOR3 = (char)19;   // Used in Flags, which is not bitpacked...         
    public const char SEPARATOR4 = (char)20;   // Used in Sounds, which is not bitpacked...        
    public const char SEPARATOR5 = (char)21;   // Used in Sounds, which is not bitpacked... 
    public const char DATAFILE_SEPARATOR1 = (char)22; // Used to seperate out data sections in data file
    public const char DATAFILE_SEPARATOR2 = (char)23; // Used to seperate out data sections in data file
    public const char DATAFILE_SEPARATOR3 = (char)24; // Used to seperate sub-section when required

    // Server Incoming packets, Client Outgoing
    public const string HEADER_IN_NEWPLAYER = "NAMEIS";
    public const string HEADER_IN_INPUTS = "MYINPUTS";
    public static float time_after_data_received = 0f; // The time in s from the start of program after all current frame internet packets are received.

    // HTTP Server settings for getting server info
    public const string HTTP_ADDRESS = "http://xrcsimulator.org/game/getserverlist.php";
    public static string HTTP_REGISTRATION = "http://xrcsimulator.org/game/registerserver.php";
    public const long registration_update_rate = 30 * 1000; // Time in milliseonds when to send server registration info. Here's its basically every 30s

    // Variables for license unlocking
    public const string HTTP_LICENSE_ACTIVATE = "https://xrcsimulator.org/wp-json/lmfwc/v2/licenses/activate/";
    public const string HTTP_LICENSE_VALIDATE = "https://xrcsimulator.org/wp-json/lmfwc/v2/licenses/validate/";
    public const string HTTP_LICENSE_KEY      = "ck_9baaa4b34b6149c8b576fd7ddbdd5f971f6a21b8";
    public const string HTTP_LICENSE_SECRET   = "cs_ca7dbf4b6b9c215f9276f1f746dbe568a9e71e59";
    public static List<LicenseData> myLicenses = new List<LicenseData>();

    // General Behavior
    public static long SERVER_SEND_UPDATE_DELAY = 50; // Number of milliseconds to wait (minimum) before sending more updates
    public static long CLIENT_SEND_UPDATE_DELAY = 20; // Number of milliseconds to wait (minimum) before sending more updates
    public static long CLIENT_FLAGS_UPDATE_DELAY = 200; // Number of milliseconds to wait (minimum) before updating flags. These are ID tracked and nothing here is very time sensitive

    public static long CLIENT_CONNECT_RETRY_TIME = 2000;  // Amount of time to retry sending a connect request 
    public static long CLIENT_DISCONNECT_TIMEOUT = 3000; // Amount of time (ms) without server communication to consider a disconnect
    public static long SERVER_DISCONNECT_TIMEOUT = 3000; // Amount of time (ms) without client communication to consider a disconnect, forcing client to be dropped
    public static long SERVER_MESSAGE_COUNT_TIME = 1000; // Amount of time (ms) over which to count client messages for display
    public static int MESSAGES_TO_KEEP = 6; // Number of messages to always keep to re-display when in chat mode
    public static long SERVER_CACHE_REFRESH = 500; // Clear cache every once in a while to force re-send of all packets
    public static bool INTERPOLATE = true;
    public static int CAMERA_AVERAGING = 1;
    public static long STATUS_FILES_UPDATE_PERIOD = 250; // Update status log files every 250ms
    public static int CLIENT_ROBOT_INACTIVITY_TIMEOUT = 600; // Seconds to wait with no activity from a client before kicking them

    // Bandwidth management
    public static long UDP_MAX_BYTES_IN_MS = 3000;  // Max bytes sent in one delay time period
    public static float UDP_DELAY_TIME_MS = 0.5f;   // number of milliseconds to wait when we exceed max instant bytes
    public static long SERVER_MAX_UPDATE_DELAY = 500; // Force a data object update at least every period defined here even if object is not moving

    // Cryptographics and Random Seeds
    public static string XRC_PRIVATEPUBLIC_KEY_XML = "<RSAKeyValue><Modulus>fe8zZC1JIoxuSexi7cQKZA/yA9NKCoP9Gt+OWO6WiYc33iHeoMSdSuD7oCqjT3VEkmMYfKTUJEXblqLMU4hDOw==</Modulus><Exponent>AQAB</Exponent><P>9fucrbEoJIIyzeB15ZE47lNA2MjVrHO9HieZ6DRoUzc=</P><Q>gxATPkPdf+5uEjf0pOqhtpvFFqo9C2qAQhQYJhr22h0=</Q><DP>uctTu4ntFS5Wa1SYGE7JXpH5kASaCAjflpA42sAC8J8=</DP><DQ>BVh1gHeiJCKkaKfRmZxcRidqTXdaEAoi+w74wS0eXl0=</DQ><InverseQ>r/vRnW84CE8vC95CCLDB+45wBlNchp78EgOXZgXUHpQ=</InverseQ><D>BdKHV7xYQ0am2rgZItELgfDSyaZ9J9tOWm23kRkG0LjkR6qJA7PCoaSRIUeaXj4p302xVgXxVLOTsHKs9lhH8Q==</D></RSAKeyValue>";
    public static string XRC_PUBLIC_KEY_XML = "<RSAKeyValue><Modulus>fe8zZC1JIoxuSexi7cQKZA/yA9NKCoP9Gt+OWO6WiYc33iHeoMSdSuD7oCqjT3VEkmMYfKTUJEXblqLMU4hDOw==</Modulus><Exponent>AQAB</Exponent></RSAKeyValue>";
    public static byte[] RANDOMIZER_SEED = System.Text.Encoding.UTF8.GetBytes("BpczAbhmxBqfSidUqvdSncnuAvhmcfqt");



    // ******** Game Settings ********
    // General Game settings
    public static string GAME = "Rapid React";
    public static int GAME_INDEX = 11;
    public static bool forceframerate = false;
    public static int framerate = 60;
    public static float worldscale = 2f;
    public static int PlayerCount = 99; // Will get overriden by game settings
    public static int TMinPlayers = 99;// Tournament mode player count. If higher than player count, player count will be used
    public static int TIMER_TOTAL = 120;
    public static int TIMER_AUTO = 0;
    public static int TIMER_ENDGAME = 30;
    public static int game_option = 1;

    // Overlay options
    public static float OVERLAY_NAME_Y_OFFSET = 100f;
    public static float OVERLAY_NAME_Y_INCREMENT = 7f;

    // ROBOT OPTIONS
    public static string RobotModel = "RR_MiniDrone";
    public static string DriveTrain = "Mecanum";
    public static int DriveTrainIndex = 1;
    public static int RobotModelCategory = 10;
    public static int RobotModelIndex = 0;
    public static long MESSAGE_DISPLAY_PERIOD = 4000;    // Number of ms to display a new message
    public static float friction = 2f;  // ft/s^2 deceleration due to friction. Normally ~15% off acceleration. Applies to general wheel vector
 
    public static int motortypeindex = 0;
    public static float gear_ratio = 20f;
    public static float wheel_diameter = 4f;
    public static float weight = 15f;
    public static float motor_count = 4f;
    public static float turning_scaler = 1f; // % speed reduction for rotation/turning to help eaier control of robot
    public static float turning_priority = 0f; // When forward + turning exceeds max motor power, this is the weighing factor for turning
    public static bool fieldcentric = false;
    public static bool activebreaking = true;
    public static bool tankcontrol = false;

    public static float speed = 1f;  // Calculated - not saved
    public static float acceleration = 1f; // Calculated - not saved
    public static bool I_AM_RED = false; // Used for power-ups to know if invisibility should be translucent
    public static bool I_AM_SPECTATOR = false; // Used for power-ups to know if invisibility should be translucent

    // Camera option
    public static bool camera_follows = true;    // false = normal camera, true = follow player
    public static bool CAMERA_COUNTDOWN_CONTROL = false; // false = normal, true = countdown is happening
    // Video Options
    public static string video_quality = "Good";
    public static string video_resolution = "";
    public static bool video_fullscreen = false;
    public static string skybox = "Cosmic";

    // Generic hash of strings to save to memory
    public static Dictionary<string, string> GENERIC_DATA = new Dictionary<string, string>();

    // Keyboard options
    public static IDictionary<string, keyinfo> KeyboardMap
        = new Dictionary<string, keyinfo>();
      

    // Mark if keyboard is locked down by someone else
    public static bool keyboard_inuse = false;
    public static Dictionary<string, string> keyboard_states = new Dictionary<string, string>();

    // Joystick options
    public static IDictionary<string, JoystickRawInfo> JoystickMap
        = new Dictionary<string, JoystickRawInfo>();

    public static Dictionary<string, string> joystick_states = new Dictionary<string, string>();


    // Other options
    public static string default_player_name = "";
    public static bool settings_loaded = false;
    public static float SOUND_MIN_DISTANCE = 0.3f;
    public static float SOUND_MAX_DISTANCE = 10f;

    // Skin Options: 1st = eyes, 2nd = hats, 3rd = spoilers, 4th = other
    public static Dictionary<int, GameObject> skins_eyes = new Dictionary<int, GameObject>();
    public static Dictionary<int, GameObject> skins_hats = new Dictionary<int, GameObject>();
    public static Dictionary<int, GameObject> skins_spoilers = new Dictionary<int, GameObject>();
    public static Dictionary<int, GameObject> skins_other = new Dictionary<int, GameObject>();
    public const char SKIN_SEPERATOR = ':';
    public static string skins = "0";
    public static string robotskins = "";
    public static List<string> robotskinslist = new List<string>() { "" };
    public static List<string> all_robot_capital_names = new List<string>();


    // ******** Under-The Hood Options ********

    // UDP Algorithm:  0 = udpClient receiveasync  // Causes Unity issues
    //                 1 = Parallel thread, blocking read // Causes unity issues
    //                 2 = Only during Update
    public static int UDP_ALGORITHM = 2;
    public static int LAYER_RobotBoundry = 23;


    // ******** GENERAL EASE-OF-ACCESS VARIABLES  ********
    // Ease of access globals
    public static Dictionary<int, string> client_names = new Dictionary<int, string>();  // Record all id/name pair every assigned
    public static Dictionary<string, int> client_ids = new Dictionary<string, int>();  // Record all name/id pair every assigned

    // ********** Score-files look-up table for descriptive display
    // Joystick options
    public static IDictionary<string, string> ScorefilesDescription
        = new Dictionary<string, string>()
        {
            { "Timer", "Timer" },
            { "NetFPS", "Server Network FPS" },
            { "GameState", "Game State" },
            { "RedADJ", "Red Score Adjustment" },
            { "BlueADJ", "Blue Score Adjustment" },
            { "OPR", "Player Score Contribution" },
            { "ScoreR", "Red Score" },
            { "ScoreB", "Blue Score" },
            { "RBalls", "Red Balls Scored" },
            { "BBalls", "Blue Balls Scored" },
            { "AutoR", "Red Auto Score" },
            { "AutoB", "Blue Auto Score" },
            { "RRows", "Red Rows" },
            { "BRows", "Blue Rows" },
            { "PC_R", "Red Power Cells" },
            { "PC_B", "Blue Power Cells" },
            { "TeleR", "Red Teleop Score" },
            { "TeleB", "Blue Teleop Score" },
            { "EndR", "Red Endgame Score" },
            { "EndB", "Blue Endgame Score" },
            { "PenR", "Penalties Red" },
            { "PenB", "Penalties Blue" },
            { "BlueWP", "Blue Win Point" },
            { "RedWP", "Red Win Point" }
        };

    public static List<string> killingWords = new List<string>()
    {
        "kills",
        "executes",
        "slaughters",
        "murders",
        "finishes",
        "snuffs",
        "neutralizes",
        "massacres",
        "dispatches",
        "liquidates",
        "obliterates",
        "annihilates",
        "exterminates",
        "knocks out",
        "wipes out",
        "defeats",
        "destroys",
        "crushes",
        "eradicates",
        "shatters",
        "wrecks",
        "ends",
        "butchers",
        "creams",
        "decimates",
        "eliminates",
        "puts an end to",
        "rubs out",
        "dismembers"
    };

};


// ************************* SETTINGS MANAGEMENT ******************************

public class Settings : MonoBehaviour
{
    public GameObject robotOptionsPanel = null;
    public GameObject skinsOptionsPanel = null;
    public GameObject controllerOptionsPanel = null;
    public GameObject drivetrainOptionsPanel = null;
    public GameObject otherOptionsPanel = null;
    public GameObject videoOptionsPanel = null;
    public GameObject keyboardGrid = null;
    public GameObject keyboardlineentry = null;
    public GameObject joysticklineentry = null;
    public GameObject joystickGrid = null;
    public GameObject joystickAdvancedMenu = null;
    public GameObject themeup = null;
    public GameObject themedown = null;
    public GameObject theme_graphics = null;
    public TMPro.TMP_Dropdown game_dropdown = null;
    public TMPro.TextMeshPro version_text = null;
    public GameObject ApplyVideoButton = null;
    public int curr_robotskins = 0;
    public TMPro.TextMeshProUGUI status_robot;

    private bool init_done = false;

    private List<string> themes = new List<string>(new string[]
        {           
            "Splish Splash",
            "Relic Recovery",
            "Rover Ruckus",
            "Skystone",
            "Infinite Recharge",
            "Change Up",
            "Bot Royale",
            "Ultimate Goal",
            "Tipping Point",
            "Freight Frenzy",
            "Rapid React",
            "Spin Up",
            "Power Play",
            "Battle Bots"
        });
    public int current_theme = 13; // DEFAULT TO BATTLEBOTS!!! IT'S ROBOT FIGHTING TIME

    private List<bool> allowed_themes = new List<bool>(new bool[]
    {
            true, // "Splish Splash",
            true, //"Relic Recovery",
            true, //"Rover Ruckus",
            true, // "Skystone",
            true, //"Infinite Recharge",
            true, //"Change Up",
            true, // "Bot Royale",
            true, // "Ultimate Goal",
            true, //"Tipping Point",
            true, //"Freight Frenzy",
            true, // "Rapid React",
            true,  // "Spin Up",
            true, //"Battle Bots"

    });

    public Transform ThemesColumn;
    public Dropdown robot_choices;


    // Video setting: list of supported resolutions
    private List<Dropdown.OptionData> resolutionList;
    private List<Dropdown.OptionData> qualityList;

    // Things that need to be set before anything else is loaded
    private void Awake()
    {
        // Save the original language
        GLOBALS.TWO_LETTER_LANGUAGE = CultureInfo.CurrentCulture.TwoLetterISOLanguageName;

        // Set default culture for all threads in app domain
        var culture = new System.Globalization.CultureInfo("en-US");  // US english
        CultureInfo.DefaultThreadCurrentCulture = culture;
        CultureInfo.DefaultThreadCurrentUICulture = culture;

        // remove themes that we dont want
        // The indexing in themesecolum is opposite: 0 = last, last = count-1
        int maxindex = allowed_themes.Count - 1;

        for ( int i = maxindex; i >=0; i--)
        {
            if( !allowed_themes[i])
            {
                // Delete it from the themes list
                themes.RemoveAt(i);

                // Disable current picture
                ThemesColumn.GetChild(maxindex-i).gameObject.SetActive(false);

                // Move all themese down
                for( int j= maxindex-i+1; j<= maxindex; j++)
                {
                    Vector3 cpos = ThemesColumn.GetChild(j).position;
                    cpos.y -= 100f;
                    ThemesColumn.GetChild(j).position = cpos;
                }
            }
        }
        
        
        // Remove the robot choices
        List<Dropdown.OptionData> robotchoices = robot_choices.options;
        for( int i = robotchoices.Count -1; i>=0; i--)
        {
            // See if the choice exists
            String choice = robotchoices[i].text;

            if( choice == "Pushbots") { continue; }
            if( !themes.Contains(choice))
            {
                robotchoices.RemoveAt(i);
            }

        }

        // Put back in
        robot_choices.options = robotchoices;

    }

    // Use this for initialization
    void Start()
    {
        if (version_text)
        {
            version_text.text = GLOBALS.VERSION;
        }


        // Initialize joystick and keyboard settings
        SetKeyboardToDefaults();
        SetJoystickToDefaults();

        // Load and find all the skins
        LoadSkins("Eyes", "Eyes_", GLOBALS.skins_eyes);
        LoadSkins("Hats", "Hat_", GLOBALS.skins_hats);
        LoadSkins("Spoilers", "Spoiler_", GLOBALS.skins_spoilers);
        LoadSkins("Other", "Other_", GLOBALS.skins_other);

        // Retrieve all license keys
        GLOBALS.myLicenses.Clear();
        for (int i = 0; i < 99; i++)
        {
            if (PlayerPrefs.HasKey("license" + i))
            {
                LicenseData.AddLicense(PlayerPrefs.GetString("license" + i));
            }
        }

        // Load preferences 
        if (PlayerPrefs.HasKey("UDP_PORT")) { GLOBALS.UDP_PORT = PlayerPrefs.GetInt("UDP_PORT"); }
        if (PlayerPrefs.HasKey("RobotModel")) { GLOBALS.RobotModel = PlayerPrefs.GetString("RobotModel"); }
        if (PlayerPrefs.HasKey("RobotModelIndex")) { GLOBALS.RobotModelIndex = PlayerPrefs.GetInt("RobotModelIndex"); }
        if (PlayerPrefs.HasKey("RobotModelCategory")) { GLOBALS.RobotModelCategory = PlayerPrefs.GetInt("RobotModelCategory"); }
        if (PlayerPrefs.HasKey("DriveTrain")) { GLOBALS.DriveTrain = PlayerPrefs.GetString("DriveTrain"); }
        if (PlayerPrefs.HasKey("DriveTrainIndex")) { GLOBALS.DriveTrainIndex = PlayerPrefs.GetInt("DriveTrainIndex"); }
        if (PlayerPrefs.HasKey("MotorTypeIndex")) { GLOBALS.motortypeindex = PlayerPrefs.GetInt("MotorTypeIndex"); }
        if (PlayerPrefs.HasKey("GearRatio")) { GLOBALS.gear_ratio = PlayerPrefs.GetFloat("GearRatio"); }
        if (PlayerPrefs.HasKey("WheelDiameter")) { GLOBALS.wheel_diameter = PlayerPrefs.GetFloat("WheelDiameter"); }
        if (PlayerPrefs.HasKey("Weight")) { GLOBALS.weight = PlayerPrefs.GetFloat("Weight"); }
        if (PlayerPrefs.HasKey("turningScaler")) { GLOBALS.turning_scaler = PlayerPrefs.GetFloat("turningScaler"); }
        if (PlayerPrefs.HasKey("turningPriority")) { GLOBALS.turning_priority = PlayerPrefs.GetFloat("turningPriority"); }
        if (PlayerPrefs.HasKey("fieldcentric")) { GLOBALS.fieldcentric = PlayerPrefs.GetInt("fieldcentric") == 1; }
        if (PlayerPrefs.HasKey("activebreaking")) { GLOBALS.activebreaking = PlayerPrefs.GetInt("activebreaking") == 1; }
        if (PlayerPrefs.HasKey("tankcontrol")) { GLOBALS.tankcontrol = PlayerPrefs.GetInt("tankcontrol") == 1; }
        if (PlayerPrefs.HasKey("FrameRate")) { GLOBALS.framerate = PlayerPrefs.GetInt("FrameRate"); }
        if (PlayerPrefs.HasKey("ForceFrameRate")) { GLOBALS.forceframerate = PlayerPrefs.GetInt("ForceFrameRate") == 1; }
        if (PlayerPrefs.HasKey("interpolate")) { GLOBALS.INTERPOLATE = PlayerPrefs.GetInt("interpolate") == 1; }
        if (PlayerPrefs.HasKey("camera_averaging")) { GLOBALS.CAMERA_AVERAGING = PlayerPrefs.GetInt("camera_averaging"); }
        if (PlayerPrefs.HasKey("audio")) { GLOBALS.AUDIO = PlayerPrefs.GetInt("audio") == 1; }
        if (PlayerPrefs.HasKey("robotaudio")) { GLOBALS.ROBOTAUDIO = PlayerPrefs.GetInt("robotaudio") == 1; }
        if (PlayerPrefs.HasKey("volume")) { GLOBALS.VOLUME = PlayerPrefs.GetFloat("volume"); }
        if (PlayerPrefs.HasKey("WorldScale")) { GLOBALS.worldscale = PlayerPrefs.GetFloat("WorldScale"); }
        if (PlayerPrefs.HasKey("video_fullscreen")) { GLOBALS.video_fullscreen = PlayerPrefs.GetInt("video_fullscreen") == 1; }
        if (PlayerPrefs.HasKey("video_quality")) { GLOBALS.video_quality = PlayerPrefs.GetString("video_quality"); }
        if (PlayerPrefs.HasKey("playername")) { GLOBALS.default_player_name = PlayerPrefs.GetString("playername"); }
        if (PlayerPrefs.HasKey("video_resolution")) { GLOBALS.video_resolution = PlayerPrefs.GetString("video_resolution"); }
        if (PlayerPrefs.HasKey("skins")) { GLOBALS.skins = PlayerPrefs.GetString("skins"); }
        if (PlayerPrefs.HasKey("robotskins")) { GLOBALS.robotskins = PlayerPrefs.GetString("robotskins"); }
        if (PlayerPrefs.HasKey("game")) { GLOBALS.GAME = PlayerPrefs.GetString("game"); }
        if (PlayerPrefs.HasKey("gameindex")) { GLOBALS.GAME_INDEX = PlayerPrefs.GetInt("gameindex"); }
        else { GLOBALS.GAME_INDEX = themes.Count - 1; }
        if (PlayerPrefs.HasKey("replay_buffer")) { GLOBALS.PB_BUFFER_DURATION = PlayerPrefs.GetInt("replay_buffer"); }
        if (PlayerPrefs.HasKey("auto_dir")) { GLOBALS.AUTOMATION_DIR = PlayerPrefs.GetString("auto_dir"); }
        if (PlayerPrefs.HasKey("skybox")) { GLOBALS.skybox = PlayerPrefs.GetString("skybox"); }

        // Lets set the frame-rate \
        Application.targetFrameRate = GLOBALS.framerate;
        if ( GLOBALS.forceframerate)
        {
            QualitySettings.vSyncCount = 0;
        }
        else
        {
            QualitySettings.vSyncCount = 1; 
        }
        

        // Keyboard assignemnts
        foreach (string currkey in GLOBALS.KeyboardMap.Keys)
        {
            if (PlayerPrefs.HasKey(currkey)) { GLOBALS.KeyboardMap[currkey].key = PlayerPrefs.GetString(currkey); }
        }

        // Joystick assignemnts
        foreach (string currkey in GLOBALS.JoystickMap.Keys)
        {
            if (PlayerPrefs.HasKey(currkey)) { GLOBALS.JoystickMap[currkey].FromString(PlayerPrefs.GetString(currkey)); }
        }

        LoadJoystickStates();
        LoadKeyboardStates();


        // Get all the generic data
        // Limitted to 1k entries
        GLOBALS.GENERIC_DATA.Clear();
        for (int i = 1; i < 1000; i++)
        {
            if (PlayerPrefs.HasKey("GD_" + i))
            {
                string[] key_value = PlayerPrefs.GetString("GD_" + i).Split('|');

                // Make sure there are only 2 items
                if (key_value.Length != 2) { continue; }

                GLOBALS.GENERIC_DATA.Add(key_value[0], key_value[1]);
            }
            else
            {
                break;
            }
        }

        // Initialize the settings pulldowns and our variables. Assumed you first load default values before doing this

        if (robotOptionsPanel != null)
        {

            // Check sanity for robot model
            if (robotOptionsPanel.transform.Find("Panel/Settings Robot/Category/Dropdown").GetComponent<Dropdown>().options.Count - 1 < GLOBALS.RobotModelCategory)
            {
                GLOBALS.RobotModelCategory = 0;
            }

            string categoryName = robotOptionsPanel.transform.Find("Panel/Settings Robot/Category/Dropdown").GetComponent<Dropdown>().options[GLOBALS.RobotModelCategory].text;


            if (robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + categoryName + "/Dropdown").GetComponent<Dropdown>().options.Count - 1 < GLOBALS.RobotModelIndex)
            {
                GLOBALS.RobotModelIndex = 0;
            }

            robotOptionsPanel.transform.Find("Panel/Settings Robot/Category/Dropdown").GetComponent<Dropdown>().value = GLOBALS.RobotModelCategory;
            robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + categoryName + "/Dropdown").GetComponent<Dropdown>().value = GLOBALS.RobotModelIndex;
            GLOBALS.RobotModel = robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + categoryName + "/Dropdown").GetComponent<Dropdown>().options[GLOBALS.RobotModelIndex].text;
            old_model = GLOBALS.RobotModel;

            // Find all our robot names
            Transform robot_settings = robotOptionsPanel.transform.Find("Panel/Settings Robot");
            GLOBALS.all_robot_capital_names.Clear();

            foreach ( Transform curr_child in robot_settings)
            {
                if( curr_child.name.StartsWith("Model"))
                {
                    Dropdown all_dropdowns = curr_child.GetComponentInChildren<Dropdown>();
                    foreach( Dropdown.OptionData curr_dropdown in all_dropdowns.options)
                    {
                        GLOBALS.all_robot_capital_names.Add(curr_dropdown.text.ToUpper());
                    }
                }
            }
        }

        // Initialize the settings pulldowns and our variables. Assumed you first load default values before doing this
        if (otherOptionsPanel != null)
        {

            // Interpolate
            otherOptionsPanel.transform.Find("Panel/Settings/Interpolate").GetComponent<Toggle>().isOn = GLOBALS.INTERPOLATE;

            // Camera averaging
            otherOptionsPanel.transform.Find("Panel/Settings/CameraSmoothing").GetComponent<InputField>().text = GLOBALS.CAMERA_AVERAGING.ToString();

            // Audio
            otherOptionsPanel.transform.Find("Panel/Settings/Audio").GetComponent<Toggle>().isOn = GLOBALS.AUDIO;
            otherOptionsPanel.transform.Find("Panel/Settings/RobotAudio").GetComponent<Toggle>().isOn = GLOBALS.ROBOTAUDIO;
            otherOptionsPanel.transform.Find("Panel/Settings/Volume").GetComponent<Slider>().value = GLOBALS.VOLUME;
            otherOptionsPanel.transform.Find("Panel/Settings/RecordingTime").GetComponent<Slider>().value = GLOBALS.PB_BUFFER_DURATION;
            otherOptionsPanel.transform.Find("Panel/Settings/RecordingTime/Text").GetComponent<Text>().text = GLOBALS.PB_BUFFER_DURATION.ToString();
            // World Scale
            otherOptionsPanel.transform.Find("Panel/Settings/WorldScale").GetComponent<InputField>().text = GLOBALS.worldscale.ToString();
        }

        if (videoOptionsPanel != null)
        {
            // set frame-rate
            videoOptionsPanel.transform.Find("Panel/Settings/FrameRate").GetComponent<InputField>().text = GLOBALS.framerate.ToString();
            // Set force frame rate
            videoOptionsPanel.transform.Find("Panel/Settings/forceframerate").GetComponent<Toggle>().isOn = GLOBALS.forceframerate;

            // Add all the resolutions
            resolutionList = new List<Dropdown.OptionData>();
            foreach (var curr_res in Screen.resolutions)
            {
                Dropdown.OptionData newitem = new Dropdown.OptionData(curr_res.ToString());
                resolutionList.Add(newitem);
            }

            videoOptionsPanel.transform.Find("Panel/Settings/Resolution/Dropdown").GetComponent<Dropdown>().ClearOptions();
            videoOptionsPanel.transform.Find("Panel/Settings/Resolution/Dropdown").GetComponent<Dropdown>().AddOptions(resolutionList);

            // Pick the resolution if it matches our saved one
            int i = 0;
            for (i = 0; i < resolutionList.Count; i++)
            {
                if (resolutionList[i].text == GLOBALS.video_resolution) { break; }
            }
            videoOptionsPanel.transform.Find("Panel/Settings/Resolution/Dropdown").GetComponent<Dropdown>().value = i;

            // Add all the qualities
            qualityList = new List<Dropdown.OptionData>();
            foreach (var curr_qual in QualitySettings.names)
            {
                Dropdown.OptionData newitem = new Dropdown.OptionData(curr_qual.ToString());
                qualityList.Add(newitem);
            }
            videoOptionsPanel.transform.Find("Panel/Settings/Quality/Dropdown").GetComponent<Dropdown>().ClearOptions();
            videoOptionsPanel.transform.Find("Panel/Settings/Quality/Dropdown").GetComponent<Dropdown>().AddOptions(qualityList);

            // Pick the quality
            for (i = 0; i < qualityList.Count; i++)
            {
                if (qualityList[i].text == GLOBALS.video_quality) { break; }
            }
            videoOptionsPanel.transform.Find("Panel/Settings/Quality/Dropdown").GetComponent<Dropdown>().value = i;

            // Set fullscreen mode
            videoOptionsPanel.transform.Find("Panel/Settings/Full Screen").GetComponent<Toggle>().isOn = GLOBALS.video_fullscreen;

            Dropdown skybox_drop = videoOptionsPanel.transform.Find("Panel/Settings/Skybox/Dropdown").GetComponent<Dropdown>();
            List<Dropdown.OptionData> skybox_options = skybox_drop.options;
            // Set Skybox
            for (i = 0; i < skybox_options.Count; i++)
            {
                if( skybox_options[i].text == GLOBALS.skybox)
                {
                    skybox_drop.value = i;
                    break;
                }
            }

        }

        if (drivetrainOptionsPanel != null)
        {
            // Calculate initial values
            DriveTrainCalcs.CalcDriveTrain(GLOBALS.wheel_diameter, GLOBALS.gear_ratio, GLOBALS.motortypeindex, GLOBALS.weight, out GLOBALS.speed, out GLOBALS.acceleration);

            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>().value = GLOBALS.DriveTrainIndex;
            // drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>().options[GLOBALS.DriveTrainIndex].text = GLOBALS.DriveTrain;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/MotorType/Dropdown").GetComponent<Dropdown>().value = GLOBALS.motortypeindex;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/GearRatio").GetComponent<InputField>().text = GLOBALS.gear_ratio.ToString();
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/wheelDiameter").GetComponent<InputField>().text = GLOBALS.wheel_diameter.ToString();
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Weight").GetComponent<InputField>().text = GLOBALS.weight.ToString();
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed").GetComponent<Slider>().value = GLOBALS.turning_scaler;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed2").GetComponent<InputField>().text = (GLOBALS.turning_scaler*100f).ToString();
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningPriority").GetComponent<InputField>().text = (GLOBALS.turning_priority * 100f).ToString();
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/FieldCentric").GetComponent<Toggle>().isOn = GLOBALS.fieldcentric;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/ActiveBreaking").GetComponent<Toggle>().isOn = GLOBALS.activebreaking;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TankControl").GetComponent<Toggle>().isOn = GLOBALS.tankcontrol;
        }

        // Do the keyboard grid and joystick grid
        LoadJoystickStates();
        LoadKeyboardStates();
        PopulateKeyboardGrid();
        PopulateJoystickGrid();

        // Skins option
        if (skinsOptionsPanel != null)
        {
            // Get all the skin selections
            // Add any skins
            string[] skin_chosen = GLOBALS.skins.Split(GLOBALS.SKIN_SEPERATOR);

            if (skin_chosen.Length > 0)
            {
                skinsOptionsPanel.transform.Find("Panel/Settings/Eyes/InputField").GetComponent<InputField>().text = skin_chosen[0];
            }

            if (skin_chosen.Length > 1)
            {
                skinsOptionsPanel.transform.Find("Panel/Settings/Hat/InputField").GetComponent<InputField>().text = skin_chosen[1];
            }

            if (skin_chosen.Length > 2)
            {
                skinsOptionsPanel.transform.Find("Panel/Settings/Spoiler/InputField").GetComponent<InputField>().text = skin_chosen[2];
            }

            if (skin_chosen.Length > 3)
            {
                skinsOptionsPanel.transform.Find("Panel/Settings/Other/InputField").GetComponent<InputField>().text = skin_chosen[3];
            }

            curr_robotskins = UpdateRobotSkins();
        }

        if (game_dropdown != null)
        {
            game_dropdown.ClearOptions();
            game_dropdown.AddOptions(themes);
        }

        // Set initial Theme
        current_theme = themes.Count;
        MoveThemeToTarget(GLOBALS.GAME_INDEX);

        init_done = true;
        GLOBALS.settings_loaded = true;

// *** DEBUG  ***
// GLOBALS.HEADLESS_MODE = true;
        
        
        // If this is headless mode, deal with it
        if (Application.isBatchMode)
        {
            GLOBALS.HEADLESS_MODE = true;
            Console.Out.WriteLine("****** xRC Simulator *********** ");
            Console.Out.WriteLine("****** Version " + GLOBALS.VERSION + " ************* ");
            Console.Out.WriteLine("Time started: " + DateTime.Now.ToString("dddd, dd MMMM yyyy HH:mm:ss"));
            Console.Out.WriteLine("\n\n");

            ProcessHeadlessCommands();
        }
        else
        {
            // Make sure options menu is initialized correctly.
            OnMenuChanged();
        }
    }

    public void MoveThemeToTarget(int index = 0)
    {
        if (index == current_theme) { return; }

        if (index > current_theme)
        {
            MoveThemeUp(index - current_theme);
        }
        else
        {
            MoveThemeDown(current_theme - index);
        }
    }


    public void ProcessHeadlessCommands()
    {
        // If we're in headless mode, read in all the line commands
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length; i++)
        {
            // Process all command line arguments
            string[] arg_in = args[i].Split('=');

            if (arg_in[0].ToLower().Equals("game"))
            {
                // Make sure a game argument was specified
                if (arg_in.Length < 2)
                {
                    Console.Out.WriteLine("Game incorrectly specified. Specify as GAME=# where # starts from 0. 0 = Splish Splash.");
                    continue;
                }

                //  Make sure it was an int
                int game_num = 0;
                if (!int.TryParse(arg_in[1], out game_num))
                {
                    Console.Out.WriteLine("Game incorrectly specified - unable to parse integer after GAME= command line. Specify as GAME=# where # starts from 0. 0 = Splish Splash.");
                    continue;
                }

                // Make sure Game is in range
                if ((game_num < 0) || (game_num >= themes.Count))
                {
                    Console.Out.WriteLine("Game number out of range. Starting game # = 0 and last game # is " + (themes.Count - 1) + ". You specified " + game_num + ".");
                    continue;
                }

                // set the game
                GLOBALS.GAME = themes[game_num];
                GLOBALS.GAME_INDEX = game_num;

                Console.Out.WriteLine("Game = " + themes[game_num]);
            } else
            if (arg_in[0].ToLower().Equals("framerate"))
            {
                // Make sure a game argument was specified
                if (arg_in.Length < 2)
                {
                    Console.Out.WriteLine("Framerate incorrectly specified. Specify as FRAMERATE=##.");
                    continue;
                }

                //  Make sure it was an int
                int framerate = 0;
                if (!int.TryParse(arg_in[1], out framerate))
                {
                    Console.Out.WriteLine("Framerate incorrectly specified - unable to parse integer after FRAMERATE= command line. Specify as FRAMERATE=##.");
                    continue;
                }

                // Make sure Game is in range
                if ((framerate < 0) || (framerate >= 120))
                {
                    Console.Out.WriteLine("Framerate specified invalid range. Needs to be between 1 and 120.");
                    continue;
                }

                GLOBALS.framerate = framerate;
                Application.targetFrameRate = framerate;
                QualitySettings.vSyncCount = 0;

                Console.Out.WriteLine("FrameRate = " + framerate);
            }


        }
    }

    // Saves all preferences to the 
    public static void SavePrefs()
    {
        if (!GLOBALS.settings_loaded) { return; }

        // Remove all old keys
        PlayerPrefs.DeleteAll();

        // Set preferences 
        PlayerPrefs.SetInt("UDP_PORT", GLOBALS.UDP_PORT);
        PlayerPrefs.SetString("RobotModel", GLOBALS.RobotModel);
        PlayerPrefs.SetString("skins", GLOBALS.skins);
        PlayerPrefs.SetString("robotskins", GLOBALS.robotskins);
        PlayerPrefs.SetInt("RobotModelIndex", GLOBALS.RobotModelIndex);
        PlayerPrefs.SetInt("RobotModelCategory", GLOBALS.RobotModelCategory);
        PlayerPrefs.SetString("DriveTrain", GLOBALS.DriveTrain);
        PlayerPrefs.SetInt("DriveTrainIndex", GLOBALS.DriveTrainIndex);
        PlayerPrefs.SetInt("MotorTypeIndex", GLOBALS.motortypeindex);
        PlayerPrefs.SetFloat("GearRatio", GLOBALS.gear_ratio);
        PlayerPrefs.SetFloat("WheelDiameter", GLOBALS.wheel_diameter);
        PlayerPrefs.SetFloat("Weight", GLOBALS.weight);
        PlayerPrefs.SetFloat("turningScaler", GLOBALS.turning_scaler);
        PlayerPrefs.SetFloat("turningPriority", GLOBALS.turning_priority);
        PlayerPrefs.SetInt("fieldcentric", (GLOBALS.fieldcentric) ? 1 : 0);
        PlayerPrefs.SetInt("activebreaking", (GLOBALS.activebreaking) ? 1 : 0);
        PlayerPrefs.SetInt("tankcontrol", (GLOBALS.tankcontrol) ? 1 : 0);
        PlayerPrefs.SetInt("FrameRate", GLOBALS.framerate);
        PlayerPrefs.SetInt("ForceFrameRate", (GLOBALS.forceframerate) ? 1 : 0);
        PlayerPrefs.SetInt("interpolate", (GLOBALS.INTERPOLATE) ? 1 : 0);
        PlayerPrefs.SetInt("camera_averaging", GLOBALS.CAMERA_AVERAGING);
        PlayerPrefs.SetInt("audio", (GLOBALS.AUDIO) ? 1 : 0);
        PlayerPrefs.SetInt("robotaudio", (GLOBALS.ROBOTAUDIO) ? 1 : 0);
        PlayerPrefs.SetFloat("volume", GLOBALS.VOLUME);
        PlayerPrefs.SetFloat("WorldScale", GLOBALS.worldscale);
        PlayerPrefs.SetInt("video_fullscreen", (GLOBALS.video_fullscreen) ? 1 : 0);
        PlayerPrefs.SetString("video_quality", GLOBALS.video_quality);
        PlayerPrefs.SetString("playername", GLOBALS.default_player_name);
        PlayerPrefs.SetString("video_resolution", GLOBALS.video_resolution);
        PlayerPrefs.SetString("game", GLOBALS.GAME);
        PlayerPrefs.SetInt("gameindex", GLOBALS.GAME_INDEX);
        PlayerPrefs.SetInt("replay_buffer", GLOBALS.PB_BUFFER_DURATION);
        PlayerPrefs.SetString("auto_dir", GLOBALS.AUTOMATION_DIR);
        PlayerPrefs.SetString("skybox", GLOBALS.skybox);



        // Keyboard assignemnts
        foreach (string currkey in GLOBALS.KeyboardMap.Keys)
        {
            PlayerPrefs.SetString(currkey, GLOBALS.KeyboardMap[currkey].key);
        }

        // Joystick assignments
        foreach (string currkey in GLOBALS.JoystickMap.Keys)
        {
            PlayerPrefs.SetString(currkey, GLOBALS.JoystickMap[currkey].GetString());
        }

        SaveJoystickStates();
        SaveKeyboardStates();

        // Save license keys
        int i = 0;

        foreach (LicenseData currlicense in GLOBALS.myLicenses)
        {
            if (currlicense.DaysLeft() >= 0)
            {
                PlayerPrefs.SetString("license" + i++, currlicense.ToString());
            }
        }

        // Generic User Data
        // Danger here is the data can only grow, never shrink (no way of pruning old data)
        i = 1;
        foreach (string currkey in GLOBALS.GENERIC_DATA.Keys)
        {
            PlayerPrefs.SetString("GD_" + i++, currkey + "|" + GLOBALS.GENERIC_DATA[currkey]);
        }

        PlayerPrefs.Save();
    }

    private void LoadSkins(string dir_name, string file_header, Dictionary<int, GameObject> prefabs)
    {
        // Clear the prefabs
        prefabs.Clear();

        // Load all the prefabs
        for (int i = 1; i < 100; i++)
        {
            GameObject curr_prefab = Resources.Load("Skins/" + dir_name + "/" + file_header + i.ToString()) as GameObject;
            if (!curr_prefab) { return; }

            prefabs.Add(i, curr_prefab);
        }
    }

    private GameObject robot_on_display = null;
    public GameObject robot_showcase = null;
    public GameObject license_locked = null;
    public Text license_daysleft = null;

    private string old_model = "";

    public void OnMenuChanged()
    {
        // When we initialize the settings, it triggers this callback... need to ignore it until everything is initialized
        if (!init_done) { return; }

        // Update our globals to the new menu items
        // Initialize the settings pulldowns and our variables. Assumed you first load default values before doing this
        if (robotOptionsPanel != null)
        {
            Dropdown catdropdown = robotOptionsPanel.transform.Find("Panel/Settings Robot/Category/Dropdown").GetComponent<Dropdown>();
            GLOBALS.RobotModelCategory = catdropdown.value;
            string categoryName = catdropdown.options[GLOBALS.RobotModelCategory].text;

            // Hide all menues except our category
            foreach (Dropdown.OptionData currCategory in catdropdown.options)
            {
                robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + currCategory.text).gameObject.SetActive(false);
            }
            robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + categoryName).gameObject.SetActive(true);

            GLOBALS.RobotModelIndex = robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + categoryName + "/Dropdown").GetComponent<Dropdown>().value;
            GLOBALS.RobotModel = robotOptionsPanel.transform.Find("Panel/Settings Robot/Model " + categoryName + "/Dropdown").GetComponent<Dropdown>().options[GLOBALS.RobotModelIndex].text;
            status_robot.text = GLOBALS.RobotModel;

            // If the model changed, we have to re-seed and find robot skins
            if (old_model != GLOBALS.RobotModel)
            {
                GLOBALS.robotskins = "Real";
                curr_robotskins = UpdateRobotSkins();
                old_model = GLOBALS.RobotModel;
                LoadJoystickState(GLOBALS.RobotModel);
            }

        }

        // Update our globals to the new menu items
        // Initialize the settings pulldowns and our variables. Assumed you first load default values before doing this
        if (otherOptionsPanel != null)
        {
            // Get interpolate
            GLOBALS.INTERPOLATE = otherOptionsPanel.transform.Find("Panel/Settings/Interpolate").GetComponent<Toggle>().isOn;

            // Get camera averaging
            int camera_averaging = 1;
            if (!int.TryParse(otherOptionsPanel.transform.Find("Panel/Settings/CameraSmoothing").GetComponent<InputField>().text, out camera_averaging))
            {
                camera_averaging = 1;
            }

            if (camera_averaging < 1) { camera_averaging = 1; }
            if (camera_averaging > 20) { camera_averaging = 20; }
            otherOptionsPanel.transform.Find("Panel/Settings/CameraSmoothing").GetComponent<InputField>().text = camera_averaging.ToString();
            GLOBALS.CAMERA_AVERAGING = camera_averaging;



            // Get world-scale and correct for insane numbers
            float rawscale = 2f;
            if (!float.TryParse(otherOptionsPanel.transform.Find("Panel/Settings/WorldScale").GetComponent<InputField>().text, out rawscale))
            {
                rawscale = 2f;
            }

            if (rawscale > 4f) { rawscale = 4f; }
            if (rawscale < 0.1f) { rawscale = 0.1f; }

            otherOptionsPanel.transform.Find("Panel/Settings/WorldScale").GetComponent<InputField>().text = rawscale.ToString();
            GLOBALS.worldscale = rawscale;

            // Get Audio
            GLOBALS.AUDIO = otherOptionsPanel.transform.Find("Panel/Settings/Audio").GetComponent<Toggle>().isOn;
            GLOBALS.ROBOTAUDIO = otherOptionsPanel.transform.Find("Panel/Settings/RobotAudio").GetComponent<Toggle>().isOn;
            GLOBALS.VOLUME = otherOptionsPanel.transform.Find("Panel/Settings/Volume").GetComponent<Slider>().value;
            GLOBALS.PB_BUFFER_DURATION = (int)otherOptionsPanel.transform.Find("Panel/Settings/RecordingTime").GetComponent<Slider>().value;
            otherOptionsPanel.transform.Find("Panel/Settings/RecordingTime/Text").GetComponent<Text>().text = GLOBALS.PB_BUFFER_DURATION.ToString();
            AudioListener.volume = GLOBALS.VOLUME;
        }


        if ((videoOptionsPanel != null) && (videoOptionsPanel.transform.Find("Panel/Settings/Resolution/Dropdown").GetComponent<Dropdown>().options.Count > 0))
        {
            // Video options is a tricky one that can easily fault, thus put it in a try
            try
            {
                // Get frame-rate and correct for insane numbers
                int maxframerate = 60;
                if (!int.TryParse(videoOptionsPanel.transform.Find("Panel/Settings/FrameRate").GetComponent<InputField>().text, out maxframerate))
                {
                    maxframerate = 60;
                }

                if (maxframerate < 15) { maxframerate = 15; }
                videoOptionsPanel.transform.Find("Panel/Settings/FrameRate").GetComponent<InputField>().text = maxframerate.ToString();
                GLOBALS.framerate = maxframerate;

                GLOBALS.forceframerate = videoOptionsPanel.transform.Find("Panel/Settings/forceframerate").GetComponent<Toggle>().isOn;

                Application.targetFrameRate = maxframerate;

                if (GLOBALS.forceframerate)
                {
                    QualitySettings.vSyncCount = 0;
                }
                else
                {
                    QualitySettings.vSyncCount = 1;
                }

                // Retrieve all the values
                int resolution_index = videoOptionsPanel.transform.Find("Panel/Settings/Resolution/Dropdown").GetComponent<Dropdown>().value;
                GLOBALS.video_resolution = videoOptionsPanel.transform.Find("Panel/Settings/Resolution/Dropdown").GetComponent<Dropdown>().options[resolution_index].text;

                int quality_index = videoOptionsPanel.transform.Find("Panel/Settings/Quality/Dropdown").GetComponent<Dropdown>().value;
                GLOBALS.video_quality = videoOptionsPanel.transform.Find("Panel/Settings/Quality/Dropdown").GetComponent<Dropdown>().options[quality_index].text;

                GLOBALS.video_fullscreen = videoOptionsPanel.transform.Find("Panel/Settings/Full Screen").GetComponent<Toggle>().isOn;

                // Now set the screen options

                // In headless mode, set it to lowest quality
                if (GLOBALS.HEADLESS_MODE)
                {
                    quality_index = 0;
                }

                QualitySettings.SetQualityLevel(quality_index, true);

                // Do NOT change resolution here unless we're in full screen mode, the program remembers old resolution without our own intervention if in window mode....
                if (GLOBALS.video_fullscreen)
                {
                    Screen.SetResolution(Screen.resolutions[resolution_index].width, Screen.resolutions[resolution_index].height, GLOBALS.video_fullscreen, Screen.resolutions[resolution_index].refreshRate);
                }
                else
                {
                    Screen.SetResolution(Screen.width, Screen.height, GLOBALS.video_fullscreen);
                }


                // Do Skybox
                int skybox_index = videoOptionsPanel.transform.Find("Panel/Settings/Skybox/Dropdown").GetComponent<Dropdown>().value;
                GLOBALS.skybox = videoOptionsPanel.transform.Find("Panel/Settings/Skybox/Dropdown").GetComponent<Dropdown>().options[skybox_index].text;
            }
            catch (Exception e)
            {
                // Do nothing
            }

        }

        if (drivetrainOptionsPanel != null)
        {
            GLOBALS.DriveTrainIndex = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>().value;
            GLOBALS.DriveTrain = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>().options[GLOBALS.DriveTrainIndex].text;
            GLOBALS.motortypeindex = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/MotorType/Dropdown").GetComponent<Dropdown>().value;
            GLOBALS.gear_ratio = float.Parse(drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/GearRatio").GetComponent<InputField>().text);
            GLOBALS.wheel_diameter = float.Parse(drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/wheelDiameter").GetComponent<InputField>().text);
            GLOBALS.weight = float.Parse(drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Weight").GetComponent<InputField>().text);
            GLOBALS.turning_scaler = float.Parse(drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed2").GetComponent<InputField>().text)/100f;
            GLOBALS.turning_priority = float.Parse(drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningPriority").GetComponent<InputField>().text)/100f;
            GLOBALS.fieldcentric = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/FieldCentric").GetComponent<Toggle>().isOn;
            GLOBALS.activebreaking = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/ActiveBreaking").GetComponent<Toggle>().isOn;
            GLOBALS.tankcontrol = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TankControl").GetComponent<Toggle>().isOn;
        }

        // Skins option
        if (skinsOptionsPanel != null)
        {
            int index = int.Parse(skinsOptionsPanel.transform.Find("Panel/Settings/Eyes/InputField").GetComponent<InputField>().text);
            if (index < 0) { skinsOptionsPanel.transform.Find("Panel/Settings/Eyes/InputField").GetComponent<InputField>().text = "0"; }
            if (index > GLOBALS.skins_eyes.Count) { skinsOptionsPanel.transform.Find("Panel/Settings/Eyes/InputField").GetComponent<InputField>().text = GLOBALS.skins_eyes.Count.ToString(); }

            index = int.Parse(skinsOptionsPanel.transform.Find("Panel/Settings/Hat/InputField").GetComponent<InputField>().text);
            if (index < 0) { skinsOptionsPanel.transform.Find("Panel/Settings/Hat/InputField").GetComponent<InputField>().text = "0"; }
            if (index > GLOBALS.skins_hats.Count) { skinsOptionsPanel.transform.Find("Panel/Settings/Hat/InputField").GetComponent<InputField>().text = GLOBALS.skins_hats.Count.ToString(); }

            index = int.Parse(skinsOptionsPanel.transform.Find("Panel/Settings/Spoiler/InputField").GetComponent<InputField>().text);
            if (index < 0) { skinsOptionsPanel.transform.Find("Panel/Settings/Spoiler/InputField").GetComponent<InputField>().text = "0"; }
            if (index > GLOBALS.skins_spoilers.Count) { skinsOptionsPanel.transform.Find("Panel/Settings/Spoiler/InputField").GetComponent<InputField>().text = GLOBALS.skins_spoilers.Count.ToString(); }

            index = int.Parse(skinsOptionsPanel.transform.Find("Panel/Settings/Other/InputField").GetComponent<InputField>().text);
            if (index < 0) { skinsOptionsPanel.transform.Find("Panel/Settings/Other/InputField").GetComponent<InputField>().text = "0"; }
            if (index > GLOBALS.skins_other.Count) { skinsOptionsPanel.transform.Find("Panel/Settings/Other/InputField").GetComponent<InputField>().text = GLOBALS.skins_other.Count.ToString(); }

            // Get all the skin selections
            GLOBALS.skins = skinsOptionsPanel.transform.Find("Panel/Settings/Eyes/InputField").GetComponent<InputField>().text;
            GLOBALS.skins += GLOBALS.SKIN_SEPERATOR;
            GLOBALS.skins += skinsOptionsPanel.transform.Find("Panel/Settings/Hat/InputField").GetComponent<InputField>().text;
            GLOBALS.skins += GLOBALS.SKIN_SEPERATOR;
            GLOBALS.skins += skinsOptionsPanel.transform.Find("Panel/Settings/Spoiler/InputField").GetComponent<InputField>().text;
            GLOBALS.skins += GLOBALS.SKIN_SEPERATOR;
            GLOBALS.skins += skinsOptionsPanel.transform.Find("Panel/Settings/Other/InputField").GetComponent<InputField>().text;

            // Get the robotskins options
            if (curr_robotskins < 0) { curr_robotskins = GLOBALS.robotskinslist.Count - 1; }
            if (curr_robotskins >= GLOBALS.robotskinslist.Count) { curr_robotskins = 0; }

            if (GLOBALS.robotskinslist.Count < 1) { GLOBALS.robotskins = ""; }
            else { GLOBALS.robotskins = GLOBALS.robotskinslist[curr_robotskins]; }
        }

        // Save any keyboard changes
        // Copy over all the keyboard settings to our global hash table
        foreach (string currname in GLOBALS.KeyboardMap.Keys) // try to find an entry for each entry in keyboard hash
        {
            Transform currkey = keyboardGrid.transform.Find(currname);

            if (currkey != null)
            {
                GLOBALS.KeyboardMap[currname].key = currkey.Find("InputField").GetComponent<InputField>().text;
            }
        }

        SavePrefs();

        // Turn off audio in main scene if AudioSource is present
        if (!GLOBALS.AUDIO)
        {
            foreach (AudioSource currsource in GameObject.FindObjectsOfType<AudioSource>())
            {
                currsource.Stop();
            }
        }

        // Instantiate new robot for showing model
        if (robot_on_display)
        {
            Destroy(robot_on_display);
        }

        robot_on_display = MyUtils.InstantiateRobot(GLOBALS.RobotModel, Vector3.zero, Quaternion.identity, GLOBALS.skins, GLOBALS.robotskins);
        robot_on_display.transform.SetParent(robot_showcase.transform);
        robot_on_display.transform.localPosition = Vector3.zero;
        robot_on_display.transform.localRotation = Quaternion.identity;

        // Remove interpolation
        RobotInterface3D robot_ri3d = robot_on_display.GetComponentInChildren<RobotInterface3D>();
        if( robot_ri3d && robotOptionsPanel)
        {
            Transform robotinfo = MyUtils.FindHierarchy(robotOptionsPanel.transform, "RobotInfo");
            robotinfo.GetComponent<Text>().text = robot_ri3d.info;
            robot_ri3d.SetUserParameters();

            // Update locking of all parameters as required
            LockRobotParameters(robot_ri3d);

            interpolation[] all_interpolations = robot_ri3d.GetComponentsInChildren<interpolation>();
            foreach( interpolation currint in all_interpolations )
            {
                currint.enabled = false;
            }
        }

        license_daysleft.text = "";

        // See if we have a license for this robot
        if (LicenseData.CheckRobotIsUnlocked(GLOBALS.RobotModel, GLOBALS.robotskins))
        {
            // Unlock robot
            license_locked.SetActive(false);
            int daysleft = LicenseData.GetFeatureDaysLeft(GLOBALS.RobotModel, GLOBALS.robotskins);
            if (daysleft < 9999)
            {
                license_daysleft.text = daysleft.ToString() + " Days";
            }

        }
        else
        {
            // Lock it up
            license_locked.SetActive(true);
        }


        // Clear old memory
        Resources.UnloadUnusedAssets();

        // Clear apply 
        ApplyVideoButton.SetActive(false);
    }

    private void LockRobotParameters(RobotInterface3D ri3d)
    {
        if(ri3d.total_weight_lock)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Weight").GetComponent<InputField>().interactable = false;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Weight").GetComponent<InputField>().text = ri3d.total_weight.ToString();

        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Weight").GetComponent<InputField>().interactable = true;
        }


        if( ri3d.max_speed_lock || ri3d.max_acceleration_lock || ri3d.lock_all_parameters)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/GearRatio").GetComponent<InputField>().interactable = false;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/wheelDiameter").GetComponent<InputField>().interactable = false;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Weight").GetComponent<InputField>().interactable = false;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/MotorType/Dropdown").GetComponent<Dropdown>().interactable = false;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Graph/notused").GetComponent<Image>().enabled = true;
        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/GearRatio").GetComponent<InputField>().interactable = true;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/wheelDiameter").GetComponent<InputField>().interactable = true;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/MotorType/Dropdown").GetComponent<Dropdown>().interactable = true;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Graph/notused").GetComponent<Image>().enabled = false;
        }

        if( ri3d.DriveTrain_lock)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>().interactable = false;
        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>().interactable = true;
        }

        if( ri3d.turn_scale_lock)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed").GetComponent<Slider>().interactable = false;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed2").GetComponent<InputField>().interactable = false;
        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed").GetComponent<Slider>().interactable = true;
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed2").GetComponent<InputField>().interactable = true;
        }

        if( ri3d.fieldcentric_lock)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/FieldCentric").GetComponent<Toggle>().interactable = false;
        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/FieldCentric").GetComponent<Toggle>().interactable = true;
        }
 
        if( ri3d.activebreaking_lock)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/ActiveBreaking").GetComponent<Toggle>().interactable = false;
        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/ActiveBreaking").GetComponent<Toggle>().interactable = true;
        }
   
        if( ri3d.tankcontrol_lock)
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TankControl").GetComponent<Toggle>().interactable = false;
        }
        else
        {
            drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TankControl").GetComponent<Toggle>().interactable = true;
        }

        // correct for proper drivetrain
        // Assumes first Tank drive is always allowed
        Dropdown dt_dropdown = drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/Drivetrain/Dropdown").GetComponent<Dropdown>();

        if (! ri3d.valid_DriveTrains.Contains(GLOBALS.DriveTrain))
        {
            dt_dropdown.value = 0;
            GLOBALS.DriveTrainIndex = 0 ;
            GLOBALS.DriveTrain = dt_dropdown.options[GLOBALS.DriveTrainIndex].text;
        }

        // Mark all valid drivetrains
        foreach( Dropdown.OptionData currdata in dt_dropdown.options)
        {
            // Clear any old invalid marker
            if(currdata.text.StartsWith("[X] "))
            {
                currdata.text = currdata.text.Substring(4);
            }

            // Check if this is a valid drivetrain
            if(!ri3d.valid_DriveTrains.Contains(currdata.text) )
            {
                currdata.text = "[X] " + currdata.text;
            }    
        }



    }

    public void OnTurningScaler2Changed()
    {
        GLOBALS.turning_scaler = float.Parse(drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed2").GetComponent<InputField>().text)/100f;
        if(GLOBALS.turning_scaler < 0.1f)
        {
            GLOBALS.turning_scaler = 0.1f;
        }

        if (GLOBALS.turning_scaler > 1f)
        {
            GLOBALS.turning_scaler = 1f;
        }

        drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed2").GetComponent<InputField>().text = (GLOBALS.turning_scaler * 100f).ToString();
        drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningSpeed").GetComponent<Slider>().value = GLOBALS.turning_scaler;
    }

    public void OnTurningPriorityChanged(string newvalue)
    {
        GLOBALS.turning_priority = float.Parse(newvalue) / 100f;
        if (GLOBALS.turning_priority < 0f)
        {
            GLOBALS.turning_priority = 0f;
        }

        if (GLOBALS.turning_priority > 1f)
        {
            GLOBALS.turning_priority = 1f;
        }

        drivetrainOptionsPanel.transform.Find("Panel/Settings Drivetrain/TurningPriority").GetComponent<InputField>().text = (GLOBALS.turning_priority * 100f).ToString();
    }

    public int UpdateRobotSkins() // Returns index of the skin assigned
    {
        // Clear out old data
        GLOBALS.robotskinslist.Clear();

        // Find all the robot skins 
        Object[] skins = Resources.LoadAll("Robots/Skins/" + GLOBALS.RobotModel, typeof(GameObject));

        if ((skins != null) && (skins.Length > 0))
        {
            // Add all skins
            foreach (Object currskin in skins)
            {
                // Add name to our list
                GLOBALS.robotskinslist.Add(currskin.name);
            }

            // Return the skin selected
            if (GLOBALS.robotskinslist.Contains(GLOBALS.robotskins))
            {
                return GLOBALS.robotskinslist.IndexOf(GLOBALS.robotskins);
            }

            // If default exists, set robotskins to defaut
            if (GLOBALS.robotskinslist.Contains("Default"))
            {
                GLOBALS.robotskins = "Default";
                return GLOBALS.robotskinslist.IndexOf(GLOBALS.robotskins);

            }
        }

        GLOBALS.robotskins = "";
        return -1;
    }


    public void MoveThemeUp(int steps = 1)
    {
        if (theme_graphics == null) { return; }

        // Make sure there isn't an existing animation
        if (theme_graphics.GetComponent<ThemeAnimation>().animation_started)
        { return; }

        int old_theme = current_theme;
        current_theme += steps;

        if (current_theme > themes.Count - 1)
        {
            current_theme = themes.Count - 1;
            steps = current_theme - old_theme;
        }

        // Start Animation
        theme_graphics.GetComponent<ThemeAnimation>().MoveUp((float)steps);

        if (current_theme >= themes.Count - 1)
        {
            GLOBALS.GAME = themes[themes.Count - 1];

            themeup.SetActive(false);
            themedown.SetActive(true);
        }
        else
        {
            themeup.SetActive(true);
            themedown.SetActive(true);
        }

        GLOBALS.GAME = themes[current_theme];
        GLOBALS.GAME_INDEX = current_theme;
        SavePrefs();

        game_dropdown.value = current_theme;

        return;
    }

    public void MoveThemeDown(int steps = 1)
    {
        if (theme_graphics == null) { return; }

        // Sanity check
        if (theme_graphics.GetComponent<ThemeAnimation>().animation_started)
        { return; }

        int old_theme = current_theme;
        current_theme -= steps;

        if (current_theme < 0)
        {
            current_theme = 0;
            steps = old_theme - current_theme;
        }

        // Start Animation
        theme_graphics.GetComponent<ThemeAnimation>().MoveDown((float)steps);

        if (current_theme < themes.Count - 1)
        {
            themeup.SetActive(true);
        }
        else
        {
            themeup.SetActive(false);
        }

        if (current_theme <= 0)
        {
            themedown.SetActive(false);
        }
        else
        {
            themedown.SetActive(true);
        }

        GLOBALS.GAME = themes[current_theme];
        GLOBALS.GAME_INDEX = current_theme;
        SavePrefs();

        game_dropdown.value = current_theme;

        return;
    }

    public void ResetKeyboardToDefaults()
    {
        // Change table to defaults
        SetKeyboardToDefaults();

        // Save changes
        SavePrefs();

        // Reload menu based on this
        PopulateKeyboardGrid();

        // Unlock global keyboard
        GLOBALS.keyboard_inuse = false;
    }

    public void ResetJoystickToDefaults()
    {
        // Change table to defaults
        SetJoystickToDefaults();

        // Save changes
        SavePrefs();

        // Reload menu based on this
        PopulateJoystickGrid();

        // Unlock global keyboard
        GLOBALS.keyboard_inuse = false;
    }

    public void SetKeyboardToDefaults()
    {
        GLOBALS.KeyboardMap.Clear();


        GLOBALS.KeyboardMap = new Dictionary<string, keyinfo>()
        {
            // {"switch_camera", new keyinfo("SPACE","Switch Camera" ) },
            {"controlls_A", new keyinfo("u","Button A" ) },
            {"controlls_B", new keyinfo("o", "Button B" )},
            {"controlls_X", new keyinfo("i", "Button X") },
            {"controlls_Y", new keyinfo("k", "Button Y") },
            {"controlls_LD", new keyinfo("q", "Left Trigger")  },
            {"controlls_RD", new keyinfo("z", "Right Trigger") },
            {"controlls_LT", new keyinfo("e", "Slow Turn L") },
            {"controlls_RT", new keyinfo("c", "Slow Turn R") },
            {"controlls_turn_l", new keyinfo("j", "Turn Left") },
            {"controlls_turn_r", new keyinfo("l", "Turn Right") },
            {"controlls_move_u", new keyinfo("w", "Move Up") },
            {"controlls_move_d", new keyinfo("s", "Move Down") },
            {"controlls_move_l", new keyinfo("a", "Move Left") },
            {"controlls_move_r", new keyinfo("d", "Move Right") },
            {"controlls_dpad_u", new keyinfo("t", "D-Pad Up") },
            {"controlls_dpad_l", new keyinfo("f", "D-Pad Left") },
            {"controlls_dpad_d", new keyinfo("g", "D-Pad Down") },
            {"controlls_dpad_r", new keyinfo("h", "D-Pad Right") },
            {"controlls_stop", new keyinfo("[","Stop Timer" ) },
            {"controlls_restart", new keyinfo("]","Start Timer" ) },
            {"controlls_reset", new keyinfo("\\","Reset Position" ) },
            {"controlls_rightstick_up", new keyinfo("p","Right-Stick Up" ) },
            {"controlls_rightstick_down", new keyinfo(";","Right-Stick Down" ) },
        };
    }

    public void PopulateKeyboardGrid()
    {
        // Create our keyboard grid of keyboard line entries
        if (keyboardlineentry != null && keyboardGrid != null)
        {
            // Clear old grid
            foreach (Transform child in keyboardGrid.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            keyboardGrid.GetComponent<RectTransform>().ForceUpdateRectTransforms();

            // Add info about fixed keys
            GameObject keyentry = (GameObject)Instantiate(keyboardlineentry, keyboardGrid.transform);
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().text = "Slow Turning";
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize - 5;
            keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize - 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().characterLimit = 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().text = "SHIFT";
            keyentry.transform.Find("InputField").GetComponent<InputField>().interactable = false;
            keyentry.name = "SHIFT";

            keyentry = (GameObject)Instantiate(keyboardlineentry, keyboardGrid.transform);
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().text = "Switch Camera";
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize - 5;
            keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize - 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().characterLimit = 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().text = "SPACE";
            keyentry.transform.Find("InputField").GetComponent<InputField>().interactable = false;
            keyentry.name = "SPACE";

            keyentry = (GameObject)Instantiate(keyboardlineentry, keyboardGrid.transform);
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().text = "Toggle Names";
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize - 5;
            keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize - 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().characterLimit = 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().text = "!";
            keyentry.transform.Find("InputField").GetComponent<InputField>().interactable = false;
            keyentry.name = "EXCLAMATION";

            keyentry = (GameObject)Instantiate(keyboardlineentry, keyboardGrid.transform);
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().text = "Toggle Details";
            keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Label").GetComponent<Text>().fontSize - 5;
            keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize = keyentry.transform.Find("InputField/Text").GetComponent<Text>().fontSize - 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().characterLimit = 10;
            keyentry.transform.Find("InputField").GetComponent<InputField>().text = "@";
            keyentry.transform.Find("InputField").GetComponent<InputField>().interactable = false;
            keyentry.name = "AT";


            foreach (string currname in GLOBALS.KeyboardMap.Keys)
            {
                keyinfo currkey = GLOBALS.KeyboardMap[currname];
                keyentry = (GameObject)Instantiate(keyboardlineentry, keyboardGrid.transform);
                keyentry.transform.Find("InputField/Label").GetComponent<Text>().text = currkey.details;
                keyentry.transform.Find("InputField").GetComponent<InputField>().text = currkey.key;
                keyentry.name = currname;
            }
            keyboardGrid.GetComponent<RectTransform>().anchoredPosition3D = new Vector3(0, -500f, 0);

        }

    }

    public Dropdown joystick_sate_dropdown;

    public void PopulateJoystickGrid()
    {
        // Create our joystick grid of joystick line entries
        if (joysticklineentry != null && joystickGrid != null)
        {
            // Clear old grid
            foreach (Transform child in joystickGrid.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            joystickGrid.GetComponent<RectTransform>().ForceUpdateRectTransforms();

            // Add info about fixed keys
            GameObject joystickentry;

            foreach (string currname in GLOBALS.JoystickMap.Keys)
            {
                JoystickRawInfo curraxis = GLOBALS.JoystickMap[currname];
                joystickentry = (GameObject)Instantiate(joysticklineentry, joystickGrid.transform);
                joystickentry.GetComponent<JoystickInfo>().joydata = curraxis;
                joystickentry.GetComponent<JoystickInfo>().UpdateGUI();
                joystickentry.name = currname;
            }

            // Scroll to top
            joystickGrid.GetComponent<RectTransform>().anchoredPosition3D = new Vector3(0, -500f, 0);
        }

    }

    public void SetJoystickToDefaults()
    {
        GLOBALS.JoystickMap.Clear();
        GLOBALS.JoystickMap
        = new Dictionary<string, JoystickRawInfo>()
        {
            {"Jcontrolls_turn", new JoystickRawInfo("Turn", 0, false, 4) },
            {"Jcontrolls_move_lr", new JoystickRawInfo("Strafe", 0, false, 1) },
            {"Jcontrolls_move_ud", new JoystickRawInfo("Forward/Back", 0, false, 2) },
            {"Jcontrolls_right_y", new JoystickRawInfo("Right Stick Y", 0, false, 5) },
            {"Jcontrolls_dpad_lr", new JoystickRawInfo("D-Pad Left/Right", 0, false, 6) },
            {"Jcontrolls_dpad_ud", new JoystickRawInfo("D-Pad Up/Down", 0, false, 7, 0, 0, 1f, -1f) },
            {"Jcontrolls_A", new JoystickRawInfo("Button A", 0, true, 0, 0 ) },
            {"Jcontrolls_B", new JoystickRawInfo("Button B", 0, true, 0, 1 ) },
            {"Jcontrolls_X", new JoystickRawInfo("Button X", 0, true, 0, 2) },
            {"Jcontrolls_Y", new JoystickRawInfo("Button Y", 0, true, 0, 3) },
            {"Jcontrolls_LB", new JoystickRawInfo("Slow Turn L", 0, true, 0, 4 ) },
            {"Jcontrolls_RB", new JoystickRawInfo("Slow Turn R", 0, true, 0, 5) },
            {"Jcontrolls_LTR", new JoystickRawInfo("Left Trigger", 0, false, 3, 0, 0, 1f, -1f  ) },
            {"Jcontrolls_RTR", new JoystickRawInfo("Right Trigger", 0, false, 3 ) },
            {"Jcontrolls_stop", new JoystickRawInfo("Stop Timer", 0, true, 0, 7 ) },
            {"Jcontrolls_restart", new JoystickRawInfo("Start Timer", 0, true, 0, 6) },
            {"Jcontrolls_camera", new JoystickRawInfo("Switch Camera", 0, true, 0, 8) },
            {"Jcontrolls_reset", new JoystickRawInfo("Restart Position", 0, true, 0, 9) }
        };

    }

    public void OnRobotSkinsUp()
    {
        curr_robotskins += 1;
        OnMenuChanged();
    }

    public void OnRobotSkinsDown()
    {
        curr_robotskins -= 1;
        OnMenuChanged();
    }

    public GameObject savestring_go;
    public TMPro.TMP_InputField save_name;
    public Text save_name_status;
    public bool savestate_joystick = true;

    public bool SaveStateValidator()
    {
        string name = save_name.text;

        // Make sure something is inside
        if (name.Length < 1)
        {
            save_name_status.text = "Enter name...";
            return false;
        }

        // Change name to capital
        name = name.ToUpper();

        // Make sure it isn't our Default state
        if (name == "DEFAULT")
        {
            save_name_status.text = "Can't override default state!";
            return false;
        }


        if (GLOBALS.all_robot_capital_names.Contains(name))
        {
            save_name_status.text = "Set " + name + " Robot default state.";
        }
        else
        {
            save_name_status.text = "State name Ok.";
        }
 
       
        return true;
    }

    public void OnSaveStateButton()
    {
        SaveStateValidator();
    }

    // If joystick = true, it affects joystick state
    // else affects keyboard state
    public void OnSaveButton(bool joystick)
    {
        savestring_go.SetActive(true);
        savestate_joystick = joystick;
    }

    public void OnDeleteState(bool joystick)
    {
        Dropdown state_dropdown = (joystick) ? joystick_state_dropdown : keyboard_state_dropdown;
        Dictionary<string,string> states = (joystick) ? GLOBALS.joystick_states : GLOBALS.keyboard_states;

        // Get name of the state
        string name = state_dropdown.options[state_dropdown.value].text;

        // Can't delete default state
        if (name == "Default")
        {
            return;
        }

        // Delete appropriate item
        states.Remove(name);

        // Adjust dropdown selection
        state_dropdown.value = 1;

        // Update list
        if( joystick)
        {
            UpdateJoystickList();
            SaveJoystickStates();
        }
        else
        {
            UpdateKeyboardList();
            SaveKeyboardStates();
        }

        PlayerPrefs.Save();
    }


    public void OnSaveState()
    {
        // Make sure it's a valid name
        if (!SaveStateValidator()) { return; }

        // Ok time to save
        if (savestate_joystick) { SaveJoystickState(save_name.text); }
        else { SaveKeyboardState(save_name.text); }

        // Hide screen
        savestring_go.SetActive(false);
    }

    public void OnLoadState(bool joystick)
    {
        if (joystick)
        {
            if(joystick_state_dropdown.value >= joystick_state_dropdown.options.Count)
            {
                joystick_state_dropdown.value = joystick_state_dropdown.options.Count - 1;
            }
            // Get name of state
            string name = joystick_state_dropdown.options[joystick_state_dropdown.value].text;

            if (name == "Default")
            {
                ResetJoystickToDefaults();
                return;
            }

            // Load appropriate state
            LoadJoystickState(name);
        }
        else
        {
            if (keyboard_state_dropdown.value >= keyboard_state_dropdown.options.Count)
            {
                keyboard_state_dropdown.value = keyboard_state_dropdown.options.Count - 1;
            }

            // Get name of state
            string name = keyboard_state_dropdown.options[keyboard_state_dropdown.value].text;

            if (name == "Default")
            {
                ResetKeyboardToDefaults();
                return;
            }

            // Load appropriate state
            LoadKeyboardState(name);
        }
    }


    public void SaveJoystickState(string name)
    {
        string out_string = "";
        name = name.ToUpper();

        // Joystick assignmentLoad joystick assignments into Dictionary
        foreach (string currkey in GLOBALS.JoystickMap.Keys)
        {
            out_string += "||" + currkey + "||" + GLOBALS.JoystickMap[currkey].GetString();
        }

        GLOBALS.joystick_states[name] = out_string;

        // Save to PlayerPrefs
        SaveJoystickStates();

        UpdateJoystickList();

        PlayerPrefs.Save();

        // Now select our state
        joystick_state_dropdown.value = GetOptionsIndex(name, true);
       
    }


    public int GetOptionsIndex(string value, bool joystick)
    {
        Dropdown dropdown = (joystick) ? joystick_state_dropdown : keyboard_state_dropdown;

        for( int i =0; i < dropdown.options.Count; i++)
        {
            if( dropdown.options[i].text == value)
            {
                return i;
            }
        }

        return 0;
    }

    public void SaveKeyboardState(string name)
    {
        string out_string = "";

        name = name.ToUpper();

        // Keyboard assignment Load keyboard assignments into Dictionary
        foreach (string currkey in GLOBALS.KeyboardMap.Keys)
        {
            out_string += "||" + currkey + "||" + GLOBALS.KeyboardMap[currkey].GetString();
        }

        GLOBALS.keyboard_states[name] = out_string;

        // Save to PlayerPrefs
        SaveKeyboardStates();

        UpdateKeyboardList();

        PlayerPrefs.Save();

        // Now select our state
        keyboard_state_dropdown.value = GetOptionsIndex(name, false);        
    }


    public static void SaveJoystickStates()
    {
        string out_string = "";

        // Create combined string of all the states
        foreach( string curr_key in GLOBALS.joystick_states.Keys)
        {
            out_string += "~~" + curr_key+ "~~" + GLOBALS.joystick_states[curr_key];
        }
       
        // Save it to our preferences
        PlayerPrefs.SetString("JOY_STATES", out_string);
    }

    public static void SaveKeyboardStates()
    {
        string out_string = "";

        // Create combined string of all the states
        foreach (string curr_key in GLOBALS.keyboard_states.Keys)
        {
            out_string += "~~" + curr_key + "~~" + GLOBALS.keyboard_states[curr_key];
        }

        // Save it to our preferences
        PlayerPrefs.SetString("KEY_STATES", out_string);
    }

    public bool LoadJoystickState(string key)
    {
        key = key.ToUpper();

        if( !GLOBALS.joystick_states.ContainsKey(key))
        { return false; }

        string[] seperators = { "||" };

        // Decompose the state
        string[] value_pair = GLOBALS.joystick_states[key].Split(seperators, StringSplitOptions.None);

        if (value_pair.Length < 3) { return false;}

        for( int i = 1; i < value_pair.Length-1; i+=2)
        {
            if (GLOBALS.JoystickMap.ContainsKey(value_pair[i]))
            {
                GLOBALS.JoystickMap[value_pair[i]].FromString(value_pair[i + 1]);
            }
            else
            {
                Debug.Log("Joystick Map did not have key " + value_pair[i]);
            }
        }

        // Update grid
        PopulateJoystickGrid();

        return true;
    }

    public bool LoadKeyboardState(string key)
    {
        key = key.ToUpper();

        if (!GLOBALS.keyboard_states.ContainsKey(key))
        { return false; }

        string[] seperators = { "||" };

        // Decompose the state
        string[] value_pair = GLOBALS.keyboard_states[key].Split(seperators, StringSplitOptions.None);

        if (value_pair.Length < 3) { return false; }

        for (int i = 1; i < value_pair.Length - 1; i += 2)
        {
            if (GLOBALS.KeyboardMap.ContainsKey(value_pair[i]))
            {
                GLOBALS.KeyboardMap[value_pair[i]].FromString(value_pair[i + 1]);
            }
            else
            {
                Debug.Log("Keyboard Map did not have key " + value_pair[i]);
            }
        }

        // Update grid
        PopulateKeyboardGrid();

        return true;
    }

    // Loads al lthe joystick states from the preferences
    public bool LoadJoystickStates()
    {
        // Get all the joystick states
        if (!PlayerPrefs.HasKey("JOY_STATES"))
        { return false; }

        string[] seperators = { "~~" };

        // Split the combined string along the seperator
        string[] all_states = PlayerPrefs.GetString("JOY_STATES").Split(seperators, StringSplitOptions.None);

        if( all_states.Length < 2) { return false; }

        GLOBALS.joystick_states.Clear();

        // Load each one in to our joystick states dictionary
        for ( int i = 1; i < all_states.Length-1; i+=2)
        {
            GLOBALS.joystick_states[all_states[i]] = all_states[i + 1];
        }

        // Update Joystick List
        UpdateJoystickList();

        return true;
    }

    public bool LoadKeyboardStates()
    {
        // Get all the joystick states
        if (!PlayerPrefs.HasKey("KEY_STATES"))
        { return false; }

        string[] seperators = { "~~" };

        // Split the combined string along the seperator
        string[] all_states = PlayerPrefs.GetString("KEY_STATES").Split(seperators, StringSplitOptions.None);

        if (all_states.Length < 2) { return false; }

        GLOBALS.keyboard_states.Clear();

        // Load each one in to our joystick states dictionary
        for (int i = 1; i < all_states.Length - 1; i += 2)
        {
            GLOBALS.keyboard_states[all_states[i]] = all_states[i + 1];
        }

        // Update Joystick List
        UpdateKeyboardList();

        return true;
    }

    public Dropdown joystick_state_dropdown;
    public Dropdown keyboard_state_dropdown;

    // Update joystick list will update the list based on current joystick_states dictionary
    public void UpdateJoystickList()
    {
        joystick_state_dropdown.ClearOptions();
        List<Dropdown.OptionData> new_options = new List<Dropdown.OptionData>();

        new_options.Add(new Dropdown.OptionData("Default"));

        foreach ( string curr_key in GLOBALS.joystick_states.Keys)
        {
            new_options.Add(new Dropdown.OptionData(curr_key));
        }
        joystick_state_dropdown.AddOptions(new_options);

        // adjust selection to be in bound
        if(joystick_state_dropdown.value >= joystick_state_dropdown.options.Count)
        {
            joystick_state_dropdown.value = joystick_state_dropdown.options.Count - 1;
        }

        joystick_state_dropdown.RefreshShownValue();
    }

    public void UpdateKeyboardList()
    {
        keyboard_state_dropdown.ClearOptions();
        List<Dropdown.OptionData> new_options = new List<Dropdown.OptionData>();

        new_options.Add(new Dropdown.OptionData("Default"));

        foreach (string curr_key in GLOBALS.keyboard_states.Keys)
        {
            new_options.Add(new Dropdown.OptionData(curr_key));
        }
        keyboard_state_dropdown.AddOptions(new_options);

        // adjust selection to be in bound
        if (keyboard_state_dropdown.value >= keyboard_state_dropdown.options.Count)
        {
            keyboard_state_dropdown.value = keyboard_state_dropdown.options.Count - 1;
        }

        keyboard_state_dropdown.RefreshShownValue();
    }
}

