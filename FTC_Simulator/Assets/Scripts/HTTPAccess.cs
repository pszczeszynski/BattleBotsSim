using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text.RegularExpressions;
using System.Text;

using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

public class ServerInfo
{
    public string IP = "0.0.0.0";
    public int PORT = 1446;
    public string GAME = "Unknown";
    public float PING = 9.999f; // Ping in seconds
    public int PLAYERS = 0; // Number of online players
    public int MAXPLAYERS = 0; // Maximum number of players
    public string VERSION = "0.0";
    public string COMMENT = "";
    public string PASSWORD = "0";
    public GameObject gui_line = null;
    public Ping pingobject = null;
};



public class HTTPAccess : MonoBehaviour
{

    public GameObject gui_listofservers; // Contains gridlayoutgroup to list all elements
    public GameObject gui_serverlineprefab;  // Prefab of a single server line
    public GameObject gui_ip_target_text;
    public GameObject error_msg;

    private List<ServerInfo> serverlist = new List<ServerInfo>();

    // Use this for initialization
    void Start()
    {
        /*  System.Random rnd = new System.Random();

          // Debug code to initialzie list
          for ( int i=1; i < 15; i++)
          {

              ServerInfo newserver = new ServerInfo()
              {
                  IP = "127.110.256." + i.ToString(),
                  PING = (float)rnd.Next(1, 500) / 1000f
              };

              serverlist.Add(newserver);
          }
          */
    }

    public void ClearGUIServerList()
    {
        // Clear any existing entries in GUI
        foreach (Transform currchild in gui_listofservers.transform)
        {
            GameObject.Destroy(currchild.gameObject);
        }
    }

    // When the registered Servers screen is enabled
    public void RefreshGUIServerList()
    {
        // Make sure listofservers and serverlineprefab are set
        if (gui_listofservers == null ||
            gui_serverlineprefab == null)
        { return; }

        // Clear any existing entries in GUI
        ClearGUIServerList();

        // Sort the array
        serverlist.Sort((x, y) => x.PING.CompareTo(y.PING));

        // Populate new ones
        foreach (ServerInfo currserver in serverlist)
        {
            GameObject gui_currserver = (GameObject)Instantiate(gui_serverlineprefab, gui_listofservers.transform);
            currserver.gui_line = gui_currserver;

            gui_currserver.transform.Find("IP").GetComponent<Text>().text = currserver.IP;
            gui_currserver.transform.Find("GAME").GetComponent<Text>().text = currserver.GAME;
            gui_currserver.transform.Find("PING").GetComponent<Text>().text = (currserver.PING * 1000f).ToString("G0") + "ms";
            gui_currserver.transform.Find("PLAYERS").GetComponent<Text>().text = currserver.PLAYERS + "/" + currserver.MAXPLAYERS;
            gui_currserver.transform.Find("VERSION").GetComponent<Text>().text = currserver.VERSION;
            gui_currserver.transform.Find("COMMENT").GetComponent<Text>().text = currserver.COMMENT;

            if( currserver.PASSWORD == "1")
            {
                gui_currserver.transform.Find("PASSWORD").gameObject.SetActive(true);
            }
            else
            {
                gui_currserver.transform.Find("PASSWORD").gameObject.SetActive(false);
            }

            // Add callback
            gui_currserver.GetComponent<Button>().onClick.AddListener(delegate { OnServerClick(currserver.IP, currserver.PORT); });
        }

    }

    // When someone clicks on a server
    public void OnServerClick(string server_ip, int server_port)
    {
        // Assign the IP and Port
        gui_ip_target_text.transform.Find("IPInput").GetComponent<InputField>().text = server_ip;
        gui_ip_target_text.transform.Find("PORTInput").GetComponent<InputField>().text = server_port.ToString();
    }

    // Starts the process of retrieving srever list
    public void StartServerRefresh()
    {
        StopAllCoroutines();
        StartCoroutine(RetrieveHTTPServerList());
    }
    
    IEnumerator RetrieveHTTPServerList()
    {
        // Clear all existing servers in the GUI
        ClearGUIServerList();

        // Clear the list of servers internally
        serverlist.Clear();

        string[] returnedmsg = null;

        // Start the web request
        using (UnityWebRequest www = UnityWebRequest.Get(GLOBALS.HTTP_ADDRESS))
        {
            yield return www.SendWebRequest();

            if (www.isNetworkError || www.isHttpError)
            {
                GameObject gui_error = (GameObject)Instantiate(error_msg, gui_listofservers.transform);
                gui_error.GetComponent<Text>().text = "Error! Unable to connect to master server!";
                yield break;
            }

            // Add servers to our list
            returnedmsg = www.downloadHandler.text.Split('\n');
        }

        bool begin = false;
        Regex match_begin = new Regex(@"^\s*BEGIN\s*\n"); // Look for begin
        Regex match_end = new Regex(@"^\s*END\s*\n");     // Look for end
        Regex match_ip_and_port = new Regex(@"^\s*IP=(.+)\|PORT=(\d+)\s*");     // Look for IP and PORT

        for (int i = 0; i < returnedmsg.Length; i++)
        {
            // If we found end, then quit
            Match check_end = match_end.Match(returnedmsg[i]);
            if (check_end.Success)
            { break; }

            // Search for BEGIN
            if (!begin)

            {
                Match check_begin = match_begin.Match(returnedmsg[i]);
                if (check_begin.Success)
                {
                    begin = true;
                    continue;
                }
            }

            // Extract IP and PORT
            Match find_ip = match_ip_and_port.Match(returnedmsg[i]);

            if (find_ip.Success)
            {
                // Create the new server info and add it to our list
                ServerInfo newserver = new ServerInfo();

                newserver.IP = find_ip.Groups[1].Value;
                if (!int.TryParse(find_ip.Groups[2].Value, out newserver.PORT))
                {
                    Debug.Log("Failed to parse PORT # from " + returnedmsg[i]);
                }

                serverlist.Add(newserver);
            }

        }

        if (serverlist.Count == 0)
        {
            GameObject gui_error = (GameObject)Instantiate(error_msg, gui_listofservers.transform);
            gui_error.GetComponent<Text>().text = "No servers found.";
            yield break;
        }

        // Update the GUI
        RefreshGUIServerList();

        // Ping all the servers
        for (int i = 0; i < serverlist.Count; i++)
        {
            serverlist[i].pingobject = new Ping(serverlist[i].IP);
        }

        yield return 0;

        // Wait for all the servers to come back
        bool waiting_on_servers = true;
        bool servers_updated = false;
        long start_ping_time = MyUtils.GetTimeMillis();


        // Timeout after 1.5 seconds (Who wants to play on a 1.5s ping-time server?? This does include overhead time though)
        while (waiting_on_servers && (MyUtils.GetTimeMillis() - start_ping_time) < 1500)
        {
            // Reset waiting on servers
            waiting_on_servers = false;

            // Make sure there are some servers
            if (serverlist.Count == 0) { yield break; };

            // Go through the servers and check if they are done
            // Create a ping timeout

            for (int i = 0; i < serverlist.Count; i++)
            {
                // If this server is complete, move on
                if (serverlist[i].pingobject == null)
                { continue; }

                // If we are waiting on it, then mark that
                if (!serverlist[i].pingobject.isDone)
                {
                    waiting_on_servers = true;
                    continue;
                }

                // Otherwise get the value (assuming ping.time is in milliseconds
                serverlist[i].PING = ((float)serverlist[i].pingobject.time) / 1000f;
                serverlist[i].pingobject.DestroyPing();
                serverlist[i].pingobject = null;
                servers_updated = true;
            }

            // If servers were updated, then refresh the list
            if (servers_updated)
            {
                RefreshGUIServerList();
            }

            // Go through the servers with a real ping and get their info

            // Check again next frame
            yield return 0;
        }

        // Clear out ping objects from remaining items
        for (int i = 0; i < serverlist.Count; i++)
        {
            // If this server is complete, move on
            if (serverlist[i].pingobject != null)
            {
                serverlist[i].pingobject.DestroyPing();
                serverlist[i].pingobject = null;
            }
        }


        // Refresh one more time
        RefreshGUIServerList();

        // Go through each server and try to retrieve server info
        //IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, GLOBALS.UDP_PORT);
        //UdpClient m_udpClient = new UdpClient(remoteEP);
        //IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, GLOBALS.UDP_PORT);
        UdpClient m_udpClient = new UdpClient();

        // Send out requests
        for (int i = 0; i < serverlist.Count; i++)
        {
            // If this server's ping is bad, move on
            //if (serverlist[i].PING < 0 || serverlist[i].PING > 9)
            //{ continue; }

            // Send a serverinfo request
            // <SERVERINFO>|<SERVERIP>%<SERVERPORT>|
            string msgstring = GLOBALS.GETSERVERINFO + GLOBALS.SEPARATOR1 + serverlist[i].IP + GLOBALS.SEPARATOR2 + serverlist[i].PORT + GLOBALS.SEPARATOR1;
            byte[] messageBytes = Encoding.UTF8.GetBytes(msgstring);

            // Send the data
            IPEndPoint destination = new IPEndPoint(IPAddress.Parse(serverlist[i].IP), serverlist[i].PORT);

            try
            {
                m_udpClient.Send(messageBytes, messageBytes.Length, destination);
            }
            catch (Exception t)
            {
                // Drop/Ignore the error
            }

            // Update GUI
            yield return 0;
        }

        // Now wait to receive requests

        // Timeout after 1.5s
        start_ping_time = MyUtils.GetTimeMillis();
        waiting_on_servers = true;

        while (waiting_on_servers && (MyUtils.GetTimeMillis() - start_ping_time) < 1500)
        {
            yield return 0;

            // Quit if our udpClient died
            if (m_udpClient == null) { yield break; }

            // If no data, continue
            if (m_udpClient.Available < 10)
            { continue; }

            byte[] receivedBytes;
            IPEndPoint incomingEP;
            incomingEP = new IPEndPoint(IPAddress.Any, 0);

            try
            {
                receivedBytes = m_udpClient.Receive(ref incomingEP);

                // Extract the server info
                ProcessReceivedServerInfo(receivedBytes);
            }
            catch (Exception e)
            {
                // Drop/Ignore errors
            }

            RefreshGUIServerList();
        }

        RefreshGUIServerList();
    }

    // Take the UDP message from server and extract data, add it to our client list
    private void ProcessReceivedServerInfo(byte[] receivedBytes)
    {
        // Firstly extract password, etc..
        List<byte[]> extracted_data = new List<byte[]>();
        MyUtils.ExtractMessageHeader(receivedBytes, extracted_data);

        // Check passcode
        if (!Encoding.UTF8.GetString(extracted_data[0]).Equals(GLOBALS.GETSERVERINFO) || extracted_data.Count < 3)
        {
            return;
        }

        // Get all the server information
        string serverData = Encoding.UTF8.GetString(extracted_data[1]);
        string serverInfo = Encoding.UTF8.GetString(extracted_data[2]);

        string[] serverDataList = serverData.Split(GLOBALS.SEPARATOR2);
        string[] serverInfoList = serverInfo.Split(GLOBALS.SEPARATOR2);

        // Sanity check that we got IP and Port from our old data
        if (serverInfoList.Length < 2) { return; }

        // Divide up the serverDataList
        Dictionary<string, string> serverhash = new Dictionary<string, string>();

        for (int i = 0; i < serverDataList.Length - 1; i += 2)
        {
            serverhash[serverDataList[i]] = serverDataList[i + 1];
        }

        // Get our server info
        string IP = serverInfoList[0];
        int PORT = 0;

        if (!int.TryParse(serverInfoList[1], out PORT)) { return; }

        // Now search for our server and update the data
        for (int i = 0; i < serverlist.Count; i++)
        {
            // If don't match, go to next one
            if (serverlist[i].IP != IP ||
                serverlist[i].PORT != PORT)
            { continue; }

            // Go through the server hash and populate it
            foreach (string currkey in serverhash.Keys)
            {
                try
                {
                    switch (currkey)
                    {
                        case "GAME":
                            serverlist[i].GAME = serverhash["GAME"];
                            break;
                        case "PLAYERS":
                            serverlist[i].PLAYERS = int.Parse(serverhash["PLAYERS"]);
                            break;
                        case "MAXPLAYERS":
                            serverlist[i].MAXPLAYERS = int.Parse(serverhash["MAXPLAYERS"]);
                            break;
                        case "VERSION":
                            serverlist[i].VERSION = serverhash["VERSION"];
                            break;
                        case "COMMENT":
                            serverlist[i].COMMENT = serverhash["COMMENT"];
                            break;
                        case "PASSWORD":
                            serverlist[i].PASSWORD = serverhash["PASSWORD"];
                            break;
                        default:
                            break;
                    }
                }
                catch (Exception e)
                {
                    // Drop errors
                }
            } // Foreach serverhash keys

            break;
        } // for serverlist
    } // ProcessReceivedServerInfo

}
