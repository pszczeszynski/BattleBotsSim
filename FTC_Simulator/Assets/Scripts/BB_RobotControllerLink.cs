using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class BB_RobotControllerLink
{
    private string robotControllerIP;
    private int robotControllerPort;
    private UdpClient robotControllerClient;
    private IPEndPoint robotControllerEP;

    // The latest message received from the server. Empty after consuming to prevent re-parsing
    private string _latestResponseString = "";
    private bool _hasReceivedMessage = false;

    // Lock before using _latestResponseString or _hasReceivedMessages
    private object _latestResponseStringLockObj = new object();

    private RobotControllerMessage _latestResponse;
    private Thread receiveThread;

    public BB_RobotControllerLink(string robotControllerIP = "127.0.0.1",
                                  int robotControllerPort = 11115)
    {
        this.robotControllerIP = robotControllerIP;
        this.robotControllerPort = robotControllerPort;

        robotControllerEP = new IPEndPoint(IPAddress.Parse(robotControllerIP), robotControllerPort);
        robotControllerClient = new UdpClient();
        robotControllerClient.Connect(robotControllerEP);
    }

    // start the receiver thread on Start() of the Monobehavior
    void Start()
    {
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    // receives messages from the robot controller in the background
    private void ReceiveData()
    {
        while (true)
        {
            try
            {
                // Receive data from the server
                IPEndPoint endPoint = new IPEndPoint(IPAddress.Any, robotControllerPort);
                byte[] data = robotControllerClient.Receive(ref endPoint);

                // Convert the received data to a string and store it as the latest message
                string message = System.Text.Encoding.ASCII.GetString(data);
                lock (_latestResponseStringLockObj)
                {
                    _hasReceivedMessage = true;
                    _latestResponseString = message;
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Error receiving UDP data: " + e.ToString());
            }
        }
    }

    public void SendMessage(string message)
    {
        byte[] messageBytes = Encoding.UTF8.GetBytes(message);
        robotControllerClient.Send(messageBytes, messageBytes.Length);
    }

    // returns the response from the robot controller
    // it is in json form, so we must deserialize it
    public RobotControllerMessage? GetLatestControlInput()
    {
        lock (_latestResponseStringLockObj)
        {
            if (!_hasReceivedMessage) {
                return null;
            }

            // only deserialize if we haven't already
            if (_latestResponseString != "")
            {
                _latestResponse = JsonUtility.FromJson<RobotControllerMessage>(_latestResponseString);
                _latestResponseString = "";
            }
        }
        return _latestResponse;
    }
}

[System.Serializable]
public struct RobotControllerMessage
{
    public double drive_amount;
    public double turn_amount;
}
