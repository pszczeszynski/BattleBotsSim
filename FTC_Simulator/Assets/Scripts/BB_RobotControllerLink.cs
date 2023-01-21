using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class BB_RobotControllerLink
{
    private UdpClient robotControllerClient;
    private IPEndPoint robotControllerEP;
    bool hasReceivedMessage = false;
    private RobotControllerMessage lastRobotControllerMessage;

    public BB_RobotControllerLink(string robotControllerIP = "127.0.0.1",
                                  int robotControllerPort = 11115)
    {
        robotControllerEP = new IPEndPoint(IPAddress.Parse(robotControllerIP), robotControllerPort);
        robotControllerClient = new UdpClient();

        // somehow this prevents a weird exception from being thrown
        uint IOC_IN = 0x80000000;
        uint IOC_VENDOR = 0x18000000;
        uint SIO_UDP_CONNRESET = IOC_IN | IOC_VENDOR | 12;
        robotControllerClient.Client.IOControl((int)SIO_UDP_CONNRESET, new byte[] { Convert.ToByte(false) }, null);
        ///////////////

        robotControllerClient.Connect(robotControllerEP);
    }

    ~BB_RobotControllerLink()
    {
        robotControllerClient.Close();
    }

    // Receive the latest message
    public RobotControllerMessage? Receive()
    {
        // THE NUMBER OF LOOPS CANNOT BE INIFINITE since the robot controller only sends one message per frame
        while (robotControllerClient.Available > 0)
        {
            byte[] data = robotControllerClient.Receive(ref robotControllerEP);
            string message = System.Text.Encoding.ASCII.GetString(data);
            lastRobotControllerMessage = JsonUtility.FromJson<RobotControllerMessage>(message);
            hasReceivedMessage = true;
        }

        if (!hasReceivedMessage) {
            return null;
        }

        return lastRobotControllerMessage;
    }

    public void SendMessage(string message)
    {
        byte[] messageBytes = Encoding.UTF8.GetBytes(message);
        robotControllerClient.Send(messageBytes, messageBytes.Length);
    }
}

[System.Serializable]
public struct RobotControllerMessage
{
    public double drive_amount;
    public double turn_amount;
    public List<Vector3> point_cloud;
}
