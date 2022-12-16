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


public class BB_robotControllerIP : MonoBehaviour
{
    private string robotControllerIP = "127.0.0.1";
    private int robotControllerPort = 11115;

    private UdpClient udpClient;

    private IPEndPoint robotControllerEP;

    void init() {
        robotControllerEP = new IPEndPoint(IPAddress.Parse(
            robotControllerIP), robotControllerPort); 
        udpClient.Connect(robotControllerEP);
    }

    void Update() {
        string messageString = "hello";
        byte[] messageBytes = Encoding.UTF8.GetBytes(messageString);
        
        udpClient.Send(messageBytes, messageBytes.Length);
    }
}




