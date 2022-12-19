using System.Net;
using System.Net.Sockets;
using System.Text;


public class BB_RobotControllerLink
{
    private string robotControllerIP;
    private int robotControllerPort;
    private UdpClient udpClient;
    private IPEndPoint robotControllerEP;

    public BB_RobotControllerLink(string robotControllerIP = "127.0.0.1",
                                  int robotControllerPort = 11115)
    {
        this.robotControllerIP = robotControllerIP;
        this.robotControllerPort = robotControllerPort;
        robotControllerEP = new IPEndPoint(
            IPAddress.Parse(robotControllerIP), robotControllerPort);
        udpClient = new UdpClient();
        udpClient.Connect(robotControllerEP);
    }

    public void SendMessage(string message)
    {
        byte[] messageBytes = Encoding.UTF8.GetBytes(message);
        udpClient.Send(messageBytes, messageBytes.Length);
    }

    // returns the latest requested control from the server
    public ControlInput GetLatestControlInput()
    {
        return new ControlInput { drive_amount = 1.0, turn_amount = 0.0 };
    }
}

public struct ControlInput
{
    public double drive_amount;
    public double turn_amount;
}



