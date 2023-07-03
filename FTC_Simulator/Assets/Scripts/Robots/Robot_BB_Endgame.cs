using UnityEngine;

public class Robot_BB_Endgame : RobotInterface3D
{
    private TankDrive _tankDrive;
    public void Awake()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }
        // find the tank drive script
        _tankDrive = GetComponent<TankDrive>();
    }


    public void Update()
    {
        // don't execute this on the client
        if (GLOBALS.CLIENT_MODE) { return; }

        // movement with w and s
        float movement_amount = (Input.GetKey(KeyCode.UpArrow) ? 1.0f : 0) - (Input.GetKey(KeyCode.DownArrow) ? 1.0f : 0);
        // turn with j and l
        float turn_amount = (Input.GetKey(KeyCode.LeftArrow) ? 1.0f : 0) - (Input.GetKey(KeyCode.RightArrow) ? 1.0f : 0);

        // apply input
        _tankDrive.Drive(movement_amount, turn_amount);
    }
}