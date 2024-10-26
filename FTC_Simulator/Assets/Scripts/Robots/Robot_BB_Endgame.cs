using UnityEngine;

public class Robot_BB_Endgame : RobotInterface3D
{
    private TankDrive _tankDrive;
    public float DOWN_FORCE;
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
        float movement_amount = (Input.GetKey(KeyCode.W) ? 1.0f : 0) - (Input.GetKey(KeyCode.S) ? 1.0f : 0);
        // turn with j and l
        float turn_amount = (Input.GetKey(KeyCode.A) ? 1.0f : 0) - (Input.GetKey(KeyCode.D) ? 1.0f : 0);

        movement_amount += (float) Robot_BB_Orbitron.opponent_movement_amount;
        turn_amount += (float) Robot_BB_Orbitron.opponent_turn_amount;

        // movement_amount -= Input.GetAxis("J1Axis5");
        // turn_amount -= Input.GetAxis("J1Axis1");

        // apply input
        _tankDrive.Drive(movement_amount, turn_amount);

        ApplyDownForce();
    }

    public Transform robot_body;

    // adds a force to the robot body to keep it on the ground
    void ApplyDownForce()
    {
        // apply down force
        robot_body.GetComponent<Rigidbody>().AddForce(Vector3.down * DOWN_FORCE);
    }

}