using UnityEngine;

public class RobotControlUpdater : MonoBehaviour
{
    public RobotInterface3D robotController;

    void Update()
    {
        robotController.updateGamepadVars();
    }
}