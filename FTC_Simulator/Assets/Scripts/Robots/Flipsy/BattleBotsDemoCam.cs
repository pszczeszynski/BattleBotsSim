using System.Collections.Generic;
using UnityEngine;

public class BattleBotsDemoCam : MonoBehaviour
{
    // A list of all the robots with a RobotInterface3d script component.
    private List<GameObject> robots = new List<GameObject>();

    private void Start()
    {
        // Start the search for robots loop.
        InvokeRepeating(nameof(SearchForRobots), 0f, 2f);
    }

    private void SearchForRobots()
    {
        // Find all the robots in the scene and cache them in the robots list.
        robots.Clear();
        var robotObjects = GameObject.FindObjectsOfType<RobotInterface3D>();
        foreach (var robot in robotObjects)
        {
            robots.Add(robot.gameObject);
        }

        if (robotObjects.Length > 0)
        {
            GetComponent<UnityStandardAssets.Cameras.LookatTarget>().SetTarget(robots[0].transform.Find("Body"));
        }

    }


    private void Update()
    {
        // Check if the "r" key is pressed.
        if (Input.GetKeyDown(KeyCode.R))
        {
            MoveRobotsToTargets();
        }
    }

    private void MoveRobotsToTargets()
    {
        // Move each robot in the robots list to the corresponding target location.
        for (int i = 0; i < robots.Count; i++)
        {
            robots[i].GetComponent<RobotInterface3D>().ResetPosition();
        }
    }
}