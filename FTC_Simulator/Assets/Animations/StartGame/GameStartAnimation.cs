using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameStartAnimation : MonoBehaviour
{

    public Transform[] robot_red;
    public Transform[] robot_blue;

    public TMPro.TextMeshPro[] text_r;
    public TMPro.TextMeshPro[] text_b;

    public GameObject canvas_text;
    public Animator myanimator;

    private void Awake()
    {
        myanimator = GetComponent<Animator>();
    }

    // Called everytime this item is re-enabled, or by user
    public void OnEnable()
    {
        // Make sure we are active
        if( !gameObject.activeSelf) { return; }

        // Clean up any old data that may be there (should never really be needed)
        // Will at least run off all player names
        OnDisable();

        // Go thorugh all the players and instantiate them
        int i_red = 0;
        int i_blue = 0;

        foreach( ClientLow.Player currplayer in GLOBALS.topclient.players.Values)
        {
            // Delete spectators
            if( currplayer.isSpectator) { continue; }

            // Add the red robot
            if( currplayer.isRed && (i_red < robot_red.Length))
            {
                // First create it
                GameObject currbot = MyUtils.InstantiateRobot(
                                            currplayer.model, 
                                            // Vector3.zero,
                                            robot_red[i_red].transform.position, 
                                            robot_red[i_red].transform.rotation, 
                                            // Quaternion.Euler(0, UnityEngine.Random.Range(0, 360f),0), 
                                            currplayer.skins, 
                                            currplayer.robot_skins);

                // Turn off bandwidth helper updates
                foreach( BandwidthHelper currhelper in currbot.GetComponentsInChildren<BandwidthHelper>())
                {
                    currhelper.pauseUpdates = true;
                }

                // Set it's color
                RobotInterface3D robot = currbot.GetComponent<RobotInterface3D>();
                robot.SetName(currplayer.playerName); 
                robot.SetKinematic(true);
                robot.isSpectator = true;
                robot.SetColorFromPosition(currplayer.position);

                // Set the parent
                currbot.transform.SetParent(robot_red[i_red].parent);

                // Set the name
                text_r[i_red].gameObject.SetActive(true);
                text_r[i_red].text = currplayer.playerName;
               
                i_red++;
            }

            // Add the blue robot
            if (!currplayer.isRed && (i_blue < robot_blue.Length))
            {
                // First create it
                GameObject currbot = MyUtils.InstantiateRobot(
                                            currplayer.model,
                                            robot_blue[i_blue].transform.position,
                                            robot_blue[i_blue].transform.rotation,
                                            currplayer.skins,
                                            currplayer.robot_skins);

                // Turn off bandwidth helper updates
                foreach (BandwidthHelper currhelper in currbot.GetComponentsInChildren<BandwidthHelper>())
                {
                    currhelper.pauseUpdates = true;
                }

                // Set it's color
                RobotInterface3D robot = currbot.GetComponent<RobotInterface3D>();
                robot.SetName(currplayer.playerName);
                robot.SetKinematic(true);
                robot.isSpectator = true;
                robot.SetColorFromPosition(currplayer.position);

                // Set the parent
                currbot.transform.SetParent(robot_blue[i_blue].parent);

                // Set the name
                text_b[i_blue].gameObject.SetActive(true);
                text_b[i_blue].text = currplayer.playerName;
                
                i_blue++;
            }
        }

        if (canvas_text)
        {
            canvas_text.SetActive(true);
        }

        
    }


    private void OnDisable()
    {
        RobotInterface3D[] allbots = GetComponentsInChildren<RobotInterface3D>();

        // Delete all bots that were instantiated
        foreach( RobotInterface3D currbot in allbots)
        {
            Destroy(currbot.gameObject);
        }

        // Turn off all name tags
        foreach(TMPro.TextMeshPro currtext in text_r)
        {
            currtext.gameObject.SetActive(false);
        }
        foreach (TMPro.TextMeshPro currtext in text_b)
        {
            currtext.gameObject.SetActive(false);
        }

        if (canvas_text)
        {
            canvas_text.SetActive(false);
        }
    }

    // TurnOff immediately disables this
    public void TurnOff()
    {
        gameObject.SetActive(false);
    }

    // Plays the end animation and closes the object
    public void EndAnimation()
    {
        // Turn off display if no animator
        if( !myanimator ||( myanimator.enabled == false) )
        {
            TurnOff();
            return;
        }

        // Play close if not already
        if( myanimator.GetCurrentAnimatorStateInfo(0).IsName("Close"))
        {
            // Already playing, thus don't play again
            return;
        }

        // Start the close animation
        myanimator.Play("Close");
    }

    public void ShowNoAnimation(bool state)
    {
        if( !state)
        { 
            gameObject.SetActive(false);
            return;
        }

        // If already enabled, exit
        if (gameObject.activeSelf)
        {  return; }

        gameObject.SetActive(true);

        // Turn off the animations
        myanimator.Play("OpenNoAnimation");
    
    }

}
