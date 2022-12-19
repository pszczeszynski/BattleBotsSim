
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using UnityEngine;
// using UnityScript.Steps;

/// <summary>
/// During the 3.. 2.. 1.. countdown, we zoom and move on different part of the 
/// </summary>
public class CountdownCameraController : MonoBehaviour
{
    #region StateMachine
    /***************** STATE MACHINE *******************/
    enum CameraStates {
        DoingNothing = 0,
        OverheadLook,

        FieldAnimation1,

        // 1 second animations
        OrbitAroundPlayer,
        SlideByBumper,
        OverheadZoomout,
    }
    CameraStates currentCameraState = CameraStates.DoingNothing;
    /// Set everytime the state switches
    private float stateStartTime;
    /// Set to true on state transitions
    private bool stateFinished = true;

    private float stateElapsedTime => Time.time - stateStartTime;

    /// <summary>
    /// Call when you want to change the state,
    /// this initializes state variables
    /// </summary>
    private void SetState(int index) {
        currentCameraState = (CameraStates) index;
        stateFinished = true;
        stateStartTime = Time.time;
    }

    /// <summary>
    /// Goes to the next part of the sequence
    /// </summary>
    private void NextState() {
        SetState((int) currentCameraState + 1);
    }
    /****************************************************/
    #endregion

    #region PrivateVars
    public static CountdownCameraController instance;
    private Transform _camera => ClientLow.instance?.main_camera?.transform ?? SinglePlayer.main_camera?.transform;
    #endregion
    /**************** ACTUAL LOGIC VARIABLES *********************/
    #region LogicVariables
    #region LinearMotion
    /// We save the location the camera was in before the script began
    private Vector3 savedStartingLocation;
    private Quaternion savedStartingRotation;

    // a list of all the RobotInterface3D's 
    RobotInterface3D[] allRobots;
    // the index of the player we are zooming in on
    private int currentPlayerIndex = 0;
    RobotInterface3D currentRobotInView => allRobots?[currentPlayerIndex];
    Transform currentFocusedBody => currentRobotInView.rb_body.transform;
    /// The relative position of the camera to the player when the linear motion starts
    public Vector3 ORBIT_START_POS = new Vector3(1.4f, 0.2f, 0);
    public Vector3 ORBIT_DIRECTION = new Vector3(1, 0, 0);
    public Vector3 ORBIT_LOOK_OFFSET = new Vector3(0, 0.3f, 0);
    public float ORBIT_SPEED = 0.5f;
    #endregion

    #region BumperSlide
    public Vector3 BUMPERSLIDE_START_POS = new Vector3(-0.3f, 1, 1);
    public Vector3 BUMPERSLIDE_LOOK_OFFSET = new Vector3(0, 0.3f, 0);
    public Vector3 BUMPERSLIDE_DIRECTION = new Vector3(0, 0, 1);
    public float BUMPERSLIDE_SPEED = 0.5f;
    #endregion

    #region OverheadZoomout
    public Vector3 OVERHEAD_ZOOMOUT_START_POS = new Vector3(-0.3f, 1, 1);
    public Vector3 OVERHEAD_ZOOMOUT_LOOK_OFFSET = new Vector3(0, 0.3f, 0);
    public float OVERHEAD_ZOOMOUT_SPEED = 0.1f;
    #endregion
    #endregion
    /*************************************************************/

    private void Awake()
    {
        instance = this;
    }

    void LateUpdate() {
        //if (Input.GetKey(KeyCode.G) && !isRunning)
        //{
        //    StartCountdown();
        //}

        // Don't do anything to the camera
        if (currentCameraState == CameraStates.DoingNothing)
        {
            if (stateFinished)
            {
                stateFinished = false;
            }
        }

        if (currentCameraState == CameraStates.FieldAnimation1)
        {
            if (stateFinished)
            {
                stateFinished = false;
                _camera.gameObject.GetComponent<Animator>().Play("IntroAnimation1", -1, 0);
                //_camera.gameObject.GetComponent<Animator>().
            }
        }

        if (currentCameraState == CameraStates.OrbitAroundPlayer) 
        {
            if (stateFinished) 
            {
                stateFinished = false;
                _camera.position = currentFocusedBody.localToWorldMatrix.MultiplyPoint(ORBIT_START_POS);
            }
            _camera.Translate(Time.deltaTime * ORBIT_SPEED * ORBIT_DIRECTION);
            _camera.LookAt(currentFocusedBody.position + ORBIT_LOOK_OFFSET);

            if (stateElapsedTime >= 1)
            {
                NextState();
                currentPlayerIndex ++;
                currentPlayerIndex %= allRobots.Length;
            }
        }

        if (currentCameraState == CameraStates.SlideByBumper)
        {
            if (stateFinished)
            {
                stateFinished = false;
                _camera.position = currentFocusedBody.localToWorldMatrix.MultiplyPoint(BUMPERSLIDE_START_POS);
                _camera.LookAt(currentFocusedBody.position + BUMPERSLIDE_LOOK_OFFSET);
            }
            _camera.Translate(Time.deltaTime * BUMPERSLIDE_SPEED * BUMPERSLIDE_DIRECTION);

            if (stateElapsedTime >= 1)
            {
                NextState();
                currentPlayerIndex ++;
                currentPlayerIndex %= allRobots.Length;
            }
        }

        if (currentCameraState == CameraStates.OverheadZoomout)
        {
            if (stateFinished)
            {
                stateFinished = false;
                _camera.position = currentFocusedBody.localToWorldMatrix.MultiplyPoint(OVERHEAD_ZOOMOUT_START_POS);
                _camera.LookAt(currentFocusedBody.position + OVERHEAD_ZOOMOUT_LOOK_OFFSET);
            }
            _camera.Translate(Time.deltaTime * OVERHEAD_ZOOMOUT_SPEED * _camera.forward * -1);

            if (stateElapsedTime >= 1)
            {
                NextState();
                Stop();
            }
        }
    }

    public void Stop()
    {
        _camera.position = savedStartingLocation;
        _camera.rotation = savedStartingRotation;
        SetState((int)CameraStates.DoingNothing);
        GLOBALS.CAMERA_COUNTDOWN_CONTROL = false;
    }

    /// <summary>
    /// Instantiates a new gameobject and starts it's countdown
    /// </summary>
    public void StartCountdown_Internal()
    {
        allRobots = FindObjectsOfType<RobotInterface3D>();
        if (allRobots.Length == 0) allRobots = null;
        // Choose a random player to zoom in on
        currentPlayerIndex = UnityEngine.Random.Range(0, allRobots.Length);
        SetState((int)CameraStates.FieldAnimation1);

        savedStartingLocation = _camera.position;
        savedStartingRotation = _camera.rotation;
        // stops ClientLow from setting the camera position
        GLOBALS.CAMERA_COUNTDOWN_CONTROL = true;
    }

    #region StaticMethods
    public static void StartCountdown() => instance.StartCountdown_Internal();
    #endregion
}
