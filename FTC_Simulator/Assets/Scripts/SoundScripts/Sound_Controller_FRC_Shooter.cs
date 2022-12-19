using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sound_Controller_FRC_Shooter : MonoBehaviour
{
    public AudioManager mySoundPlayer;

    ///////SHOOTER STATEMACHINE VARIABLES///////
    shooterStates myShooterState = shooterStates.off;
    private bool shooterStateFinished = false;

    enum shooterStates {
        off,
        hovering
    }
    ///////////////////////////////////////

    private void Start() {
        //get a link to our sound player
        mySoundPlayer = GetComponent<AudioManager>();
    }

    private void Update() {
        updateShooterSoundStateMachine();
    }
    
    public void revup() {
        nextShooterState((int)shooterStates.hovering);
    }
    public void revdown() {
        nextShooterState((int) shooterStates.off);
    }

    //Goes to the next shooter sound state
    private void nextShooterState() {
        nextShooterState((int) myShooterState + 1);
    }
    private void nextShooterState(int number) {
        myShooterState = (shooterStates) number;
        shooterStateFinished = true;
    }

    void updateShooterSoundStateMachine()
    {
        if (myShooterState == shooterStates.off)
        {
            if (shooterStateFinished)
            {
                shooterStateFinished = false;
                mySoundPlayer.Stop("shooter_hover", 0);
            }
        }
        
        if (myShooterState == shooterStates.hovering)
        {
            if (shooterStateFinished)
            {
                shooterStateFinished = false;
                //crossfade between hover and revup
                mySoundPlayer.Play("shooter_hover", 0);
            
            }     
        }
    }
  
    
}
