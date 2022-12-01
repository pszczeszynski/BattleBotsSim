using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spider_Auto : MonoBehaviour
{
    private Animator theanimator;


    // Start is called before the first frame update
    void Start()
    {
        theanimator = GetComponent<Animator>();
    }

    // Update is called once per frame
    private float rand_wait;
    private System.Random random = new System.Random();
    long exit_time = 0;
    bool waiting = false;

    void Update()
    {
        // See if our current time expired
        long curr_time = MyUtils.GetTimeMillis();

        if(exit_time - curr_time < 0)
        {

            if (!waiting)
            {
                // Create a random wait event
                waiting = true;
                exit_time = curr_time + random.Next(1, 8) * 1000;

                theanimator.Play("idle");
                return;
            }

            waiting = false;

            // Do a 2s action
            exit_time = curr_time +  2000;

            // Chose a random play action
            switch (random.Next(1,4))
            {
                case 1:
                    theanimator.Play("jump");
                    break;

                case 2:
                    theanimator.Play("attack");
                    break;

                case 3:
                    theanimator.Play("die");
                    break;

                default:
                    theanimator.Play("walk");
                    exit_time = curr_time + 4000;
                    break;
            }
        }

    }
}
