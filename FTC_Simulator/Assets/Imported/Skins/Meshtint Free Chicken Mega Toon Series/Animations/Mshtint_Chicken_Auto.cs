using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Mshtint_Chicken_Auto : MonoBehaviour
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
            // Reset actions
            theanimator.SetBool("Run", false);
            theanimator.SetBool("Eat", false);
            theanimator.SetBool("Turn Head", false);
            theanimator.SetBool("Walk", false);

            if (!waiting)
            {
                // Create a random wait event
                waiting = true;
                exit_time = curr_time + random.Next(1, 8) * 1000;

                theanimator.SetBool("Walk", true);
                return;
            }

            waiting = false;

            // Do a 2s action
            exit_time = curr_time +  2000;

            // Chose a random play action
            switch (random.Next(1,4))
            {
                case 1:
                    theanimator.SetBool("Run", true);
                    break;

                case 2:
                    theanimator.SetBool("Eat", true);
                    break;

                case 3:
                    theanimator.SetBool("Turn Head", true);
                    break;

                default:
                    theanimator.SetBool("Walk", true);
                    break;
            }
        }

    }
}
