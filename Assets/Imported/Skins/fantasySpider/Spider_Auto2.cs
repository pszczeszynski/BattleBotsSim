using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spider_Auto2 : MonoBehaviour
{
    private Animation theanimator;


    // Start is called before the first frame update
    void Start()
    {
        theanimator = GetComponent<Animation>();
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
            switch (random.Next(1,12))
            {
                case 1:
                    theanimator.Play("run");
                    break;

                case 2:
                    theanimator.Play("attack1");
                    break;

                case 3:
                    theanimator.Play("attack2");
                    break;

                case 4:
                    theanimator.Play("taunt");
                    break;

                case 5:
                    theanimator.Play("hit1");
                    break;

                case 6:
                    theanimator.Play("hit2");
                    break;

                case 7:
                    theanimator.Play("jump");
                    break;

                case 8:
                    theanimator.Play("death1");
                    break;

                case 9:
                    theanimator.Play("death2");
                    break;

                default:
                    theanimator.Play("walk");
                    exit_time = curr_time + 4000;
                    break;
            }
        }

    }
}
