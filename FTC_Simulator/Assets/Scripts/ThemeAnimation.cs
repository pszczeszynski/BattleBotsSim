using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ThemeAnimation : MonoBehaviour {

    public bool animation_started = false;
    public bool animation_finished = false;
    public float multiplier = 1f;

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {


    }

    private void LateUpdate()
    {
        if(transform.localPosition.y != 0)
        {
            Vector3 pos = transform.localPosition;
            pos.y *= multiplier;
            transform.localPosition = pos;
        }

        // if the animation is finished and it was started
        if (animation_finished && animation_started)
        {
            // set the flag
            animation_started = false;
            animation_finished = false;

            transform.parent.localPosition += transform.localPosition;
            // update the box position to zero inside the parent
            transform.localPosition = Vector3.zero;
            multiplier = 1f;
        }
    }
 

    public void MoveUp(float mymultiplier = 1f)
    {
        multiplier = mymultiplier;
        animation_started = true;
        animation_finished = false;

        GetComponent<Animation>().Play("themeUp");
    }

    public void MoveDown(float mymultiplier = 1)
    {
        multiplier = mymultiplier;
        animation_started = true;
        animation_finished = false;

        GetComponent<Animation>().Play("themeDown");
    }
}
