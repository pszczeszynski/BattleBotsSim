using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ProgressMeter : MonoBehaviour
{
    Image progressbar;
    Canvas mycanvas;

    private void Start()
    {
        progressbar = gameObject.GetComponent<Image>();
        mycanvas = gameObject.GetComponent<Canvas>();
        SetProgress(0);
    }

    // A value from 0 to 1 sets the progress bar
    public void SetProgress(float value)
    {
        if( mycanvas == null) { return; }

        if (value <= 0f)
        {
            progressbar.fillAmount = 0;
            mycanvas.enabled = false;
            return;
        }
        mycanvas.enabled = true;
        progressbar.fillAmount = (value > 1f) ? 1f : value;
    }

    public void SetColor( Color newcolor)
    {
        // If progressbar hasn't been initialize, initialize it. If it still is null, that means it was destroyed.
        if( progressbar == null ) { progressbar = gameObject.GetComponent<Image>(); }
        if(progressbar == null) { return; }
        progressbar.color = newcolor;
    }
}

