using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MinQualityLevel : MonoBehaviour
{
    // System quality must >= this setting to enable component
    public int quality = 2;
    public int maxquality = -1; // If set >= 1, it wil lbe disable if quality is above this number

    public bool disableEveryFrame = false;
    public bool enableEveryFrame = false;

    private void Update()
    {
        if( !disableEveryFrame && !enableEveryFrame) { return; }

        if( (QualitySettings.GetQualityLevel() < quality) || (maxquality >= 0 && QualitySettings.GetQualityLevel() >= maxquality))
        {
            if( gameObject.activeSelf && disableEveryFrame )
            {
                gameObject.SetActive(false);
            }
        }
        else 
        {
            if (!gameObject.activeSelf && enableEveryFrame)
            {
                gameObject.SetActive(true);
            }
        }

    }

    public void Enforce()
    {
        if ((QualitySettings.GetQualityLevel() < quality) || (maxquality >= 0 && QualitySettings.GetQualityLevel() >= maxquality))
        {
            if (gameObject.activeSelf)
            {
                gameObject.SetActive(false);
            }
        }
        else
        {
            if (!gameObject.activeSelf )
            {
                gameObject.SetActive(true);
            }
        }
    }

}
