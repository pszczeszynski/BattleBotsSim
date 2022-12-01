using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VFX;

public class Fireworks : MonoBehaviour
{
    public VisualEffectAsset reduced_fireworks;

    private void Start()
    {
        // If graphics quality level is low, reduce the fireworks
        if(QualitySettings.GetQualityLevel() < 3)
        {
            // Reduce fireworks to the poorer version
            foreach(VisualEffect curreffect in transform.GetComponentsInChildren<VisualEffect>())
            {
                curreffect.visualEffectAsset = reduced_fireworks;
            }
        }
    }
    public void Disable()
    {
        transform.gameObject.SetActive(false);
    }
}
