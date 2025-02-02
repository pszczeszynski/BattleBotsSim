using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine;

public class AdjustMaterialTiling : MonoBehaviour
{
    static int randomizer = 0;
    static bool initialized = false;

    private void Awake()
    {
        if( !initialized )
        {
            randomizer = (int)(Random.value * 100);
            initialized = true;
        }
    }

    void Start()
    {
        // Get my row and column
        float row = transform.GetSiblingIndex();
        float column = transform.parent.GetSiblingIndex();

        // Get the MeshRenderer component attached to this GameObject
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();

        if (meshRenderer != null)
        {
            // Ensure there's at least one material in the array
            if (meshRenderer.materials.Length > 0)
            {
                // Get the first material (assuming you want to adjust the first or only material)
                Material material = meshRenderer.materials[0];

                // Set the new tiling offset for this material
                material.mainTextureOffset = material.mainTextureScale * (new Vector2(row + randomizer, column+randomizer/2));
            }
            else
            {
                Debug.LogWarning("No materials found on the MeshRenderer.");
            }
        }
        else
        {
            Debug.LogError("No MeshRenderer component found on this GameObject.");
        }
    }
}