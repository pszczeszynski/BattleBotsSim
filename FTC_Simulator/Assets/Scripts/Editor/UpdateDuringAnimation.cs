using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(BBLightManager))]
public class UpdateDuringAnimation : Editor
{
    private void OnEnable()
    {
        // Subscribe to editor update event
        EditorApplication.update += OnEditorUpdate;
    }

    private void OnDisable()
    {
        // Unsubscribe to prevent memory leaks
        EditorApplication.update -= OnEditorUpdate;
    }
    private void OnEditorUpdate()
    {
        // Check if we're in animation mode
        if (AnimationMode.InAnimationMode())
        {
            // Cast the 'target' property to MyScript to call MyFunction
            if (target != null)
            {
                BBLightManager myScript = target as BBLightManager;
                if (myScript != null)
                {
                    myScript.UpdateLights();
                }
            }
        }
    }

}