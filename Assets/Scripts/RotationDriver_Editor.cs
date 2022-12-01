#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

// Sets our target type
[CustomEditor(typeof(RotationDriver))]
public class RotationDriver_Editor : Editor
{
    public override void OnInspectorGUI() {
        DrawDefaultInspector();

        RotationDriver myRotationDriver = (RotationDriver) target;



        if (GUILayout.Button("Add Keyframe")) {
            myRotationDriver.AddKeyframe();
            EditorUtility.SetDirty(target);

        }

        if (GUILayout.Button("Remove Keyframe")) {
            myRotationDriver.RemoveKeyframe();
            EditorUtility.SetDirty(target);

        }

        if (GUILayout.Button("Restore KeyFrame")) {
            myRotationDriver.RestoreKeyframe();
            EditorUtility.SetDirty(target);

        }

        if (GUILayout.Button("Set Keyframe")) {
            myRotationDriver.SetKeyframe();
            EditorUtility.SetDirty(target);

        }

        if (GUILayout.Button("SetInitialPos"))
        {
            myRotationDriver.SetInitialPos();
            EditorUtility.SetDirty(target);

        }

        if (GUILayout.Button("Initialize"))
        {
            myRotationDriver.InitializeInEditor();
            EditorUtility.SetDirty(target);
        }

    }
}
#endif
