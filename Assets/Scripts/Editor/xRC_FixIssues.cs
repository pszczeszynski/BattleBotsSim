using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;



public class xRC_FixIssues : EditorWindow
{
	static List<GameObject> newSelected = new List<GameObject>();
	static int issues_fixed = 0;

	[MenuItem("Tools/xRC Fix Naming Issues")]
	static void Init()
	{
		xRC_FixIssues fixissues = (xRC_FixIssues)EditorWindow.GetWindow(typeof(xRC_FixIssues));
		fixissues.Show();
	}

	void OnGUI()
	{

		EditorGUILayout.PrefixLabel("Fix Robot Options");
		if (Selection.gameObjects.Length == 1)
		{
			EditorGUILayout.TextField("Top Cell =" + Selection.gameObjects[0].name );
			if (GUILayout.Button("Fix Issues"))
			{
				issues_fixed = 0;

				// Get top prefab
				GameObject root = Selection.gameObjects[0];
				FixIssues(root.transform);

				// Write it back
				// var prefabAsset = Selection.activeObject;
				// PrefabUtility.SaveAsPrefabAsset(root, AssetDatabase.GetAssetPath(prefabAsset));
			}
			EditorGUILayout.TextField("Issues fixed = " + issues_fixed);
		}
		else
		{
			EditorGUILayout.TextField("Must select only 1 objects. Currently you have " + Selection.gameObjects.Length + " selected.");
		}

	}

	void OnInspectorUpdate()
	{
		Repaint();
	}

	// Recursive function that goes through all children and make sure there are
	// no name conflicts at the current level
	void FixIssues(Transform top_first)
	{

		// *********************************************************
		// Go through all objects and if any are the same name in the same level of hierarchy,
		// then give them a unique number
		List<string> current_names = new List<string>();

		// Now iterate through all the children top-down: add more children if they exist, otherwise keep going;
		for (int i = 0; i < top_first.childCount; i++)
		{
			Transform currobj = top_first.GetChild(i);

			// Recurse it's children
			FixIssues(currobj);

			// If this name hasn't been taken, then add it to our list
			if( !current_names.Contains(currobj.name))
			{
				current_names.Add(currobj.name);
				continue;
			}

			// Otherwise we have to find a unique name
			bool unique_name_found = false;
			for( int j=1; (j<999) && !unique_name_found; j++)
			{
				string new_name = currobj.name + "_" + j;

				if( current_names.Contains(new_name)) { continue; }

				// Change object to this new name
				currobj.name = new_name;
				current_names.Add(new_name);
				unique_name_found = true;
				issues_fixed += 1;
				Debug.Log("Changed object to " + MyUtils.GetFullName(currobj));
			}

			// If unique name was not found then throw an error
			if( !unique_name_found) {
				Debug.Log("A unique name was not found! Original name = " + currobj.name + ", last number added and failed = 998");
			}			
		}
	}
}