using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;



public class xRC_giveUniqueID : EditorWindow
{
	static List<GameObject> newSelected = new List<GameObject>();
	static int starting_number = 1;

	[MenuItem("Tools/xRC Give Unique Game ID")]
	static void Init()
	{
		xRC_giveUniqueID giveuniqueid = (xRC_giveUniqueID)EditorWindow.GetWindow(typeof(xRC_giveUniqueID));
		giveuniqueid.Show();
	}

	void OnGUI()
	{

		EditorGUILayout.PrefixLabel("Give Unique ID Script");

		if (Selection.gameObjects.Length >= 1)
		{
			EditorGUILayout.TextField("Starting Number" );
			starting_number = EditorGUILayout.IntField("Starting number:", starting_number);

			if (GUILayout.Button("Apply") )
			{

				GiveUniqueIDs(Selection.gameObjects);

			}
			EditorGUILayout.TextField("Final Value = " + starting_number);
		}
		else
		{
			EditorGUILayout.TextField("Must select >= 1 objects. Currently you have " + Selection.gameObjects.Length + " selected.");
		}

	}

	void OnInspectorUpdate()
	{
		Repaint();
	}

	// Recursive function that goes through all children and make sure there are
	// no name conflicts at the current level
	void GiveUniqueIDs(GameObject[] allobjects)
	{
		foreach(GameObject currobj in allobjects)
        {
			// Get the game stuff
			gameElement currelement = currobj.GetComponent<gameElement>();

			if( currelement )
            {
				currelement.id = starting_number;
				starting_number += 1;
				EditorUtility.SetDirty(currelement);

			}
        }

	}
}