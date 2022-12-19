using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;



public class xRC_MergeObjects : EditorWindow
{
	static List<GameObject> newSelected = new List<GameObject>();
	static bool switch_order = false;

	[MenuItem("Tools/xRC Merge Objects")]
	static void Init()
	{
		xRC_MergeObjects merger = (xRC_MergeObjects)EditorWindow.GetWindow(typeof(xRC_MergeObjects));
		merger.Show();
	}

	void OnGUI()
	{

		EditorGUILayout.PrefixLabel("Merge Options");
		if (Selection.gameObjects.Length == 2)
		{
			EditorGUILayout.TextField("Master =" + ((switch_order) ? Selection.gameObjects[1].name : Selection.gameObjects[0].name) );
			EditorGUILayout.TextField("To Be Merged =" + ((switch_order) ? Selection.gameObjects[0].name : Selection.gameObjects[1].name));
			if (GUILayout.Button("Merge"))
			{
				Merge();
			}
		}
		else
		{
			EditorGUILayout.TextField("Must select only 2 objects. Currently you have " + Selection.gameObjects.Length + " selected.");
		}
		switch_order = GUILayout.Toggle(switch_order, "Switch Master");
	}

	void OnInspectorUpdate()
	{
		Repaint();
	}

	GameObject[] selectObjects()
	{
		return Selection.gameObjects;
	}

	void Merge()
	{
		GameObject[] selected = selectObjects();

		if (selected.Length != 2) { return; }

		// Only grab top object. It will not be considered for extraction, only it's children
		Transform top_first = (switch_order) ? selected[1].transform : selected[0].transform;
		Transform top_second = (switch_order) ? selected[0].transform : selected[1].transform;


		// *******************************************
		// Create a copy of top_first
		GameObject merged_obj = Instantiate(top_first.gameObject);
		merged_obj.name = top_first.name + "_merged";
		Transform merged = merged_obj.transform;

		// *******************************************
		// Create a copy of top_second. It's components will move into the new top_first
		GameObject tomerge_obj = Instantiate(top_second.gameObject);
		tomerge_obj.name = top_second.name + "_tmp";
		Transform tomerge = tomerge_obj.transform;

		MyUtils.MergeRobotSkin(merged, tomerge);

		// Finally, delete tmp copy
		DestroyImmediate(tomerge_obj);
	}
}