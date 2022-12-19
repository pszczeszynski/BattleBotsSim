using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;

// FilterOptions already exists in another script, thus commented out here
//enum FilterOptions
//{
//	Tag, Layer, Both
//}

public class PruneSelectionEditor : EditorWindow
{
	FilterOptions filterOptions = FilterOptions.Tag;
	string selectedTag = "Untagged";
	int layer = 0;
	List<int> objectIndex = new List<int>();
	bool isFilterModeOn = false;

	static List<GameObject> newSelected = new List<GameObject>();

	[MenuItem("Tools/xRC Prune Selection by Tags")]
	static void Init()
	{
		PruneSelectionEditor filter = (PruneSelectionEditor)EditorWindow.GetWindow(typeof(PruneSelectionEditor));
		filter.Show();
	}

	void OnGUI()
	{

		EditorGUILayout.PrefixLabel("Filtering Options");
		filterOptions = (FilterOptions)EditorGUILayout.EnumPopup("Filter By", filterOptions);
		if (filterOptions == FilterOptions.Tag)
		{
			selectedTag = EditorGUILayout.TagField("Select Tag", selectedTag);
			if (GUILayout.Button("Prune by Tag"))
			{
				PruneSelected(filterOptions);
			}
		}
		else if (filterOptions == FilterOptions.Layer)
		{
			layer = EditorGUILayout.LayerField("Select Layer", layer);
			if (GUILayout.Button("Filter by Layer"))
			{
				PruneSelected(filterOptions);
			}
		}
		else
		{
			selectedTag = EditorGUILayout.TagField("Select Tag", selectedTag);
			layer = EditorGUILayout.LayerField("Select Layer", layer);
			if (GUILayout.Button("Filter by All"))
			{
				PruneSelected(filterOptions);
			}
		}
		// if filter mode show clear button
		if (isFilterModeOn)
		{
			GUI.color = Color.red;
			if (GUILayout.Button("Clear"))
			{
				isFilterModeOn = false;
				GameObject[] allObjects = selectObjects();
				foreach (GameObject o in allObjects)
				{
					if (o.hideFlags != HideFlags.HideAndDontSave &&
					   o.hideFlags != HideFlags.DontSave && o.hideFlags != HideFlags.NotEditable)
						o.hideFlags = o.hideFlags & ~HideFlags.HideInHierarchy;
				}
			}
			GUI.color = Color.white;
		}
		//		EditorGUILayout.Space();
		EditorGUILayout.LabelField("Save and Load Options");
		//		EditorGUILayout.Space();
		EditorGUILayout.BeginHorizontal();
		if (Selection.gameObjects.Length >= 1)
		{
			if (GUILayout.Button("Save Selection"))
			{
				int[] selectionIDs = Selection.instanceIDs;
				var saveStr = string.Empty;
				foreach (int i in selectionIDs)
					saveStr += i.ToString() + ";";
				saveStr = saveStr.TrimEnd(char.Parse(";"));
				EditorPrefs.SetString("SelectedIDs", saveStr);
			}
		}
		if (EditorPrefs.HasKey("SelectedIDs"))
		{
			if (GUILayout.Button("Load Selection"))
			{
				string[] strIDs = EditorPrefs.GetString("SelectedIDs").Split(char.Parse(";"));
				int[] ids = new int[strIDs.Length];
				for (var i = 0; i < strIDs.Length; i++)
					ids[i] = int.Parse(strIDs[i]);
				Selection.instanceIDs = ids;
			}
		}
		EditorGUILayout.EndHorizontal();
	}

	void OnInspectorUpdate()
	{
		Repaint();
	}

	GameObject[] selectObjects()
	{
		return Selection.gameObjects;
	}

	void PruneSelected(FilterOptions ops)
	{
		GameObject[] selected = selectObjects();

		// **************************************************
		// First, remove all objects that don't have our tag
		// **************************************************

		// Record all indexes from current selection that meet our filter criteria
		objectIndex = new List<int>();
		for (int i = 0; i < selected.Length; i++)
		{
			GameObject g = selected[i];

			if (ops == FilterOptions.Tag && g.tag == selectedTag)
			{
				objectIndex.Add(i);
			}
			else if (ops == FilterOptions.Layer && g.layer == layer)
			{
				objectIndex.Add(i);
			}
			else if (ops == FilterOptions.Both && g.layer == layer && g.tag == selectedTag)
			{
				objectIndex.Add(i);
			}
		}

		// Go through our indexes, retrieve the associated objects and copy it over to our new list	
		newSelected.Clear();
		for (int i = 0; i < objectIndex.Count; i++)
		{
			// Add the current selected object
			GameObject currobj = selected[objectIndex[i]];
			newSelected.Add(currobj);
		}

		// Now go through every item in the list and include all its children
		for (int i = 0; i < newSelected.Count; i++)
		{
			// Add the current selected object
			GameObject currobj = newSelected[i];

			// Go through every child of this current object and add it in as well
			for (int j = 0; j < currobj.transform.childCount; j++)
			{
				GameObject currchild = currobj.transform.GetChild(j).gameObject;
				if (!newSelected.Contains(currchild))
				{
					newSelected.Add(currchild);
				}
			}
		}

		// Next make a list of all parents that aren't part of our selection list
		List<GameObject> parents_not_selected = new List<GameObject>();

		// Now go through every item's parent and if they are not selected, remeber it in our parentlist 
		// These objects will need to be cleared of all components later
		for (int i = 0; i < newSelected.Count; i++)
		{
			// Traverse the current objects hierarchy (go up)
			GameObject currobj = newSelected[i];

			while( currobj.transform.parent != null)
			{
				currobj = currobj.transform.parent.gameObject;

				if( !newSelected.Contains(currobj))
				{
					parents_not_selected.Add(currobj);
				}
			}
		}

		// Now that we have our selected objects, their children and their parents remembered, delete any objects that are not part of any of these lists
		for (int i = 0; i < selected.Length; i++)
		{
			GameObject g = selected[i];

			// If neither object has this, delete the object
			if( !newSelected.Contains(g) && !parents_not_selected.Contains(g))
			{
				DestroyImmediate(g);
			}
		}

		// Lastly, lets delete all components inside the parents that are not marked
		for( int i=0; i < parents_not_selected.Count; i++)
		{
			GameObject g = parents_not_selected[i];

			Component[] currcomp = g.GetComponents<Component>();
			for( int j = currcomp.Length-1; j >= 0; j--)
			{
				if (currcomp[j].GetType() != typeof(Transform))
				{ DestroyImmediate(currcomp[j]); } 
			}
		}


		Selection.objects = newSelected.ToArray() as Object[];

		// set filter mode on option to draw a gui button
		isFilterModeOn = true;
		

	}

}