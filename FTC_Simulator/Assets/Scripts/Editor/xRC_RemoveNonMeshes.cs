using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine.ProBuilder;

// FilterOptions already exists in another script, thus commented out here
//enum FilterOptions
//{
//	Tag, Layer, Both
//}

public class xRC_RemoveNonMeshes : EditorWindow
{
	[MenuItem("Tools/xRC Remove Colliders")]
	static void Init()
	{
		xRC_RemoveNonMeshes filter = (xRC_RemoveNonMeshes)EditorWindow.GetWindow(typeof(xRC_RemoveNonMeshes));
		filter.Show();
	}

	static public int num_removed = 0;

	void OnGUI()
	{

		EditorGUILayout.PrefixLabel("Remove Non Meshes");
		if (Selection.gameObjects.Length == 1)
		{
			EditorGUILayout.TextField("Top Cell =" + Selection.gameObjects[0].name);
			if (GUILayout.Button("Remove"))
			{
				Remove();
			}
		}
		else
		{
			EditorGUILayout.TextField("Must select only 1 objects. Currently you have " + Selection.gameObjects.Length + " selected.");
		}

		EditorGUILayout.TextField("Removed " + num_removed + " components.");

	}

	void OnInspectorUpdate()
	{
		Repaint();
	}

	GameObject[] selectObjects()
	{
		return Selection.gameObjects;
	}

	void Remove()
	{
		num_removed = 0;

		GameObject[] selected = selectObjects();

		if( selected.Length != 1) { return; }

		// Only grab top object. It will not be considered for extraction, only it's children
		Transform top_parent = selected[0].transform;

		List<Transform> all_objects = new List<Transform>();
		all_objects.Add(top_parent);

		for (int i = 0; i < all_objects.Count; i++)
		{
			Transform g = all_objects[i];

			// Add it's children
			for (int j = 0; j < g.childCount; j++ )
			{
				all_objects.Add(g.GetChild(j));
			}

			// *************************************
			// Prune all components that are not "approved" in foundmatches
			Component[] currcomp = g.GetComponents<Component>();

			// Do the destruction loop several times through since there are inter-dependencies that sometimes require
			// order of destruction
			for (int k = 0; k < 6; k++)
			{
				for (int j = currcomp.Length - 1; j >= 0; j--)
				{
					if ((currcomp[j].GetType() == typeof(Collider)) ||
						typeof(Collider).IsAssignableFrom(currcomp[j].GetType()) ||
						(currcomp[j].GetType() == typeof(Rigidbody)) ||
						(currcomp[j].GetType() == typeof(Joint)) ||
						typeof(Joint).IsAssignableFrom(currcomp[j].GetType())
						)
				
					{
						DestroyImmediate(currcomp[j]);
						num_removed++;

					}
				}
			}
		

		}


	}

}