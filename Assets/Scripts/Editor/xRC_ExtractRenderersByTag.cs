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

public class xRC_ExtractRenderersByTag : EditorWindow
{
	FilterOptions filterOptions = FilterOptions.Tag;
	string selectedTag = "Untagged";
	bool enableMeshes = false;
	bool removeUnapprovedComponents = false;
	bool removeFromOriginal = false;
	int layer = 0;


	static List<GameObject> newSelected = new List<GameObject>();

	[MenuItem("Tools/xRC Extract Renderers By Tag")]
	static void Init()
	{
		xRC_ExtractRenderersByTag filter = (xRC_ExtractRenderersByTag)EditorWindow.GetWindow(typeof(xRC_ExtractRenderersByTag));
		filter.Show();
	}

	void OnGUI()
	{

		EditorGUILayout.PrefixLabel("Filtering Options");
		filterOptions = (FilterOptions)EditorGUILayout.EnumPopup("Filter By", filterOptions);
		if (filterOptions == FilterOptions.Tag)
		{
			selectedTag = EditorGUILayout.TagField("Select Tag", selectedTag);
			if (GUILayout.Button("Extract by Tag"))
			{
				ExtractRenderers(filterOptions);
			}
		}
		else if (filterOptions == FilterOptions.Layer)
		{
			layer = EditorGUILayout.LayerField("Select Layer", layer);
			if (GUILayout.Button("Filter by Layer"))
			{
				ExtractRenderers(filterOptions);
			}
		}
		else
		{
			selectedTag = EditorGUILayout.TagField("Select Tag", selectedTag);
			layer = EditorGUILayout.LayerField("Select Layer", layer);
			if (GUILayout.Button("Filter by All"))
			{
				ExtractRenderers(filterOptions);
			}
		}

		enableMeshes = GUILayout.Toggle(enableMeshes, "Enable Renderers in extracted");
		removeUnapprovedComponents = GUILayout.Toggle(removeUnapprovedComponents, "Remove Non-Renderer Components");
		removeFromOriginal = GUILayout.Toggle(removeFromOriginal, "Remove extracted from Original");

	}

	void OnInspectorUpdate()
	{
		Repaint();
	}

	GameObject[] selectObjects()
	{
		return Selection.gameObjects;
	}

	void ExtractRenderers(FilterOptions ops)
	{
		GameObject[] selected = selectObjects();

		if( selected.Length < 1) { return; }

		// Only grab top object. It will not be considered for extraction, only it's children
		Transform top_parent = selected[0].transform;

		while( top_parent.transform.parent)
		{
			top_parent = top_parent.transform.parent;
		}

		// *******************************************
		// Create a copy of this object
		GameObject extracted_obj = Instantiate(top_parent.gameObject);
		extracted_obj.name = top_parent.name + "_extracted";

		// *********************************************************
		// FIND ALL TAGGED OBJECTS
		//
		// Go through top parent' schildren and find all tagged objects
		List<Transform> objectstoinspect = new List<Transform>();
		List<Transform> foundmatches = new List<Transform>();

		objectstoinspect.Add(extracted_obj.transform);

		for (int i = 0; i < objectstoinspect.Count; i++)
		{
			Transform currobj = objectstoinspect[i];
			GameObject g = currobj.gameObject;

			// Add the children to our list for inspection
			for( int j=0; j < currobj.childCount; j++ )
			{
				objectstoinspect.Add(currobj.GetChild(j));
			}

			// If this object meets our filter, add it to our list
			if (ops == FilterOptions.Tag && g.tag == selectedTag)
			{
				foundmatches.Add(currobj);
			}
			else if (ops == FilterOptions.Layer && g.layer == layer)
			{
				foundmatches.Add(currobj);
			}
			else if (ops == FilterOptions.Both && g.layer == layer && g.tag == selectedTag)
			{
				foundmatches.Add(currobj);
			}
		}

		// *********************************************************
		// ADD ALL CHILDREN OF TAGGED OBJECTS
		// But only if they aren't tagged
		// Now go through every item in the list and include all its children (and their children's children, etc..)
		for (int i = 0; i < foundmatches.Count; i++)
		{
			// Add the current selected object
			Transform currobj = foundmatches[i];

			// Go through every child of this current object and add it in as well
			for (int j = 0; j < currobj.childCount; j++)
			{
				Transform currchild = currobj.GetChild(j);
				if (!foundmatches.Contains(currchild) && currchild.gameObject.tag == "Untagged")
				{
					foundmatches.Add(currchild);
				}
			}
		}

		// *********************************************************
		// MAKE A LIST OF PARENTS TO PRESERVE THEIR HIERARCHY
		//
		// Next make a list of all parents that aren't part of our selection list
		List<Transform> parents_not_selected = new List<Transform>();

		// Now go through every item's parent and if they are not selected, remeber it in our parentlist 
		// These objects will need to be cleared of all components later
		for (int i = 0; i < foundmatches.Count; i++)
		{
			// Traverse the current objects hierarchy (go up)
			Transform currobj = foundmatches[i];

			while( currobj.parent != null)
			{
				currobj = currobj.parent;

				if( !foundmatches.Contains(currobj))
				{
					parents_not_selected.Add(currobj);
				}
			}
		}

		// *********************************************************
		// PRUNE ALL EXCESS OBJECTS
		//
		for (int i = objectstoinspect.Count-1; i >= 0; i--)
		{
			Transform g = objectstoinspect[i];

			// If neither object has this, delete the object
			if (!foundmatches.Contains(g) && !parents_not_selected.Contains(g))
			{
				DestroyImmediate(g.gameObject);
			}
		}

		// *********************************************************
		// Delete all components in non-selected parents
		//
		for (int i = 0; i < parents_not_selected.Count; i++)
		{
			Transform g = parents_not_selected[i];

			Component[] currcomp = g.GetComponents<Component>();
			for (int j = currcomp.Length - 1; j >= 0; j--)
			{
				if (currcomp[j].GetType() != typeof(Transform))
				{ DestroyImmediate(currcomp[j]); }
			}
		}

		// *************************************
		// OPTIONAL: Prune all components that are not "approved" in foundmatches
		// However, a special case needs to be made if a Collider exists: in this case we must
		// preserve it and any meshes in the original component. 
		// What we're going to do is copy over everything, including the collider, and during merging back in ignore everything if collider exists
		if (removeUnapprovedComponents)
		{
			// Prune unwanted components in all matches
			for (int i = 0; i < foundmatches.Count; i++)
			{
				Transform g = foundmatches[i];

				Component[] currcomp = g.GetComponents<Component>();

				// Check if collider exists
				if(g.GetComponent<Collider>() != null)
				{
					continue;
				}

				// Do the destruction loop several times through since there are inter-dependencies that sometimes require
				// order of destruction
				for (int k = 0; k < 6; k++)
				{
					for (int j = currcomp.Length - 1; j >= 0; j--)
					{
						if ((currcomp[j].GetType() != typeof(Transform)) &&
							 (currcomp[j].GetType() != typeof(Renderer)) &&
							 (!currcomp[j].GetType().IsSubclassOf(typeof(Renderer))) &&
							 (currcomp[j].GetType() != typeof(TMPro.TextMeshPro)) &&
							 (currcomp[j].GetType() != typeof(Material)) &&
							 (!currcomp[j].GetType().IsSubclassOf(typeof(Material))) &&
							 (currcomp[j].GetType() != typeof(RectTransform)) &&
							 (currcomp[j].GetType() != typeof(Shader)) &&
							 (currcomp[j].GetType() != typeof(MeshFilter))						
							)
						{ DestroyImmediate(currcomp[j]); }
					}
				}
			}
		}

		// *************************************
		// OPTIONAL:enable renderers
		if (enableMeshes)
		{
			// Prune unwanted components in all matches
			for (int i = 0; i < foundmatches.Count; i++)
			{
				Transform g = foundmatches[i];

				Renderer[] currcomp = g.GetComponents<Renderer>();
				for (int j = currcomp.Length - 1; j >= 0; j--)
				{
					currcomp[j].enabled = true;
				}
			}
		}

		// *************************************
		// OPTIONAL:Remove from original
		// 
		// Remove all components from original that were copied over
		// Then go through original and remove any empty paths
		if (removeFromOriginal)
		{

			// ***************************
			// First remove all components in original that are in copies
			for (int i = 0; i < foundmatches.Count; i++)
			{
				Transform t_copy = foundmatches[i];

				// Find sister component in original top_parent
				Transform t_original = top_parent.Find(MyUtils.GetFullName(t_copy, extracted_obj.transform));

				if( t_copy.name == "SimMotorVisual")
				{
					t_copy.gameObject.SetActive(true);
				}

				if ( t_original == null)
				{
					Debug.Log("Transform not found in parent! " + MyUtils.GetFullName(t_copy));
					continue;
				}

				// If a collider exists, then only turn-off the mesh renderer
				if( t_copy.GetComponent<Collider>() != null)
				{
					Renderer top_renderer = t_original.GetComponent<Renderer>();
					if( top_renderer != null)
					{
						top_renderer.enabled = false;
					}
					continue;
				}

				// Remove all matching components. Assumes only 1 component of each type!
				for (int loop = 0; loop < 6; loop++) // Do it a couple times since sometimes component dependencies don't allow us to remove one without first removing the other
				{
					foreach (Component currcomp in t_copy.GetComponents<Component>())
					{
						if (currcomp.GetType() == typeof(Transform)) { continue; } // can'tremove transforms
						if (currcomp.GetType() == typeof(RectTransform)) { continue; } // can'tremove transforms

						Component curr_original = t_original.GetComponent(currcomp.GetType());
						if (curr_original)
						{
							DestroyImmediate(curr_original);
						}
					}
				}
			}

			// *************************************************
			// Next delete all objects that have no children or components.
			// The list is not guaranteed to be tree-generated since the initial seeding of "tagged" objects cna have tags underneath tags mixed with non-tags.
			// We therefore need to iterate on the removal until no more changes
			// ***************************
			// First remove all components in original that are in copies
			int sanity_check = 20; // Maximum number of iterations before we give up
			bool change_occured = true;
			while ((sanity_check > 0) || change_occured)
			{
				sanity_check--;
				change_occured = false;

				for (int i = foundmatches.Count - 1; i >= 0; i--)
				{
					Transform t_copy = foundmatches[i];

					// Find sister component in original top_parent
					Transform t_original = top_parent.Find(MyUtils.GetFullName(t_copy, extracted_obj.transform));

					if (t_original == null)
					{
						// we could be here because this is a multiple iterations
						continue;
					}

					if( t_copy.name.Contains("NametagB1"))
					{
						t_copy.name = "NametagB1";
					}

					// If original has no components or children, then delete it
					if ((t_original.childCount < 1) && (t_original.GetComponents<Component>().Length < 2))
					{
						DestroyImmediate(t_original.gameObject);
						change_occured = true;
					}
				}
			}

			Debug.Log("Sanity Check = " + sanity_check);
		}


	}

}