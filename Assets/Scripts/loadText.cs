using UnityEngine;
using System.Collections;
using System.IO;
using System;

public class loadText : MonoBehaviour {
    public string txtFile = "test";
    string fileContents;
    string[] fileLines;


	// Use this for initialization
	void Start () {
        
    }

    // Update is called once per frame
    void Update () {
	    
	}
    // Update is called once per frame
    void OnGUI()
    {
        TextAsset txtAssets = (TextAsset)Resources.Load(txtFile);
        fileContents = txtAssets.text;
        fileLines = fileContents.Split(new[] { Environment.NewLine }, StringSplitOptions.None);


        GUILayout.Label(fileLines[0]);
    }
}
