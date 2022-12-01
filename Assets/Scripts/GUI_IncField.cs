using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class GUI_IncField : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // Increment any input field
    public void IncrementField()
    {
        InputField myfield = GetComponent<InputField>();

        if( !myfield ) { return;  }

        // Increment current value
        myfield.text = (int.Parse(myfield.text) + 1).ToString();
    }

    public void DecrementField()
    {
        InputField myfield = GetComponent<InputField>();

        if (!myfield) { return; }

        // Increment current value
        myfield.text = (int.Parse(myfield.text) - 1).ToString();
    }

}
