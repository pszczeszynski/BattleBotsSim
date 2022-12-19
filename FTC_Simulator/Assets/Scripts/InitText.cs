using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class InitText : MonoBehaviour
{
    public TMPro.TMP_InputField text_to_init = null;
    public string global_variable_name = "";

    // Start is called before the first frame update
    void Start()
    {
        if( text_to_init == null) { return; }

        var global_variable = typeof(GLOBALS).GetField(global_variable_name, BindingFlags.Public | BindingFlags.Static);
        string init_value = (string) global_variable.GetValue(null);

        text_to_init.text = init_value;
    }

}
