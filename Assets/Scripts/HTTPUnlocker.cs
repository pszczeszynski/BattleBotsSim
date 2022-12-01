using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;
using SimpleJSON;

public class HTTPUnlocker : MonoBehaviour
{
    Text status_text;
    private bool coroutine_running = false;


    void Start()
    {
        status_text = transform.Find("Panel/Status").GetComponent<Text>();
    }


    // Starts the process of validating license
    public void StartUnlocking()
    {
        // Only run if coroutine is finished
        if( !coroutine_running)
        {
            status_text.text = "Contacting server....";

            // retrieve the key
            Transform licensekey = transform.Find("Panel/licensekey");
            TMPro.TMP_InputField licensetext = licensekey.GetComponent<TMPro.TMP_InputField>();

            // Make sure key is valid
            if(licensetext.text.Length < 19)
            {
                status_text.text = "Full license key must be of the form XXXX-XXXX-XXXX-XXXX";
                return;
            }

            if( LicenseData.DoesLicenseExist(licensetext.text))
            {
                status_text.text = "This license is already enabled.";
                return;
            }
            coroutine_running = true;
            StartCoroutine(RunHTTPCoroutine(licensetext.text));
        }
        else
        {
            status_text.text = "Unlock request in progress, please wait until it is complete.";
        }
        
    }

    // Authentication string
    private string authenticate(string username, string password)
    {
        string auth = username + ":" + password;
        auth = System.Convert.ToBase64String(System.Text.Encoding.GetEncoding("ISO-8859-1").GetBytes(auth));
        auth = "Basic " + auth;
        return auth;
    }


    IEnumerator RunHTTPCoroutine(string license_key)
    {
        JSONNode returned_data ;

        // Start the web request
        using (UnityWebRequest www = UnityWebRequest.Get(GLOBALS.HTTP_LICENSE_ACTIVATE + license_key))
        {
            // Add authentications
            www.SetRequestHeader("AUTHORIZATION", authenticate(GLOBALS.HTTP_LICENSE_KEY, GLOBALS.HTTP_LICENSE_SECRET));

            yield return www.SendWebRequest();

            // split up any data that may or may have not been returned
            // The data is json format
            returned_data = SimpleJSON.JSON.Parse(www.downloadHandler.text);
        
            // If unnsuccesful, report errror and quit
            if (www.isNetworkError || www.isHttpError)
            {
                // An error code could be the license manager responding to bad license
                if (returned_data.HasKey("message"))
                {
                    status_text.text = returned_data["message"];
                }
                else
                {
                    status_text.text = "Error! Unable to connect to master server!";
                }
                coroutine_running = false;
                yield break;
            }
        }

        // All our data should be in "data" key
        if( !returned_data.HasKey("data") || !returned_data["data"].HasKey("attributes") )
        {
            status_text.text = "Something went wrong: did not receive required data back. Contact support@xrcsimulator.org if this persists.";
            coroutine_running = false;
            yield break;

        }

        LicenseData newlicense = new LicenseData();
        if( ! newlicense.DecodeLicense(returned_data["data"]) )
        {
            status_text.text = "Something went wrong: unable to determine license features. Contact support@xrcsimulator.org if this persists.";
            coroutine_running = false;
            yield break;
        }

        // Add license to global list
        GLOBALS.myLicenses.Add(newlicense);

        status_text.text = "License activated: Thank you for supporting XRC Simulator!";
        if(newlicense.DaysLeft() < 9999)
        {
            status_text.text += "\r\nDays Left = " + newlicense.DaysLeft();
        }
       
        status_text.text += "\r\nFeatures enabled: \r\n";
        status_text.text += newlicense.GetFeatureString();

        // Save new key
        Settings.SavePrefs();

        coroutine_running = false;
    }

}
