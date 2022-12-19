using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SimpleJSON;
using System;
using System.Globalization;
using System.Text;

public class LicenseData 
{
    public int license_id = 0;
    public int product_id = 0;
    public string licenseKey = "";
    public DateTime expiresDate = new DateTime(1);
    public Dictionary<string,List<string>> features = new Dictionary<string,List<string>>();
    private string fromstring = "";

    public bool DecodeLicense(JSONNode data)
    {
        try
        {
            // Get license id
            if (data.HasKey("id"))
            {
                license_id = int.Parse(data["id"]);
            }
            else { return false; }

            // Get product productId
            if (data.HasKey("productId"))
            {
                license_id = int.Parse(data["productId"]);
            }
            else { return false; }

            // Get licenseKey
            if (data.HasKey("licenseKey"))
            {
                licenseKey = data["licenseKey"];
            }
            else { return false; }

            // Get license experation date
            if (data.HasKey("expiresAt"))
            {
                if (data["expiresAt"] == null)
                {
                    expiresDate = new DateTime(0);
                }
                else
                {
                    CultureInfo enUS = new CultureInfo("en-US");
                    string expires_date = data["expiresAt"] + " -04:00";
                    expiresDate = DateTime.ParseExact(expires_date, "yyyy-MM-dd HH:mm:ss zzz", enUS,DateTimeStyles.None );
                }
            }
            else { return false; }

            // Get features it unlocks
            if (data.HasKey("attributes"))
            {
                // Extract some important data
                JSONNode attributes = data["attributes"];

                // Load all features in
                foreach (string currkey in attributes.Keys)
                {
                    JSONNode currvalue = attributes[currkey];
                    List<string> values = new List<string>();

                    // Get the array of values
                    if (currvalue.IsArray)
                    {
                        for (int i = 0; i < currvalue.AsArray.Count; i++)
                        {
                            values.Add(currvalue[i]);
                        }
                    }
                    else
                    {
                        values.Add( currvalue );
                    }

                    features[currkey] = values;
                }
            }
            else { return false; }

        }
        catch (Exception e)
        {
            Debug.Log("Failed to read in internet license data: " + e);
            return false;
        }

        // Save data for later recovery
        fromstring = ToString();
        return true;
    }

    // Returns true if the feature is valid
    // All key unlocks everything
    // Reloads from string incase memory was tampered
    public bool IsFeatureValid(string key, string value)
    {
        // Reload feature
        FromString(fromstring);

        if (!features.ContainsKey("ALL"))
        {
            // Check of feature key exists
            if (!features.ContainsKey(key)) { return false; }

            // Check if feature value exists
            if (!features[key].Contains(value)) { return false; }
        }

        // If no expiry date present, return ok
        if(expiresDate.Ticks==0) { return true; }

        // Otherwise see how much time we have left
        TimeSpan timeleft = expiresDate - DateTime.Now;
        if( timeleft.TotalSeconds > 0 ) { return true;  }
        return false;
    }

    // Returns number of days left in the license
    // Returns 999 if no limit
    // Returns -1 if expired.
    // Returns number of days left, 0 if this is the last day.
    public int DaysLeft()
    {
         // If no limit, return 999
        if( expiresDate.Ticks == 0 ) { return 9999; }
        
        // Otherwise see how much time we have left
        TimeSpan timeleft = expiresDate - DateTime.Now;
        if( timeleft.Days < 0 ) { return -1; }
        return timeleft.Days;
    }

    // Returns a string listing all features enabled
    public string GetFeatureString()
    {
        string outstring = "";

        foreach(string key in features.Keys)
        {
            foreach(string value in features[key])
            {
                outstring += key + "=>" + value + "   ";
            }
        }

        return outstring;
    }

    // Returns true if the license key already is in our system
    public static bool DoesLicenseExist(string checkkey)
    {
        // If passed license key already exists in our global rpository, then retrun true.
        foreach( LicenseData currlicense in GLOBALS.myLicenses)
        {
            if( currlicense.licenseKey == checkkey) { return true; }
        }

        // Nothing found, return false
        return false;
    }

    // Returns an encrypted version of this data
    new public string ToString()
    {
        string outdata = "";

        outdata += license_id + ",";
        outdata += product_id + ",";
        outdata += licenseKey + ",";
        outdata += expiresDate.Ticks + ",";

        // Add all the feature keys
        foreach( string currkey in features.Keys)
        {
            List<string> currvalues = features[currkey];
            foreach(string currvalue in currvalues)
            {
                outdata += currkey + "," + currvalue + ",";
            }
        }

        // Now sign data
        outdata += MyUtils.EncryptRSA(outdata);

        // Finally compress it
        //byte[] compressed_data = MyUtils.CompressMessage(outdata, 1); ;

        //outdata = Encoding.ASCII.GetString(compressed_data, 0, compressed_data.Length); 
        return FixString(outdata, 1);
    }

    // Recreates the license based on the string
    public bool FromString(string indata)
    {
        // Save the string for later
        fromstring = indata;

        // Clear data
        license_id = 0;
        product_id = 0;
        expiresDate = new DateTime(1);
        features.Clear();

        try
        {
            // Decompress message
            string outdata = FixString(indata, -1); //  MyUtils.DecompressMessage(Encoding.ASCII.GetBytes(indata), 1);

            // Check MD6
            string[] alldata = outdata.Split(',');
            string RSAsignature = alldata[alldata.Length - 1];

            // Non MD5 raw string
            string indata_nosign = String.Join(",", alldata, 0, alldata.Length - 1);
            indata_nosign += ",";

            // Check MD5
            if (!MyUtils.RSAVerify(indata_nosign, RSAsignature))
            {
                Debug.LogError("Corrupted license key when reading from memory. Failed sanity check.");
                return false;
            }

            int i = 0;

            // Go through all data and extract info
            license_id = int.Parse(alldata[i++]);
            product_id = int.Parse(alldata[i++]);
            licenseKey = alldata[i++];
            expiresDate = new DateTime(long.Parse(alldata[i++]));

            // Load in all features
            features.Clear();
            while (i < alldata.Length - 2)
            {
                string key = alldata[i++];
                string value = alldata[i++];

                if (!features.ContainsKey(key))
                {
                    features[key] = new List<string>();
                }

                features[key].Add(value);
            }

            return true;
        }
        catch( Exception e)
        {
            Debug.LogError("Failed reading in license key from memory. Error:" + e);
            return false;
        }
    }

    // Adds a license to the global list from a saved string
    public static bool AddLicense(string md5data)
    {
        LicenseData newlicense = new LicenseData();
        if( ! newlicense.FromString(md5data))
        {
            return false;
        }

        GLOBALS.myLicenses.Add(newlicense);

        return true;
    }

    // Returns true if the robot is unlocked
    public static bool CheckRobotIsUnlocked(string robot, string skin )
    {
        // If no skin or Defualt skin, then we're o.k.
        if( (skin.Length < 1) || (skin.ToUpper() == "DEFAULT") ) { return true; }

        // Go through licenses
        foreach( LicenseData currlicense in GLOBALS.myLicenses)
        {
            if( currlicense.IsFeatureValid(robot, skin)) { return true; }
        }

        return false;
    }

    // Returns max number of days the feature is active
    // Returns -1 if invalid
    // Returns >=0 if valid
    // Returns 9999 if no limit
    public static int GetFeatureDaysLeft(string robot, string skin)
    {
        // If no skin or Defualt skin, then we're o.k.
        if ((skin.Length < 1) || (skin.ToUpper() == "DEFAULT")) { return 9999; }

        int daysleft = -1;

        // Go through licenses
        foreach (LicenseData currlicense in GLOBALS.myLicenses)
        {
            if (currlicense.IsFeatureValid(robot, skin)) 
            {
                int currdaysleft = currlicense.DaysLeft();
                if(currdaysleft > daysleft)
                {
                    daysleft = currdaysleft;
                }
            }
        }

        return daysleft;
    }

    private static string FixString(string instring, int mult)
    {
        char[] stringdata = instring.ToCharArray();

        // Incremenet ascii code in each position
        for(int i = 0; i < stringdata.Length; i++)
        {
            stringdata[i] = (char)(Convert.ToUInt16(stringdata[i]) + mult*(i%3));
        }

        return new string(stringdata);
    }

}
