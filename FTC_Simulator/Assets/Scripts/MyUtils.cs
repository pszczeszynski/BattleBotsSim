using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using Ionic.Zlib;
using System.IO;
using System.Security.Cryptography;
using UnityEngine.XR;
using UnityEngine.XR.Management;

//using System.Diagnostics;

// *********************************
//  MyUtils
//
//  Generic class containing static methods that simply life
//


// This functions outputs various info into seperate files
// for the purpose of streaming
// Currently: Outputs all info from the "SCORE" key
public class statusfile  // Keeps track of all the streams and the last value (don't update if value didn't change)
{
    public FileInfo fileinfo;
    public FileStream stream;
    public string value;
};

// Single data structure to save a data point
public class Saved_Data
{
    public int timestamp;
    public string[] data;
}





public static class MyUtils
{
    static long start_time = DateTime.Now.Ticks;

    // Get time in milliseconds
    public static long GetTimeMillis()
    {
        return DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
    }

    public static long GetTimeMillisSinceStart()
    {
        return (DateTime.Now.Ticks - start_time) / TimeSpan.TicksPerMillisecond;
    }

    // Get time delta in s since start of program
    public static float GetTimeSinceStart()
    {
        return ((float)(DateTime.Now.Ticks - start_time)) / ((float)TimeSpan.TicksPerSecond);
    }

    // Sending utility functions
    // Combine 2 byte arrays
    public static byte[] CombineByteArrays(byte[] first, byte[] second)
    {
        // Combine byte arrays
        byte[] combined = new byte[first.Length + second.Length];
        System.Buffer.BlockCopy(first, 0, combined, 0, first.Length);
        System.Buffer.BlockCopy(second, 0, combined, first.Length, second.Length);

        return combined;
    }

    // Compress Message
    public static byte[] CompressMessage(string message, int compression = GLOBALS.PACKET_COMPRESSION)
    {
        if (GLOBALS.PACKET_COMPRESSION == 0)
        {
            return Encoding.UTF8.GetBytes(message);
        }

        // Now zip it up
        // Fast compression
        if (compression == 1)
        {
            return CompressionHelper.Compress(message);
        }

        // Better compression
        if (compression == 2)
        {
            return ZlibStream.CompressString(message.ToString());
        }

        // Medium and fastest but with restrictions
        if (compression == 3)
        {
            return BitPacking.Compress(message.ToString());
        }

        // No compression
        return Encoding.UTF8.GetBytes(message);
    }

    // Compress Message
    public static string DecompressMessage(byte[] extracted_data, int compression = GLOBALS.PACKET_COMPRESSION)
    {
        byte[] decompressed_data;

        // Uncompress data if applicable
        if (compression == 0)
        {
            decompressed_data = extracted_data;
        }
        else if (1 == compression)
        {
            decompressed_data = CompressionHelper.Decompress(extracted_data);
        }
        else if (2 == compression)
        {
            decompressed_data = ZlibStream.UncompressBuffer(extracted_data);
        }
        else if (3 == compression)
        {
            decompressed_data = BitPacking.Decompress(extracted_data);
        }
        else
        {
            decompressed_data = extracted_data;
        }

        return Encoding.UTF8.GetString(decompressed_data);
    }


    public static bool ExtractMessageHeader(byte[] message, List<byte[]> extracted_data)
    {
        // ************ PASSCODE **************

        // We will run it 4 times:
        // TBD = Add packet integrity to front of packet in next full releae
        // 1st = passcode
        // 2nd = compression id
        // 3rd = Client id (time-stamp when received from server)
        // 4th = data length
        // 5th = data

        int start = 0;
        byte[] extracted;

        for (int count = 0; count < 4; count++) // find the above 4 things, remaining will be the 5th
        {
            int i = 0;
            for (; i + start < message.Length; i++) // iterate through all the remaining bytes until we find the separator
            {
                if (message[i + start] == GLOBALS.SEPARATOR1)
                {
                    break;
                }
            }

            // Make sure we found it
            if (i + start == message.Length || i == 0) { return false; }

            // Copy it over to extracted_data
            extracted = new byte[i];
            System.Buffer.BlockCopy(message, start, extracted, 0, i);
            extracted_data.Add(extracted);

            start += i + 1;
        }

        // Found all items, remaining data will be the last entry
        extracted = new byte[message.Length - start];
        System.Buffer.BlockCopy(message, start, extracted, 0, message.Length - start);
        extracted_data.Add(extracted);

        return true;
    }

    static public byte[] StringToByteArray(string input)
    {
        byte[] output = new byte[input.Length];

        for (int i = 0; i < input.Length; i++)
        {
            output[i] = (byte)input[i];
        }

        return output;
    }

    static public byte[] CharToByteArray(char[] input, int length)
    {
        byte[] output = new byte[length];

        for (int i = 0; i < length; i++)
        {
            output[i] = (byte)input[i];
        }

        return output;
    }

    static public void QualityLevel_AdjustObjects()
    {
        MinQualityLevel[] allobjects = Resources.FindObjectsOfTypeAll<MinQualityLevel>();
        int level = QualitySettings.GetQualityLevel();

        foreach ( MinQualityLevel currobj in allobjects)
        {
            if( (currobj.quality > level) ||
                (currobj.maxquality >= 0) && (currobj.maxquality < level)
                )
            {
                currobj.gameObject.SetActive(false);
            }
            else
            {
                currobj.gameObject.SetActive(true);
            }

        }
    }

    static public void SetCameraQualityLevel(GameObject camera)
    {
        // Make sure there is a camera
        if (camera == null) { return; }

        return;

        // DISABLING Volume realtime brakes bake.

        int level = QualitySettings.GetQualityLevel();

        // Disable Volumetric Light for <= 1
        // Disable Post-Processing as well
        if (level <= 1)
        {
            if (camera.GetComponent<UnityEngine.Rendering.Volume>())
            {
                camera.GetComponent<UnityEngine.Rendering.Volume>().enabled = false;
            }
        }
        else if (level > 1)
        {
            // In this case turn off post processing, but turn on volumetric at minimal setting
            //if (camera.GetComponent<PostProcessingBehaviour>())
            //{
            //    camera.GetComponent<PostProcessingBehaviour>().enabled = false;
            //}

            // Make volumetric light quarter
            if (camera.GetComponent<UnityEngine.Rendering.Volume>())
            {
                camera.GetComponent<UnityEngine.Rendering.Volume>().enabled = true;
            }
        }


    }

    /************** START OF MD5/RSA ENCRYPTION **********************/

    // Returns true if the strings match, otherwise false
    static public bool IsMD5Match(string instring, string MD5)
    {
        // Extract the random number from the MD5
        string rand = MD5.Substring(MD5.Length - 3);

        try
        {
            string MD5_computed = EncryptMD5(instring, int.Parse(rand));
            if (MD5_computed == MD5) { return true; }
        }
        catch
        {
            return false;
        }

        return false;
    }


    // Create an MD5 hash code for the input string with adding random number and private code
    static public string EncryptMD5(string instring, int random_number = 0)
    {
        // To generate our code, we will add  private key and a random number. The random number will be given in plain site at the end of the string
        if (random_number == 0)
        {
            System.Random rand = new System.Random();
            random_number = rand.Next(100, 999);
        }

        string mycode = "EMPTY<TBD>";
        return Md5Sum16(mycode + instring + random_number) + random_number.ToString();
    }


    // Get MD5 Encryption code
    static public string Md5Sum16(string strToEncrypt) // Returns the first 16 chars of MD5
    {
        System.Text.UTF8Encoding ue = new System.Text.UTF8Encoding();
        byte[] bytes = ue.GetBytes(strToEncrypt);

        // encrypt bytes
        System.Security.Cryptography.MD5CryptoServiceProvider md5 = new System.Security.Cryptography.MD5CryptoServiceProvider();
        byte[] hashBytes = md5.ComputeHash(bytes);

        // Convert the encrypted bytes back to a string (base 16)
        string hashString = "";

        for (int i = 0; i < hashBytes.Length; i++)
        {
            hashString += System.Convert.ToString(hashBytes[i], 16).PadLeft(2, '0');
        }

        hashString.PadLeft(32, '0');
        return hashString.Substring(0, 16);
    }

    static public string CreateMD5(int red_score, int blue_score, string position, int rand)
    {
        return EncryptMD5("RED=" + red_score + "BLUE=" + blue_score + "POS=" + position, rand);
    }

    static public string FindMD5(string code, int rand, int max_num, out int red_score, out int blue_score, out string position)
    {
        string hash, substring;
        string[] positions = { "Red Left", "Red Center", "Red Right", "Blue Left", "Blue Center", "Blue Right" };

        foreach (string curr_pos in positions)
        {
            for (red_score = 0; red_score <= max_num; red_score++)
            {
                for (blue_score = 0; blue_score <= max_num; blue_score++)
                {
                    hash = CreateMD5(red_score, blue_score, curr_pos, rand);
                    substring = hash.Substring(0, code.Length);

                    if (substring == code)
                    {
                        position = curr_pos;
                        return hash;
                    }
                }
            }
        }

        red_score = 0;
        blue_score = 0;
        position = "None";
        return "NONE FOUND";
    }

    static public string DecryptRSA(string datain)
    {
        CspParameters cspp = new CspParameters();
        cspp.KeyContainerName = "xRCPublicKey";
        RSACryptoServiceProvider myRSA_Key = new RSACryptoServiceProvider(cspp);
        myRSA_Key.FromXmlString(GLOBALS.XRC_PUBLIC_KEY_XML);

        try
        {
            byte[] decrypted_data = myRSA_Key.Decrypt(Convert.FromBase64String(datain), false);
            string out_string = System.Text.Encoding.UTF8.GetString(decrypted_data, 0, decrypted_data.Length);

            Debug.Log("Decrytped string = " + out_string);
            return out_string;
        }
        catch (Exception e)
        {
            Debug.Log("Decryption failure: " + e);
            return "";
        }
    }

    public static string EncryptRSA(string data)
    {
        byte[] dataBytes = Encoding.UTF8.GetBytes(data);
        RSACryptoServiceProvider provider = CreateProviderFromKey(GLOBALS.XRC_PRIVATEPUBLIC_KEY_XML);
        byte[] signatureBytes = provider.SignData(dataBytes, "SHA1");
        return Convert.ToBase64String(signatureBytes);
    }

    public static bool RSAVerify(string data, string signature)
    {
        byte[] dataBytes = Encoding.UTF8.GetBytes(data);
        byte[] signatureBytes = Convert.FromBase64String(signature);
        RSACryptoServiceProvider provider = CreateProviderFromKey(GLOBALS.XRC_PUBLIC_KEY_XML);
        return provider.VerifyData(dataBytes, "SHA1", signatureBytes);
    }

    // Function to create the provider based on the key
    private static RSACryptoServiceProvider CreateProviderFromKey(string key)
    {
        RSACryptoServiceProvider provider = new RSACryptoServiceProvider();
        provider.FromXmlString(key);
        return provider;
    }

    static byte[] EncryptStringToBytes_Aes(string plainText, byte[] Key, byte[] IV)
    {
        // Check arguments.
        if (plainText == null || plainText.Length <= 0)
            throw new ArgumentNullException("plainText");
        if (Key == null || Key.Length <= 0)
            throw new ArgumentNullException("Key");
        if (IV == null || IV.Length <= 0)
            throw new ArgumentNullException("IV");
        byte[] encrypted;

        // Create an Aes object
        // with the specified key and IV.
        using (Aes aesAlg = Aes.Create())
        {
            aesAlg.Key = Key;
            aesAlg.IV = IV;

            // Create an encryptor to perform the stream transform.
            ICryptoTransform encryptor = aesAlg.CreateEncryptor(aesAlg.Key, aesAlg.IV);

            // Create the streams used for encryption.
            using (MemoryStream msEncrypt = new MemoryStream())
            {
                using (CryptoStream csEncrypt = new CryptoStream(msEncrypt, encryptor, CryptoStreamMode.Write))
                {
                    using (StreamWriter swEncrypt = new StreamWriter(csEncrypt))
                    {
                        //Write all data to the stream.
                        swEncrypt.Write(plainText);
                    }
                    encrypted = msEncrypt.ToArray();
                }
            }
        }

        // Return the encrypted bytes from the memory stream.
        return encrypted;
    }

    static string DecryptStringFromBytes_Aes(byte[] cipherText, byte[] Key, byte[] IV)
    {
        // Check arguments.
        if (cipherText == null || cipherText.Length <= 0)
            throw new ArgumentNullException("cipherText");
        if (Key == null || Key.Length <= 0)
            throw new ArgumentNullException("Key");
        if (IV == null || IV.Length <= 0)
            throw new ArgumentNullException("IV");

        // Declare the string used to hold
        // the decrypted text.
        string plaintext = null;
        AesCryptoServiceProvider myAes = new AesCryptoServiceProvider();
        byte[] randkey = myAes.Key;
        byte[] randIV = myAes.IV;

        // Create an Aes object
        // with the specified key and IV.
        using (Aes aesAlg = Aes.Create())
        {
            aesAlg.Key = Key;
            aesAlg.IV = IV;

            // Create a decryptor to perform the stream transform.
            ICryptoTransform decryptor = aesAlg.CreateDecryptor(aesAlg.Key, aesAlg.IV);

            // Create the streams used for decryption.
            using (MemoryStream msDecrypt = new MemoryStream(cipherText))
            {
                using (CryptoStream csDecrypt = new CryptoStream(msDecrypt, decryptor, CryptoStreamMode.Read))
                {
                    using (StreamReader srDecrypt = new StreamReader(csDecrypt))
                    {

                        // Read the decrypted bytes from the decrypting stream
                        // and place them in a string.
                        plaintext = srDecrypt.ReadToEnd();
                    }
                }
            }
        }

        return plaintext;
    }

    // Returns hex representation of byte array with space seperation
    public static string ByteArrayToHexString(byte[] ba)
    {
        return BitConverter.ToString(ba).Replace("-"," ");
    }

    // Convers a hex string with " " seperator to byte array
    public static byte[] HexStringToByteArray(String hexraw)
    {
        // Get rid of space
        String hex = hexraw;
        hex.Replace(" ", "");

        int NumberChars = hex.Length;
        byte[] bytes = new byte[NumberChars / 2];
        for (int i = 0; i < NumberChars; i += 2)
            bytes[i / 2] = Convert.ToByte(hex.Substring(i, 2), 16);
        return bytes;
    }

    static private byte[] GetKey()
    {
        byte[] outstring = (byte[]) GLOBALS.RANDOMIZER_SEED.Clone();
        int offset = 1;
        for( int i=0; i < outstring.Length; i++)
        {
            if( offset > 0)
            {
                outstring[i]++;
                offset = -1;
            }
            else
            {
                outstring[i]--;
                offset = 1;
            }
        }

        return outstring;
    }

    static public string EncryptAES(string indata)
    {
        // Create a 3 digit random number
        System.Random rand = new System.Random();
        string random_number = rand.Next(1000, 9999).ToString();

        byte[] myIV = System.Text.Encoding.ASCII.GetBytes( random_number + random_number + random_number + random_number);


        byte[] outdata = EncryptStringToBytes_Aes(indata, GetKey(), myIV);


        string outstring = ByteArrayToHexString(outdata);
        outstring += " " + random_number.Substring(0, 2) + " " + random_number.Substring(2, 2);

        return outstring;
    }

    static public string DecryptAES(string indata_raw)
    {
        // Data coming in as ## ## ## ## where each ## pair represents a char
        string indata = indata_raw.Replace(" ", ""); // Take out space seperator
        string random_number = indata.Substring(indata.Length - 4, 4); // extract 4-char seed
        string data = indata.Substring(0, indata.Length - 4); // extract data

        // Generate IV from seed
        byte[] myIV = System.Text.Encoding.ASCII.GetBytes(random_number + random_number + random_number + random_number);

        return DecryptStringFromBytes_Aes(HexStringToByteArray(data), GetKey(), myIV);
    }
    //************** END OF ENCRYPTION **********************



    static public double AngleWrap(double degrees_in)
    {
        return degrees_in - 360f * Math.Floor(degrees_in / 360f + 0.5f);
    }
    static public float AngleWrap(float degrees_in)
    {
        return (float) (degrees_in - 360f * Math.Floor(degrees_in / 360f + 0.5f));
    }

    // Returns the 2-D clockwise angle in degrees between 2 vectors, need normal to the plave
    static public float AngleBetweenVectors(Vector3 a, Vector3 b, Vector3 normal)
    {
        // angle in [0,180]
        float angle = Vector3.Angle(a, b);
        float sign = Mathf.Sign(Vector3.Dot(normal, Vector3.Cross(a, b)));

        // angle in [-179,180]
        float signed_angle = angle * sign;

        // angle in [0,360] (not used but included here for completeness)
        //float angle360 =  (signed_angle + 180) % 360;

        return signed_angle;
    }

    // String aid functions
    static public string BoolToString(bool data)
    {
        return ((data) ? "1" : "0");
    }

    static public bool StringToBool(string data)
    {
        return (data[0] == '1') || (data[0] == 'T') || (data[0] == 't');
    }

    static public bool SpanToBool( ReadOnlySpan<char> data, int index)
    {
        return (data[index+1] == '1') || (data[index+1] == 'T') || (data[index+1] == 't');
    }


    // Instantiate a robot model with the skins specified
    private static Vector3 ftc_pos = new Vector3(-0.25f, 0, 0);

    // Instantiates Robot
    public static GameObject InstantiateRobot(string model, Vector3 start_pos, Quaternion start_rot, string skins = "0", string robot_skin = "")
    {
        // Get the prefab
        GameObject prefab = Resources.Load("Robots/" + model) as GameObject;
        if (!prefab) { return null; }

        // Instantiate it 
        GameObject instance = UnityEngine.Object.Instantiate(prefab, start_pos, start_rot);
        if (!instance) { return null; }

        // Get the robot_skin
        if (robot_skin.Length < 1)
        {
            robot_skin = "Real";
        }
// DEBUG: TAKEOUT LATER
        if( GLOBALS.FORCE_REAL)
        {
            robot_skin = "Real";
        }
        GameObject robotskin_go = Resources.Load("Robots/Skins/" + model + "/" + robot_skin) as GameObject;
        if (robotskin_go)
        {
            GameObject instanceskin = UnityEngine.Object.Instantiate(robotskin_go, start_pos, start_rot);
            if (instanceskin) {
               // instanceskin.transform.SetParent(instance.transform);
                MergeRobotSkin(instance.transform, instanceskin.transform);

                // delete the remains 
                UnityEngine.Object.Destroy(instanceskin);

                // If robot skin script(s) exist, then initialize it/them
                RobotSkin[] rs_scripts = instance.GetComponentsInChildren<RobotSkin>();

                foreach (RobotSkin currscript in rs_scripts)
                {
                    currscript.InitSkin();
                }

            }

        }


        // Get the body component
        Transform body = instance.transform.Find("Body");
        if (!body) { return instance; }

        // Get the robot interface
        RobotInterface3D ri3d = instance.GetComponent<RobotInterface3D>();
        if (!ri3d) { return instance; }

        // Add any skins
        string[] skin_chosen = skins.Split(GLOBALS.SKIN_SEPERATOR);

        if ((skin_chosen.Length > 0) && (skin_chosen[0].Length >= 1)) // eyes 
        {
            // Instantiate object
            int index = int.Parse(skin_chosen[0]);
            if (index > 0)
            {
                // Instantiate it 
                if (!GLOBALS.skins_eyes.ContainsKey(index))
                {
                    return instance;
                }
                GameObject skin = UnityEngine.Object.Instantiate(GLOBALS.skins_eyes[index], start_pos, start_rot);
                if (!skin) { return instance; }

                skin.transform.SetParent(body);
                skin.transform.localPosition = (ri3d.is_FRC) ? Vector3.zero : ftc_pos;
                skin.transform.localRotation = Quaternion.identity;
            }
        }

        if (skin_chosen.Length > 1) // hat
        {
            // Instantiate object
            int index = int.Parse(skin_chosen[1]);
            if (index > 0)
            {
                // Instantiate it 
                if (!GLOBALS.skins_hats.ContainsKey(index))
                {
                    return instance;
                }
                GameObject skin = UnityEngine.Object.Instantiate(GLOBALS.skins_hats[index], start_pos, start_rot);
                if (!skin) { return instance; }

                skin.transform.SetParent(body);
                skin.transform.localPosition = (ri3d.is_FRC) ? Vector3.zero : ftc_pos;
                skin.transform.localRotation = Quaternion.identity;
            }
        }

        if (skin_chosen.Length > 2)
        {
            // Instantiate object
            int index = int.Parse(skin_chosen[2]);
            if (index > 0)
            {
                // Instantiate it 
                if (!GLOBALS.skins_spoilers.ContainsKey(index))
                {
                    return instance;
                }
                GameObject skin = UnityEngine.Object.Instantiate(GLOBALS.skins_spoilers[index], start_pos, start_rot);
                if (!skin) { return instance; }

                skin.transform.SetParent(body);
                skin.transform.localPosition = Vector3.zero;
                skin.transform.localRotation = Quaternion.identity;
            }
        }

        if (skin_chosen.Length > 3)
        {
            // Instantiate object
            int index = int.Parse(skin_chosen[3]);
            if (index > 0)
            {
                // Instantiate it 
                if (!GLOBALS.skins_other.ContainsKey(index))
                {
                    return instance;
                }
                GameObject skin = UnityEngine.Object.Instantiate(GLOBALS.skins_other[index], start_pos, start_rot);
                if (!skin) { return instance; }

                skin.transform.SetParent(body);
                skin.transform.localPosition = Vector3.zero;
                skin.transform.localRotation = Quaternion.identity;
            }
        }

        return instance;

    }

    // ********************** Scoring File Processing **********************
    static long time_of_last_status = 0;
    static public string status_file_dir = Path.DirectorySeparatorChar.ToString() + "temp" + Path.DirectorySeparatorChar.ToString() + "xRCsim";

    static public Dictionary<string, statusfile> status_files = new Dictionary<string, statusfile>();
    static public Dictionary<string, string> score_details = new Dictionary<string, string>(); // the extracted score details
    static public Dictionary<string, statusfile> auto_files = new Dictionary<string, statusfile>();


    public static void DoScoringFiles(Dictionary<string, string> serverFlags)
    {
        // Make sure sufficient time has elapsed
        if ((GetTimeMillis() - time_of_last_status) < GLOBALS.STATUS_FILES_UPDATE_PERIOD)
        {
            return;
        }

        time_of_last_status = GetTimeMillis();

        // Clear old data
        score_details.Clear();

        // Exit if the SCORE key doesn't exist
        if (!serverFlags.ContainsKey("SCORE")) { return; }

        // Break the score apart
        string[] all_data = serverFlags["SCORE"].Split(';');
        if (all_data.Length < 1) { return; }

        // Now go through all the data
        for (int i = 0; i < all_data.Length; i++)
        {
            string[] key_value = all_data[i].Split('=');
            if (key_value.Length != 2) { continue; }  // Make sure there are 2 items: Key = Value

            score_details[key_value[0]] = key_value[1];

            // Make sure the output files are enabled before creating them
            if (!GLOBALS.OUTPUT_SCORING_FILES)
            { continue; }

            // Now create the file if it doesn't exist
            if (!status_files.ContainsKey(key_value[0]))
            {
                // Make sure directory exists
                if (!Directory.Exists(status_file_dir))
                {
                    try
                    {
                        Directory.CreateDirectory(status_file_dir);
                    }
                    catch (Exception e)
                    {
                        // Unable to create directory
                        continue;
                    }
                }

                // Create the stream
                statusfile new_status = new statusfile();
                new_status.fileinfo = new FileInfo(status_file_dir + Path.DirectorySeparatorChar.ToString() + key_value[0] + ".txt");
                // Make sure it doesn't exists
                new_status.fileinfo.Delete();

                // Now create it
                new_status.stream = new_status.fileinfo.Open(FileMode.Create, FileAccess.ReadWrite, FileShare.Read);
                new_status.value = "";
                status_files[key_value[0]] = new_status;
            }

            statusfile curr_status = status_files[key_value[0]];

            // If the file stream is null, skip this
            if (curr_status.stream == null) { continue; }  // Unable to create file??

            // If the file hasn't changed, skip it
            if (curr_status.value == key_value[1]) { continue; }
            curr_status.value = key_value[1];

            // Write to the file
            curr_status.stream.SetLength(0); // Clear old data
            curr_status.stream.Write(Encoding.GetEncoding("UTF-8").GetBytes(key_value[1].ToCharArray()), 0, key_value[1].Length);
            curr_status.stream.FlushAsync();
        }

    }

    // Output auto files
    // Input: string:string dictionary = <filename> : <data string to output>
    public static void DoAutoFiles(Dictionary<string, string> alldatahash)
    {
        // Only do automation files if enabled
        if( !GLOBALS.AUTOMATION_FILES) { return; }

        foreach (string currfile in alldatahash.Keys)
        {
            // Now create the file if it doesn't exist
            if (!auto_files.ContainsKey(currfile))
            {
                // Make sure directory exists
                if (!Directory.Exists(GLOBALS.AUTOMATION_DIR))
                {
                    try
                    {
                        Directory.CreateDirectory(GLOBALS.AUTOMATION_DIR);
                    }
                    catch (Exception e)
                    {
                        // Unable to create directory
                        continue;
                    }
                }

                // Create the stream
                statusfile new_autofile = new statusfile();
                new_autofile.fileinfo = new FileInfo(GLOBALS.AUTOMATION_DIR + Path.DirectorySeparatorChar.ToString() + currfile + ".txt");
                // Make sure it doesn't exists
                new_autofile.fileinfo.Delete();

                // Now create it
                new_autofile.stream = new_autofile.fileinfo.Open(FileMode.Create, FileAccess.ReadWrite, FileShare.Read);
                new_autofile.value = "";
                auto_files[currfile] = new_autofile;
            }

            statusfile curr_status = auto_files[currfile];

            // If the file stream is null, skip this
            if (curr_status.stream == null) { continue; }  // Unable to create file??

            // Write to the file
            curr_status.stream.SetLength(0); // Clear old data
            curr_status.stream.Write(Encoding.GetEncoding("UTF-8").GetBytes(alldatahash[currfile].ToCharArray()), 0, alldatahash[currfile].Length);
            curr_status.stream.FlushAsync();
        }

    }

    public static void ResetAutoFiles()
    {
        // Close existing files
        foreach( statusfile currfile in auto_files.Values)
        {
            if( currfile!=null && (currfile.stream != null))
            {
                currfile.stream.Close();
            }
        }
        auto_files.Clear();
    }

    // Checks for "execute.txt" which feeds commands back to this program
    public static Dictionary<string,string> GetAutoControls()
    {
        Dictionary<string, string> readincontrols = new Dictionary<string, string>();

        // Make sure directory exists
        if (!Directory.Exists(GLOBALS.AUTOMATION_DIR))
        {
            return readincontrols;
        }

        string file_name = GLOBALS.AUTOMATION_DIR + Path.DirectorySeparatorChar.ToString() + "Controls.txt";

        // Check if a command file is ready for us
        if (!File.Exists(file_name))
        {
            return readincontrols;
        }

        // Try opening it, exit if the file is already open by someone else
        FileStream controls_file;
        FileInfo file_to_open = new FileInfo(file_name);

        try
        {
            controls_file = file_to_open.Open(FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
        }
        catch (IOException)
        {
            //the file is unavailable because it is:
            //still being written to
            //or being processed by another thread
            //or does not exist (has already been processed)
            return readincontrols;
        }

        System.IO.StreamReader file_reader = new System.IO.StreamReader(controls_file);

        List<string> outstring = new List<string>();

        string currline = file_reader.ReadLine();
        while (currline != null)
        {
            outstring.Add(currline);
            currline = file_reader.ReadLine();
        }

        // See if there is a last line without a return
        currline = file_reader.ReadToEnd();
        if (currline.Length > 0)
        {
            outstring.Add(currline);
        }

        file_reader.Close();

        // Parse the data into dictionary
        foreach( string line in outstring)
        {
            //If begins with /, skip
            if( line.StartsWith("/")) { continue; }

            string[] splitdata = line.Split('=');

            // Must have exactly 2 entries
            if(splitdata.Length != 2) { continue; }

            // Key and value must have at least 1 char
            if(splitdata[0].Length < 1 || splitdata[1].Length < 1)  { continue; }

            readincontrols[splitdata[0]] = splitdata[1];
        }


        return readincontrols;
    }



    static private long time_of_last_command = -1;
    // Checks for "execute.txt" which feeds commands back to this program
    public static List<string> GetScoringFilesCommand()
    {
        // Make sure sufficient time has elapsed
        if ((GetTimeMillis() - time_of_last_command) < 75)
        {
            return null;
        }

        time_of_last_command = GetTimeMillis();

        // Make sure we enabled scoring files
        // Make sure the output files are enabled before creating them
        if (!GLOBALS.OUTPUT_SCORING_FILES)
        { return null; }

        // Make sure directory exists
        if (!Directory.Exists(status_file_dir))
        {
            return null;
        }

        string file_name = status_file_dir + Path.DirectorySeparatorChar.ToString() + "execute.txt";

        // Check if a command file is ready for us
        if (!File.Exists(file_name))
        {
            return null;
        }

        // Try opening it, exit if the file is already open by someone else
        FileStream execute_file;
        FileInfo file_to_open = new FileInfo(file_name);

        try
        {
            execute_file = file_to_open.Open(FileMode.Open, FileAccess.Read, FileShare.None);
        }
        catch (IOException)
        {
            //the file is unavailable because it is:
            //still being written to
            //or being processed by another thread
            //or does not exist (has already been processed)
            return null;
        }

        System.IO.StreamReader file_reader = new System.IO.StreamReader(execute_file);

        List<string> outstring = new List<string>();

        string currline = file_reader.ReadLine();
        while ( currline != null)
        {
            outstring.Add(currline);
            currline = file_reader.ReadLine();
        }

        // See if there is a last line without a return
        currline = file_reader.ReadToEnd();
        if( currline.Length > 0)
        {
            outstring.Add(currline);
        }

        file_reader.Close();
        File.Delete(file_name); // Delete the file

        return outstring;
    }



    // Return the scorefile keys/values as requested below
        static public string GetRedAdj()
    {
        if (score_details.ContainsKey("RedADJ"))
        {
            return score_details["RedADJ"];
        }

        return "0";
    }

    static public string GetBlueAdj()
    {
        if (score_details.ContainsKey("BlueADJ"))
        {
            return score_details["BlueADJ"];
        }

        return "0";
    }

    static public void CloseScorefiles()
    {
        // Close all the files
        foreach (statusfile currfile in status_files.Values)
        {
            currfile.stream.Close();
        }

        // Clear the status_files list
        status_files.Clear();
    }

    static public string GetFullName(Transform obj, Transform parent = null)
    {
        string name = obj.name;
        while ((obj.parent != null) && (obj.parent != parent))
        {

            obj = obj.parent;
            name = obj.name + "/" + name;
        }
        return name;
    }

    #region Math
    public static Vector3 GetPitchYawRollRad(Quaternion rotation)
    {
        float roll = Mathf.Atan2(2 * rotation.y * rotation.w - 2 * rotation.x * rotation.z, 1 - 2 * rotation.y * rotation.y - 2 * rotation.z * rotation.z);
        float pitch = Mathf.Atan2(2 * rotation.x * rotation.w - 2 * rotation.y * rotation.z, 1 - 2 * rotation.x * rotation.x - 2 * rotation.z * rotation.z);
        float yaw = Mathf.Asin(2 * rotation.x * rotation.y + 2 * rotation.z * rotation.w);

        return new Vector3(pitch, roll, yaw);
    }
    public static Vector3 GetPitchYawRollDeg(Quaternion rotation)
    {
        Vector3 radResult = MyUtils.GetPitchYawRollRad(rotation);
        return new Vector3(radResult.x * Mathf.Rad2Deg, radResult.y * Mathf.Rad2Deg, radResult.z * Mathf.Rad2Deg);
    }
    #endregion

    // Merge a transform hierarhcy into another
    // Designed ONLY for robot merging, special care and intention is to deal with renders 
    public static void MergeRobotSkin(Transform merged, Transform tomerge)
    {

        // *********************************************************
        // Go through all to-merge objects and merge them in as required
        //
        // If a top leaf doesn't exist, then re-parent it to the merged_obj
        // If it does exist, then copy over all components
        // This means we need to traverse the tree top-down
        List<Transform> objectstoinspect = new List<Transform>();

        // Seed the first level children
        for (int i = 0; i < tomerge.childCount; i++)
        {
            objectstoinspect.Add(tomerge.GetChild(i));
        }

        // Now iterate through all the children top-down: add more children if they exist, otherwise keep going;
        for (int i = 0; i < objectstoinspect.Count; i++)
        {
            Transform currobj = objectstoinspect[i];
            Transform curr_merged = merged.Find(GetFullName(currobj, tomerge));

            // Debug.Log("Processing " + MyUtils.GetFullName(currobj, tomerge) + " to go inside " + ((curr_merged == null) ? "null" : curr_merged.name));

            // If the objects doesn't exist in the merged_obj, then just re-parent it
            if (curr_merged == null)
            {
                // Get the merged objects equivalent parent
                Transform newparent = merged.Find(GetFullName(currobj.parent, tomerge));

                if (newparent == null)
                {
                    // Debug.Log("  Parent not found?! Assuming top is parent");
                    currobj.parent = merged;
                }
                else
                {
                    // Debug.Log("  Parent = " + newparent.name);
                    currobj.parent = newparent;
                }

                continue;
            }

            // However we need "special" treatment of colliders: IF they exist, assume the only thing we need to do is enable the Renderer
            // This is a fix to how standard robot is designed (before multi-skins was considered)
            // First check if Collider exists (in tobemerged - that's where we store this information)
            // ALSO, textmeshpro don't copy, so adding a collider to prevent copying (just enabling) is also used
            Collider collider_exists = currobj.GetComponent<Collider>();
            if (collider_exists != null)
            {
                // Turn on the renderer
                Renderer merged_renderer = curr_merged.GetComponent<Renderer>();
                if (merged_renderer != null)
                {
                    merged_renderer.enabled = true;
                }
            }
            else
            {
                CopyAllComponents(currobj, curr_merged);
            }

            // Finally add it's children to iteration loop
            for (int j = 0; j < currobj.childCount; j++)
            {
                objectstoinspect.Add(currobj.GetChild(j));
            }
        }

        // Finaly copy over the very top parent
        CopyAllComponents(tomerge, merged);
    }

    static public void CopyAllComponents(Transform source, Transform target)
    {
        // Copy all the components 
        Component[] found_components = source.GetComponents<Component>();

        for (int j = 0; j < found_components.Length; j++)
        {
            Component currcomp = found_components[j];
            System.Type type = currcomp.GetType();
            System.Reflection.FieldInfo[] fields = type.GetFields();
            var props = type.GetProperties();

            // if (currcomp.GetType() == typeof(Transform)) { continue; } // Leave transforms alone
            // if (currcomp.GetType() == typeof(RectTransform)) { continue; } // Same


            Component curr_copy = target.gameObject.GetComponent(currcomp.GetType());

            // If it already exists, just enable it
            // Need to do that since re-creating it with the 2021.3.10f1 unity this fails to be proper in meshes. It creates the mesh, 
            // all properties look good, but it won't actually show up.
            if (curr_copy) // If it already exists??? Try deleting it?? Probably won't work
            {
                int k = 0;
                for (; k < props.Length; k++)
                {
                    if(props[k].Name == "enabled")
                    {
                        break;
                    }
                }

                // If we found it.. update it
                if (k < props.Length)
                {
                    if (props[k].GetValue(currcomp) != props[k].GetValue(curr_copy))
                    {
                        props[k].SetValue(curr_copy, props[k].GetValue(currcomp));
                        continue;
                    }
                }

            }
            else
            {
                curr_copy = target.gameObject.AddComponent(currcomp.GetType());
            }

            // Copy over values using reflection API

            foreach (System.Reflection.FieldInfo field in fields)
            {
                // Make sure its writable/non constnat
                if (field.IsLiteral) { continue; }

                field.SetValue(curr_copy, field.GetValue(currcomp));
            }

            // Copy over properties too            
            foreach (var prop in props)
            {
                if (!prop.CanWrite || !prop.CanRead ||
                    prop.Name == "name" ||
                    prop.Name == "parent" ||
                    prop.Name == "hierarchyCapacity") continue;

                prop.SetValue(curr_copy, prop.GetValue(currcomp, null), null);
            }

        }
    }

    // Finds a child by name in hierarchy
    static public Transform FindHierarchy(Transform parent, string name)
    {
        Transform[] trs = parent.GetComponentsInChildren<Transform>(true);
        foreach (Transform t in trs)
        {
            if (t.name == name)
            {
                return t;
            }
        }
        return null;

        /*
        foreach (Transform child in parent)
        {
            // If this is our child, return it
            if (child.name == name)
                return child;

            // Otherwise go deeper
            var result = FindHierarchy(child, name);
            if (result != null)
                return result;
        }

        return null;
        */
    }

    // Returns the gameobject of name given
    // Finds inactive objects too
    static public GameObject FindGlobal(string name)
    {
        GameObject[] all_objects = Resources.FindObjectsOfTypeAll(typeof(GameObject)) as GameObject[];

        foreach (GameObject curr in all_objects)
        {
            if (curr.name == name)
            {
                return curr;
            }
        }

        return null;
    }

    // Logging messages
    public static void LogMessageToFile(string msg, bool isError = true)
    {
        // If logging turned off, exit
        if (GLOBALS.ENABLE_LOGS == false)
        { return; }

        string logLine = System.String.Format(
                   "{0:G}: {1}.", System.DateTime.Now, msg);


        if (isError)
        {
            Debug.LogError(logLine);
        }
        else
        {
            Console.WriteLine(logLine);
        }
        return;
    }

    // **********************
    // Functions relating to recording and playing data

    static public List<Saved_Data> recorded_data = new List<Saved_Data>();
    static public int playback_index = 0;
    static public long playback_offset = -1;
    static public int playback_extra_frames = 0;
    static public int time_offset = 0;

    public static void PB_RecordData(string[] data, int time, bool loading_file = false)
    {
        // Don't record if not recording - except if loading a file
        if (!loading_file && !GLOBALS.now_recording) { return; }
        Saved_Data newdata = new Saved_Data();
        newdata.timestamp = time + time_offset;
        newdata.data = data;

        // First piece of data just record without checking
        if(recorded_data.Count <= 0)
        {
            recorded_data.Add(newdata);
            return;
        }


        // Check if we need to increase time_offset
        // If data point is more then half-a-day behind the last data point, then we have wrapped around
        while((newdata.timestamp - recorded_data[recorded_data.Count - 1].timestamp) < -43200000 )
        {
            time_offset += 86400000;
            newdata.timestamp += 86400000;
        }

        // If we have wrapped around and the data point coming in is before the wrap around, then correct it
        while ((newdata.timestamp - recorded_data[recorded_data.Count - 1].timestamp) > 43200000)
        {
            newdata.timestamp -= 86400000;
        }


        // Now add the data point in the correct position
        int index = recorded_data.Count;

        // Add the data in the correct spot
        // Almost all the time it will be at the end of the list
        // but if packets arrive in incorrect order, this will fix it
        while ((index > 0) && (recorded_data[index-1].timestamp > newdata.timestamp))
        {
            index--;
        }

        recorded_data.Insert(index, newdata);

        // Apply buffer limit if desired
        if (!loading_file)
        {
            // If the time difference between first+last element is > time-limit, then release data
            while (recorded_data[recorded_data.Count - 1].timestamp - recorded_data[0].timestamp > GLOBALS.PB_BUFFER_DURATION * 60 * 1000)
            {
                recorded_data.RemoveAt(0);
            }
        }
    }

    // Gets the last elements timestamp
    public static long PB_GetStartTime()
    {
        if (recorded_data.Count < 1) { return -1; }
        return recorded_data[0].timestamp;
    }

    // Gets the first element's timestamp
    public static long PB_GetEndTime()
    {
        if (recorded_data.Count < 1) { return -1; }
        return recorded_data[recorded_data.Count - 1].timestamp;
    }

    public static long PB_GetCurrentTime()
    {
        if( PB_ReachedEnd() ) { return PB_GetEndTime(); }

        return recorded_data[playback_index].timestamp;
    }

    // Iterate through the list
    public static bool PB_StartPlayback(long start_time)
    {
        // Increment index to our starting location
        playback_index = -1;

        while ((++playback_index < recorded_data.Count) && (recorded_data[playback_index].timestamp < start_time))
        {
        }

        if( PB_ReachedEnd() ) { return false; }

        playback_offset = (long) (((float) GetTimeMillisSinceStart()) - ((float) start_time)/ GLOBALS.playback_speed);
        return true;
    }

    static public Saved_Data PB_GetNext()
    {
        // If playback wasn't started, return null
        if (playback_index < 0) { return null; }

        // If we reached the end of the rope, return null
        if (playback_index >= recorded_data.Count) { return null; }

        long time_to_stop = (long) ( ((float) (GetTimeMillisSinceStart() - playback_offset)) * GLOBALS.playback_speed);

        while( playback_extra_frames > 0)
        {
            int curr_index = playback_index - playback_extra_frames--;
            if(curr_index < 0 ) { continue; }

            return recorded_data[curr_index];
        }

        if (recorded_data[playback_index].timestamp < time_to_stop)
        {
            return recorded_data[playback_index++];
        }

        return null;
    }

    // Return true if playback wasn't started end/or we reached the end
    static public bool PB_ReachedEnd()
    {
        if( (playback_index < 0) || (playback_index >= recorded_data.Count) )
        {
            return true;
        }

        return false;
    }

    // Should be called when recording first starts
    public static void PB_ClearRecording()
    {
        recorded_data.Clear();
        playback_index = -1;
        playback_offset = -1;
        time_offset = 0;
    }

    public static void PB_UpdateDuringPause()
    {
        if ((playback_index < 0) || (playback_index >= recorded_data.Count))
        {
            return;
        }

        // Update offset to keep index the current value
        playback_offset = (long) (((float) GetTimeMillisSinceStart()) - ((float) recorded_data[playback_index].timestamp)/GLOBALS.playback_speed);
    }

    public static bool PB_ChangeTime(long start_time)
    {
        // Increment index to our starting location
        int new_playback_index = -1;

        while ((++new_playback_index < recorded_data.Count) && (recorded_data[new_playback_index].timestamp < start_time))
        {
        }

        playback_index = new_playback_index;
        playback_extra_frames = 20;
        playback_offset = (long) (((float) GetTimeMillisSinceStart()) - ((float)start_time) /GLOBALS.playback_speed);
        return true;
    }

    public static void PB_ChangeSpeed(float speed)
    {
        GLOBALS.playback_speed = speed;
        PB_UpdateDuringPause();
    }

    public static int GetEndRecordingOffset()
    {
        // Make sure there is a client
        if( !GLOBALS.topclient) { return 0; }

        // Search for a game end marker. If none found then return 0
        for (int i = recorded_data.Count - 1; i >= 0; i--)
        {
            if(GLOBALS.topclient.IsPlaybackDataEndOfMatch(recorded_data[i]))
            {
                return recorded_data.Count - i + 1;
            }
        }

        return 0;
    }

    public static int GetStartRecordingOffset(int end_offset)
    {
        // Make sure there is a client
        if (!GLOBALS.topclient) { return 0; }

        // Search for a game end marker. If none found then return 0
        for (int i = recorded_data.Count - end_offset - 1; i >= 0; i--)
        {
            if (GLOBALS.topclient.IsPlaybackDataStartOfMatch(recorded_data[i]))
            {
                return i;
            }
        }

        return 0;
    }

    // Call to record the current buffer to the file from the last end-match to its associated beggining.
    static private bool autosave_inprogress = false;
    static private int save_maxtime = 20; // Max number of milliseconds to save for in 1 frame
    static private Stream save_outputfile;
    static private List<Saved_Data> save_recorded_data;
    static private int save_position = 0;

    // Returns true if autosave didn't finish
    public static bool PB_AutoSaveInProgress()
    {
        return autosave_inprogress;
    }


    // Returns a number from 0 to 1 that indicates how close to done we are in the file save progress
    public static float PB_AutoSavePercentage()
    {
        if( autosave_inprogress)
        {
            return ((float) save_position) / ((float) save_recorded_data.Count);
        }

        return 1f;
    }
    public static bool PB_AutoSaveToFile()
    {
        // If we are in the middle of autosave, then continue it
        if( autosave_inprogress )
        {
            // Continue writing to file. PB_SaveToFile will use "save_position" instead of its counter
            PB_SaveToFile("", save_position, 0, true, save_recorded_data);

            // See if we are finished 
            if(!autosave_inprogress)
            {
                // Clear data and exit
                save_recorded_data.Clear();
                save_recorded_data = null;
                save_outputfile = null;
                save_position = 0;
            }

            return true;
        }

        // See if autosave is turned on
        if( !GLOBALS.autosave_recordings) { return false; }

        // See if we are activelly recording
        if( !GLOBALS.now_recording) { return false; }

        // Make sure filename is set
        if( GLOBALS.autosave_filename.Length < 1) { return false; }

        // Find the next available filename
        // Limit to 999 recordings
        int i = 1;
        string save_file = "";

        for ( ; i <= 999; i++)
        {
            save_file = GLOBALS.autosave_filename;
            if ( save_file.EndsWith(".xrc"))
            {
                save_file = save_file.Remove(save_file.Length - 4);
                save_file += i + ".xrc";
            }

            if (!File.Exists(save_file))
            { break; }
        }

        // Check if we reached end of all files
        if( i > 999 ) {
            LogMessageToFile("Unable to autosave to file " + GLOBALS.autosave_filename + ". Reached 999 limit on filename ending.", true);
            return false; 
        }

        // Get the start and end of the recording
        int end_offset = GetEndRecordingOffset();
        int start_offset = GetStartRecordingOffset(end_offset);

        // If start and end of recording allow us to rewind/forward a little bit, then lets do it
        if( end_offset > 10) { end_offset -= 10; }
        else { end_offset = 0; }

        if( start_offset > 10) { start_offset -= 10; }
        else { start_offset = 0; }

        // Copy data in memory that needs to be saved
        save_recorded_data = new List<Saved_Data>();
        for( i = start_offset; i < recorded_data.Count - end_offset; i++)
        {
            Saved_Data currdata = new Saved_Data();
            currdata.timestamp = recorded_data[i].timestamp;
            currdata.data = recorded_data[i].data;
            save_recorded_data.Add(currdata);
        }

        // Now save the file
        return PB_SaveToFile(save_file, 0, 0, true, save_recorded_data);
    }


    // PB_SaveToFile
    // Incremental = continue with
    // my_recorded_data: if specified, will point to data buffer, otherwise will use the current recorded_data buffer
    public static bool PB_SaveToFile(string filename, int start = 0, int stop=0, bool incremental = false, List<Saved_Data> my_recorded_data = null)
    {
        // Fix my_recorded_data if required
        if( my_recorded_data == null)
        {
            my_recorded_data = recorded_data;
        }

        // The filestream to follow
        FileStream outputfile;
        Stream outstream;
        string compression_type = "";

        // If this is an incremental write, skip all the things at the beggining
        // if we haven't initialized it yet

        // Create file and initialize it
        if (!incremental || !autosave_inprogress)
        {
            FileInfo outputfile_info = new FileInfo(filename);
            // try opening the file
            try
            {
                outputfile = outputfile_info.Open(FileMode.Create, FileAccess.Write);

                // Write version info to file
                outputfile.Write(Encoding.GetEncoding("UTF-8").GetBytes(GLOBALS.VERSION), 0, Encoding.GetEncoding("UTF-8").GetByteCount(GLOBALS.VERSION));
                outputfile.WriteByte((byte)GLOBALS.DATAFILE_SEPARATOR1);

                // Write game number 
                outputfile.Write(Encoding.GetEncoding("UTF-8").GetBytes(GLOBALS.GAME_INDEX.ToString()), 0, Encoding.GetEncoding("UTF-8").GetByteCount(GLOBALS.GAME_INDEX.ToString()));
                outputfile.WriteByte((byte)GLOBALS.DATAFILE_SEPARATOR1);

                // Add compresison index
                compression_type = "2"; // 0 = no compression (UTF-8 encoding), 1 = Zlib default string compression.
                outputfile.Write(Encoding.GetEncoding("UTF-8").GetBytes(compression_type), 0, Encoding.GetEncoding("UTF-8").GetByteCount(compression_type));
                outputfile.WriteByte((byte)GLOBALS.DATAFILE_SEPARATOR1);
            }
            catch (Exception e)
            {
                LogMessageToFile("Open file failed " + e, true);
                return false;
            }

            // Create the proper type of compression stream

            if (compression_type == "3") // gzip
            {
                outstream = new GZipStream(outputfile, CompressionMode.Compress, Ionic.Zlib.CompressionLevel.Level2);
            }
            if (compression_type == "2") // gzip
            {
                outstream = new GZipStream(outputfile, CompressionMode.Compress, Ionic.Zlib.CompressionLevel.Level5);
            }
            else if (compression_type == "1") //ZLib.. really ends up being gzip but level-6 default
            {
                outstream = new ZlibStream(outputfile, CompressionMode.Compress);
            }
            else
            {
                outstream = outputfile;
            }
        }
        else
        {
            outstream = save_outputfile;
        }

        // Now keep writing to the file
        long start_time = GetTimeMillis();

        try
        {
            string outstring;

            // Next go through all the data and record it to the file   
            for ( int i =start; i<my_recorded_data.Count-stop; i++)
            {
                // Check if we should stop
                if (incremental && (GetTimeMillis() - start_time) > save_maxtime)
                {
                    save_position = i;
                    save_outputfile = outstream;
                    autosave_inprogress = true;
                    return true;
                }
                
                // Timestamp
                outstring = my_recorded_data[i].timestamp.ToString();
                outstring += GLOBALS.DATAFILE_SEPARATOR1;

                //outputfile.Write(Encoding.GetEncoding("UTF-8").GetBytes(outstring), 0, Encoding.GetEncoding("UTF-8").GetByteCount(outstring));
                //outputfile.WriteByte((byte)GLOBALS.DATAFILE_SEPARATOR1);

                // Data
                outstring += String.Join(GLOBALS.DATAFILE_SEPARATOR2.ToString(), my_recorded_data[i].data);
                byte[] encoded_data = Encoding.GetEncoding("UTF-8").GetBytes(outstring);             
                byte[] length = Encoding.GetEncoding("UTF-8").GetBytes(encoded_data.Length.ToString());
                outstream.Write(length,0,length.Length);
                outstream.WriteByte((byte)GLOBALS.DATAFILE_SEPARATOR1);
                outstream.Write(encoded_data, 0, encoded_data.Length);
            }

            // close the file
            outstream.Close();
            // outputfile.Close(); // Do I still need to do this???

            // Clear saved variables if present
            if (incremental )
            {
                save_position = 0;
                save_outputfile = null;
                autosave_inprogress = false;
            }
        }
        catch (Exception e)
        {
            LogMessageToFile("Open/write file failed " + e, true);
            return false;
        }

        return true;
    }

    public static string PB_GetString(Stream inputfile)
    {
        // Read in 100 bytes at a time
        byte[] bytebuffer = new byte[100];
        char[] charbuffer = new char[250]; // larger char decoder just in case. in reality it will be same or smaller
        int bytecount = 100;
        String outstring = "";

        // And then use the decoder on it
        Decoder utfdecoder = Encoding.GetEncoding("UTF-8").GetDecoder();

        // While we still could read in all the bytes
        while (bytecount >= 100)
        {
            long currpos = inputfile.Position;

            // Read in 100 bytes
            bytecount = inputfile.Read(bytebuffer, 0, 100);

            // Change it to chars
            int charcount = utfdecoder.GetChars(bytebuffer, 0, bytecount, charbuffer, 0);
            
            // Get seperator
            for(int i =0; i < charcount; i++)
            {
                // If we found our seperator, then end it here
                if( charbuffer[i] == GLOBALS.DATAFILE_SEPARATOR1)
                {
                    outstring += new String(charbuffer,0, i);

                    // Rewind file to after the seperator
                    // This assumes 1 byte = 1 char. This hopefully is true ....
                    inputfile.Position = currpos + 1;

                    return outstring;
                }

                currpos++;
            }

            // Seperator not found, add all bytes in
            outstring += new String(charbuffer, 0, charcount);
        }

        // Reached end of file wihtout seeing ending seperator. Return partial string for now
        return outstring;
    }

    public static bool PB_GetLineFromStream(Stream inputfile,  out string outline)
    {
        // Initialize outline
        outline = "";

        // First read the length of this line
        // Get length of compression
        string read_length = PB_GetString(inputfile);
        int length = 0;
        if (!int.TryParse(read_length, out length) || (inputfile.Length - inputfile.Position) < length)
        {
            // Skip, something went wrong
            LogMessageToFile("Read line error at file location " + inputfile.Position + " expecting " + length + " bytes from available " + (inputfile.Length - inputfile.Position) + ".");
            return false;
        }

        // Read in the length of bytes
        byte[] bytebuffer = new byte[length];
        int bytesread = inputfile.Read(bytebuffer, 0, length);

        // See if we were able to read all our bytes
        if( bytesread != length)
        {
            // Skip, something went wrong
            LogMessageToFile("Unable to read all required bytes at file location " + inputfile.Position);
            return false;
        }

        // No compression, just UTF-8 decoding
        Decoder utfdecoder = Encoding.GetEncoding("UTF-8").GetDecoder();
        int charcount = utfdecoder.GetCharCount(bytebuffer,0,bytesread);

        char[] charbuffer = new char[charcount];
        utfdecoder.GetChars(bytebuffer, 0, bytesread, charbuffer, 0);

        outline = new string(charbuffer);
        return true;
    }

    public static bool PB_LoadFromFile(string filename)
    {
        // Create file
        FileInfo inputfile_info = new FileInfo(filename);
        FileStream inputfile = null;

        // try opening the file
        try
        {
            inputfile = inputfile_info.Open(FileMode.Open, FileAccess.Read);

            // Read the version info to file and the game info
            string readline = PB_GetString(inputfile);
            string read_game = PB_GetString(inputfile);
            string read_compression = PB_GetString(inputfile);

            if ((GLOBALS.VERSION != readline) || (GLOBALS.GAME_INDEX.ToString() != read_game) || (read_compression != "1") )
            {
                // If version info doesn't pass the smell test, abort
                if((readline.Length < 1) || (readline[0] != 'v') || (read_game.Length < 1) )
                {
                    if (GLOBALS.topclient)
                    {
                        GLOBALS.topclient.ShowMessage("This appears not to be a xRC Data file. Aborting.");
                    }
                    else
                    {
                        LogMessageToFile("This appears not to be a xRC Data file. Aborting.");
                    }

                    return false;
                }

                // If version mis-match, warn user
                if (GLOBALS.VERSION != readline)
                {
                    if (GLOBALS.topclient)
                    {
                        GLOBALS.topclient.ShowMessage("Version mis-match: our version = " + GLOBALS.VERSION + ", Data file = " + readline + ". Issues may occur.");
                    }
                    else
                    {
                        LogMessageToFile("Version mis-match: our version = " + GLOBALS.VERSION + ", Data file = " + readline + ". Issues may occur.");
                    }

                    // Adjust BHelper algorithm 
                    string version_s = readline.Substring(1, readline.Length - 2);
                    float version = float.Parse(version_s);
                    if( (version < 6.2f) || (readline == "v6.2a" || readline == "v6.2b"))
                    {
                        GLOBALS.FORCE_OLD_BHELP = true;
                    }
                    else
                    {
                        GLOBALS.FORCE_OLD_BHELP = false;
                    }
                   
                }

                // If game mis-match, exit
                if (GLOBALS.GAME_INDEX.ToString() != read_game)
                {
                    if (GLOBALS.topclient)
                    {
                        GLOBALS.topclient.ShowMessage("Wrong game: our game = " + GLOBALS.GAME_INDEX + ", Data file = " + read_game );
                    }
                    else
                    {
                        LogMessageToFile("Wrong game: our game = " + GLOBALS.GAME_INDEX + ", Data file = " + read_game);
                    }

                    return false;
                }

                // If compression isn't there, then this may be an old data file format
                if (read_compression.Length != 1)
                {
                    // Close our file and run old algorithm
                    inputfile.Close();
                    return PB_LoadFromFile_old(filename);
                }
            }

            // Clear the current data
            PB_ClearRecording();

            // Create the compressed (or uncompress) data to a memory stream
            Stream inputstream = new MemoryStream();
            Stream zipped_stream;

            if (read_compression == "0") 
            {
                zipped_stream = inputfile; 
                             
            }
            else if (read_compression == "1") //ZLib
            {
                zipped_stream = new ZlibStream(inputfile, CompressionMode.Decompress);
            }
            else
            {
                // Assume default is gzip stream
                zipped_stream = new GZipStream(inputfile, CompressionMode.Decompress);
            }

            zipped_stream.CopyTo(inputstream);
            zipped_stream.Close();

            // Rewind inputstream
            inputstream.Position = 0;

            // Read al the data in
            while ((inputstream.Length - inputstream.Position) > 5)
            {
                // Get uncompressed line
                string inline;
                if(! PB_GetLineFromStream(inputstream, out inline) )
                {
                    // Skip, something went wrong
                    LogMessageToFile("Read line error at file location " + inputfile.Position);
                    continue;
                }

                string[] split_data = inline.Split(GLOBALS.DATAFILE_SEPARATOR1);          

                string stime = split_data[0];
                int time = 0;
                if( ! int.TryParse(stime, out time) ||  (split_data.Length < 2))
                {
                    // Skip, something went wrong
                    LogMessageToFile("Read line error at file location " + inputfile.Position + " time=" + time + ", File Length=" + (inputfile.Length - inputfile.Position));
                    continue;
                }

                // Read in data
                string rawdata = split_data[1];
                string[] data = rawdata.Split(GLOBALS.DATAFILE_SEPARATOR2);
                PB_RecordData(data, time, true);
            }

            inputstream.Close();
            inputfile.Close();

        }
        catch (Exception e)
        {
            if (GLOBALS.topclient)
            {
                GLOBALS.topclient.ShowMessage("Open / read file failed " + e);
            }

            LogMessageToFile("Open/read file failed " + e, true);
            if (inputfile != null) { inputfile.Close(); }
            return false;
        }

        
        return true;
    }

    public static bool PB_LoadFromFile_old(string filename)
    {
        // Create file
        FileInfo inputfile_info = new FileInfo(filename);
        FileStream inputfile;

        // try opening the file
        try
        {
            inputfile = inputfile_info.Open(FileMode.Open, FileAccess.Read);

            // Read the version info to file and the game info
            string readline = PB_GetString(inputfile);
            string read_game = PB_GetString(inputfile);

            if ((GLOBALS.VERSION != readline) || (GLOBALS.GAME_INDEX.ToString() != read_game))
            {
                // If version info doesn't pass the smell test, abort
                if ((readline.Length < 1) || (readline[0] != 'v') || (read_game.Length < 1))
                {
                    if (GLOBALS.topclient)
                    {
                        GLOBALS.topclient.ShowMessage("This appears not to be a xRC Data file. Aborting.");
                    }
                    else
                    {
                        LogMessageToFile("This appears not to be a xRC Data file. Aborting.");
                    }

                    return false;
                }

                // If version mis-match, warn user
                // Already warned on the non older version
                if (GLOBALS.VERSION != readline)
                {
                    if (GLOBALS.topclient)
                    {
                    //    GLOBALS.topclient.ShowMessage("Version mis-match: our version = " + GLOBALS.VERSION + ", Data file = " + readline + ". Issues may occur.");
                    }
                    else
                    {
                    //    LogMessageToFile("Version mis-match: our version = " + GLOBALS.VERSION + ", Data file = " + readline + ". Issues may occur.");
                    }
                }

                // If game mis-match, exit
                if (GLOBALS.GAME_INDEX.ToString() != read_game)
                {
                    if (GLOBALS.topclient)
                    {
                        GLOBALS.topclient.ShowMessage("Wrong game: our game = " + GLOBALS.GAME_INDEX + ", Data file = " + read_game);
                    }
                    else
                    {
                        LogMessageToFile("Wrong game: our game = " + GLOBALS.GAME_INDEX + ", Data file = " + read_game);
                    }

                    return false;
                }
            }

            // Clear the current data
            PB_ClearRecording();

            // Read al the data in
            while ((inputfile.Length - inputfile.Position) > 5)
            {
                string stime = PB_GetString(inputfile);
                int time = 0;
                if (!int.TryParse(stime, out time) || (inputfile.Length - inputfile.Position) < 1)
                {
                    // Skip, something went wrong
                    LogMessageToFile("Read line error at file location " + inputfile.Position);
                    continue;
                }

                // Read in data
                string rawdata = PB_GetString(inputfile);
                string[] data = rawdata.Split(GLOBALS.DATAFILE_SEPARATOR2);
                PB_RecordData(data, time, true);
            }

        }
        catch (Exception e)
        {
            if (GLOBALS.topclient)
            {
                GLOBALS.topclient.ShowMessage("Open / read file failed " + e);
            }

            LogMessageToFile("Open/read file failed " + e, true);
            return false;
        }

        return true;
    }

    public static void AppendFile( string filename, string msg )
    {
        StreamWriter sw = File.AppendText(filename);
        sw.WriteLine(msg);
        sw.Close();
    }

// Get 3D Field
    public static float GetFloorYPos()
    {

        // OVR floor: tries to get the OVR floor position
        GameObject[] field_structures = GameObject.FindGameObjectsWithTag("FieldStructure");
        if (field_structures.Length < 1) { return 0; }

        foreach (GameObject currobj in field_structures)
        {
            if (currobj.name == "3d field")
            {
                Transform floor = currobj.transform.Find("floor");
                if (floor == null)
                {
                    floor = currobj.transform.Find("matts");
                }
                if (floor == null)
                {
                    floor = currobj.transform.Find("matt");
                }


                // If for some reason we couldn't find it, abonden the process
                if (floor == null)
                {
                    return 0;
                }

                return floor.transform.position.y;
            }
        }

        return 0f;
    }

    public static void AdjustCameraForScale(GameObject main_camera, Transform player_camera_transform, Transform robot_ref = null)
    {

        // Set Field of view
        float newfov = 60f * GLOBALS.worldscale / 2f; ;
        main_camera.GetComponent<Camera>().fieldOfView = newfov;

        // Set camera to player_camera position

        // Adjust position
        Vector3 startpos = player_camera_transform.position;
        float distance_to_bot = (robot_ref == null) ? player_camera_transform.position.y : Vector3.Distance(player_camera_transform.position, robot_ref.position);
//        Vector3 newpos = startpos + ((float)(Math.Log(newfov) - Math.Log(60f))) * 1f * player_camera_transform.forward * distance_to_bot; // * startpos.y * ((float)(Math.Log(newfov) - Math.Log(60f)));
        Vector3 newpos = startpos - (60f/newfov-1f) * player_camera_transform.forward * distance_to_bot; // * startpos.y * ((float)(Math.Log(newfov) - Math.Log(60f)));

        main_camera.transform.position = newpos;
    }

    // Given a table of [x,y], returns a Y that linearly interpolates on x
    public static float GetInterpolatedValue(float x, float[,] table)
    {
        float last_x = table[0, 0];
        float last_y = table[0, 1];
        float curr_x = 0;
        float curr_y = 0;

        for (int index = 1; index < table.Length; index++)
        {
            curr_x = table[index, 0];
            curr_y = table[index, 1];

            if ((curr_x >= x && last_x <= x) ||
                (curr_x <= x && last_x >= x))
            {
                return LinearInterpolate(curr_x, curr_y, last_x, last_y, x);
            }

            last_x = curr_x;
            last_y = curr_y;
        }

        // Couldn't interpolate normally, return closest interpolation we can find
        int endi = table.Length - 1;
        if (Math.Abs(x - table[0, 0]) < Math.Abs(table[endi, 0] - x))
        {
            return LinearInterpolate(table[0, 0], table[0, 1], table[1, 0], table[1, 1], x);
        }


        return LinearInterpolate(table[endi, 0], table[endi, 1], table[endi - 1, 0], table[endi - 1, 1], x);

    }

    public static float LinearInterpolate(float x1, float y1, float x2, float y2, float x)
    {
        float slope = (y2 - y1) / (x2 - x1);

        return (x - x1) * slope + y1;
    }

    public static string PrintVector3(Vector3 thevector)
    {
        return "[" + thevector.x + "," + thevector.y + "," + thevector.z + "]";
    }

    // Returns true if instring[0]==1
    public static bool GetBoolFromString(string instring)
    {
        if( instring.Length<1) { return false; }

        return instring[0] == '1';
    }

    // Returns float representation if possible
    public static float GetFloatFromString(string instring, float lower_limit = -1f, float upper_limit=1f)
    {
        float outfloat = 0f;

        if( float.TryParse(instring, out outfloat))
        {
            if( outfloat > upper_limit)
            {
                outfloat = upper_limit;
            }
            if( outfloat < lower_limit)
            {
                outfloat = lower_limit;
            }    

            return outfloat;
        }
        return 0f;
    }

    // XRIsPresent fix
    public static bool XR_isPresent()
    {
        //return XRGeneralSettings.Instance.Manager.activeLoader != null;
        if( !XRGeneralSettings.Instance || !XRGeneralSettings.Instance.Manager) { return false; }

        return XRGeneralSettings.Instance.Manager.isInitializationComplete;
        /*
        var xrDisplaySubsystems = new List<XRDisplaySubsystem>();
        SubsystemManager.GetInstances<XRDisplaySubsystem>(xrDisplaySubsystems);
        foreach (var xrDisplay in xrDisplaySubsystems)
        {
            if (xrDisplay.running)
            {
                return true;
            }
        }
        return false;
        */
    }

    public static void XR_Start()
    {     
        if( !XRGeneralSettings.Instance || !XRGeneralSettings.Instance.Manager)
        {
            Debug.LogError("Unable to start VR: Instance or Manager missing.");
            return;
        }
        XRGeneralSettings.Instance.Manager.InitializeLoaderSync();
        XRGeneralSettings.Instance.Manager.StartSubsystems();
    }

    public static void XR_Stop()
    {
        if (!XRGeneralSettings.Instance || !XRGeneralSettings.Instance.Manager)
        {
            Debug.LogError("Unable to stop VR: Instance or Manager missing.");
            return;
        }
        // XRGeneralSettings.Instance.Manager.StopSubsystems(); // called automatically when deinitializing
        XRGeneralSettings.Instance.Manager.DeinitializeLoader();
        
    }


    // String split using ReadOnlySpan functions
    // First entry is -1 (meaning before start of string)
    public static List<int> Split(ReadOnlySpan<char> instring, char character)
    {
        List<int> char_locations = new List<int>();
        char_locations.Add(-1);

        for(int i=0; i<instring.Length; i++)
        {
            if( instring[i] == character)
            {
                char_locations.Add(i);
            }
        }

        return char_locations;
    }

    public static bool CompareSplit( ReadOnlySpan<char> instring, List<int> indexes, int index, string comparison)
    {
        // Get start/stop of split
        int startindex = indexes[index]+1;
        int stringlength = ((index+1 >= indexes.Count) ? instring.Length : indexes[index + 1]) - startindex;
        return instring.Slice(startindex, stringlength).SequenceCompareTo<char>(comparison.AsSpan()) == 0;    
    }

    public static String GetSplitString(ReadOnlySpan<char> instring, List<int> indexes, int index)
    {
        // Get start/stop of split
        int startindex = indexes[index] + 1;
        int stringlength = ((index+1 >= indexes.Count) ? instring.Length : indexes[index + 1]) - startindex;
        return instring.Slice(startindex, stringlength).ToString();
    }

    public static ReadOnlySpan<char> GetSplitSpan(ReadOnlySpan<char> instring, List<int> indexes, int index)
    {
        // Get start/stop of split
        int startindex = indexes[index] + 1;
        int stringlength = ((index+1 >= indexes.Count) ? instring.Length : indexes[index + 1]) - startindex;
        return instring.Slice(startindex, stringlength);
    }
};

