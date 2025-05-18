using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using SimpleFileBrowser;
using System.IO;
using System.Threading.Tasks;
using System.Threading;

public class RecordManager : MonoBehaviour
{

    public Toggle toggle_rec;
    public Toggle toggle_stop;
    public Toggle toggle_save;
    public string save_dir = ".";
    public bool initialized = false;
    public CameraCapture recordingDevice;

    bool recording = false;
    private string fileExtention = "avi";
    private string save_file;

    // Start is called before the first frame update
    void Start()
    {
        // Initialize buttons
        toggle_rec.interactable = false;
        toggle_stop.interactable = false;
        toggle_save.interactable = false;

        toggle_rec.isOn = false;
        toggle_save.isOn = false;
        toggle_stop.isOn = false;

        save_file = "BB_Video." + fileExtention;

        // Initialize drive information
        Task.Run(() => InitDriveInfo());

            
    }

    private void InitDriveInfo()
    {
        Thread.Sleep(500);

        // This runs the windows initialization that polls all drives. Networked drives that are invalid can make this take many seconds 
        DriveInfo drive = new DriveInfo("C:\\");
        initialized = true;
    }


    public void RecordPressed()
    {
        // Make sure its pressed
        if( !toggle_rec.isOn)
        {
            return;
        }

        toggle_stop.isOn = false;

        // Ignore double presses
        if (recording ) { return; }

        // Start recording
        if (recordingDevice.StartRecording(save_dir, save_file, FileBrowser.compress))
        {
            toggle_rec.interactable = false;
            toggle_save.interactable = false;
            toggle_stop.interactable = true;
            recording = true;
        }
        else
        {
            // Otherwise we failed
            toggle_rec.isOn = false;
        }

    }

    public void StopRecording()
    {
        // Make sure StopRecording was pressed
        if(!toggle_stop.isOn)
        {
            return;
        }

        // Ignore double presses
        if (!recording) { return; }

        recordingDevice.CloseFile();

        toggle_rec.interactable = true;
        toggle_save.interactable = true;
        toggle_stop.interactable = false;

        toggle_rec.isOn = false;
        toggle_save.isOn = false;
        

        recording = false;
    }

    public void SavePressed(bool state)
    {
        if (state) { toggle_save.isOn = false; }

        // If this was a script releasing it, do nothing
        if (!state) { return; }

        // If file browser already open, don't do anything
        if (FileBrowser.IsOpen) { return; }

        // Coroutine example
        StartCoroutine(ShowSaveDialogCoroutine());

    }



    IEnumerator ShowSaveDialogCoroutine()
    {
        // Set filter to be .mp4 files only
        FileBrowser.Filter filters = new FileBrowser.Filter(fileExtention.ToUpper() + " Files", "." + fileExtention);

        FileBrowser.SetFilters(true, filters);
        FileBrowser.SetDefaultFilter("." + fileExtention);


        // Load file/folder: both, Allow multiple selection: true
        // Initial path: default (Documents), Initial filename: empty
        // Title: "Load File", Submit button text: "Load"
        yield return FileBrowser.WaitForSaveDialog(FileBrowser.PickMode.Files, false, null, save_file, "Save Video", "Save");

        // Dialog is closed
        // Print whether the user has selected some files/folders or cancelled the operation (FileBrowser.Success)
        Debug.Log(FileBrowser.Success);

        if (FileBrowser.Success)
        {
            if (FileBrowser.Result.Length != 1)
            {
                UnityEngine.Debug.LogError("Did not chose 1 file...");
                yield break;

            }

            if (MyUtils.PB_SaveToFile(FileBrowser.Result[0]))
            {

                save_dir = Path.GetDirectoryName(FileBrowser.Result[0]);
                save_file = Path.GetFileName(FileBrowser.Result[0]);
                toggle_rec.interactable = true;
            }

        }
    }



    // Update is called once per frame
    void Update()
    {
        if( initialized)
        {
            toggle_save.interactable = true;
        }
    }
}
