using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.Build.Reporting;
using System.Text.RegularExpressions;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;

public class BuildScript 
{
    public enum XRPlugin
    {
        OpenXR,
        Oculus,
        OpenVR
    }

    static string GetLoaderName(XRPlugin plugin) => plugin switch
    {
        XRPlugin.OpenXR => "UnityEngine.XR.OpenXR.OpenXRLoader",
        XRPlugin.Oculus => "Unity.XR.Oculus.OculusLoader",
        XRPlugin.OpenVR => "Unity.XR.OpenVR.OpenVRLoader",
        _ => ""
    };

    static void Build()
    {
        bool use_version = false;
        string VERSION = GLOBALS.VERSION.Substring(0, GLOBALS.VERSION.Length - 1);
        Regex rgx = new Regex("^\\D[\\d\\.]+");
        Match thematch = rgx.Match(GLOBALS.VERSION);
        if (thematch.Success)
        {
            VERSION = thematch.Value;
        }



        string single_build_option = "";
        bool single_build = false;

        string[] arguments = System.Environment.GetCommandLineArgs();
        foreach( string currarg in arguments)
        {
            string[] myarg = currarg.Split('=');
            if( myarg.Length >= 2)
            {
                if( myarg[0].StartsWith("BUILDONLY"))
                {
                    single_build_option = myarg[1];
                    single_build = true;
                }

            }
        }


        // Start windows console

        System.Console.WriteLine("**** STARTING CONSOLE FOR WINDOWS *****");
        Windows.ConsoleWindow win_console = new Windows.ConsoleWindow();
        win_console.Initialize();
        System.Console.WriteLine("**** CONSOLE FOR WINDOWS STARTED *****");
        if( single_build )
        {
            win_console.MyWrite("Single build started with setting =" + single_build_option);
        }

        // Get the current list of scenes
        List<string> scenes = new List<string>();
        foreach (EditorBuildSettingsScene scene in EditorBuildSettings.scenes)
        {
            if (scene.enabled)
            { scenes.Add(scene.path); }
        }

        BuildPlayerOptions mybuildoptions = new BuildPlayerOptions();
        mybuildoptions.targetGroup = BuildTargetGroup.Standalone;
        mybuildoptions.subtarget = (int)StandaloneBuildSubtarget.Player;
        mybuildoptions.scenes = scenes.ToArray();
        mybuildoptions.options = BuildOptions.None;

        // XR enabling/disabling
        var buildTargetSettings = XRGeneralSettingsPerBuildTarget.XRGeneralSettingsForBuildTarget(mybuildoptions.targetGroup);
        var pluginsSettings = buildTargetSettings.AssignedSettings;

        BuildReport report;
        BuildSummary summary;

        if (!single_build || (single_build_option == "WIN64"))
        {
            // *******************************
            // ******* Windows x64 ****
            mybuildoptions.locationPathName = "../builds/Windows x64/xRC Simulator";
            if (use_version)
            {
                mybuildoptions.locationPathName += " " + VERSION;
            }
            mybuildoptions.locationPathName += ".exe";            
            mybuildoptions.target = BuildTarget.StandaloneWindows64;


            // Enable OpenVR
            XRPackageMetadataStore.AssignLoader(pluginsSettings, GetLoaderName(XRPlugin.OpenXR), mybuildoptions.targetGroup);

            win_console.MyWrite("Starting " + mybuildoptions.target.ToString());
            report = BuildPipeline.BuildPlayer(mybuildoptions);
            summary = report.summary;

            if (summary.result == BuildResult.Succeeded)
            {
                Debug.Log("Build succeeded Windows x64: " + summary.totalSize + " bytes");
                win_console.MyWrite("Build succeeded Windows x64: " + summary.totalSize + " bytes");
            }
            else
            {
                Debug.Log("Build failed.");
                win_console.MyWrite("Build Failed.");
            }
        }


        // Remove VR from here on
        XRPackageMetadataStore.RemoveLoader(pluginsSettings, GetLoaderName(XRPlugin.OpenXR), mybuildoptions.targetGroup);


        if (!single_build || (single_build_option == "WIN86"))
        {
            // *******************************
            // ******* Windows x86 ****
            /* mybuildoptions.locationPathName = "../builds/Windows x86/xRC Simulator " + VERSION + ".exe";
             mybuildoptions.target = BuildTarget.StandaloneWindows;

             win_console.MyWrite("Starting " + mybuildoptions.target.ToString());
             report = BuildPipeline.BuildPlayer(mybuildoptions);
             summary = report.summary;

             if (summary.result == BuildResult.Succeeded)
             {
                 Debug.Log("Build succeeded Windows x86: " + summary.totalSize + " bytes");
             }
             else
             {
                 Debug.Log("Build failed.");
             }
             */
        }

        if (!single_build || (single_build_option == "LINUX"))
        {
            // *******************************
            // ******* Linux x64 ****
            mybuildoptions.locationPathName = "../builds/Linux/xRC Simulator";

            if (use_version)
            {
                mybuildoptions.locationPathName += " " + VERSION;
            }
            mybuildoptions.locationPathName += ".x86_64";       
            mybuildoptions.target = BuildTarget.StandaloneLinux64;



            win_console.MyWrite("Starting " + mybuildoptions.target.ToString());
            report = BuildPipeline.BuildPlayer(mybuildoptions);
            summary = report.summary;

            if (summary.result == BuildResult.Succeeded)
            {
                Debug.Log("Build succeeded Linux: " + summary.totalSize + " bytes");
                win_console.MyWrite("Build succeeded Linux: " + summary.totalSize + " bytes");
            }
            else
            {
                Debug.Log("Build failed.");
                win_console.MyWrite("Build Failed.");
            }
        }

        if (!single_build || (single_build_option == "OSX"))
        {
            // *******************************
            // ******* MacOSX  ****
            mybuildoptions.locationPathName = "../builds/OSX/xRC Simulator";
            
            if (use_version)
            {
                mybuildoptions.locationPathName += " " + VERSION;
            }
            mybuildoptions.locationPathName += ".app"; 
            mybuildoptions.target = BuildTarget.StandaloneOSX;

            win_console.MyWrite("Starting " + mybuildoptions.target.ToString());
            report = BuildPipeline.BuildPlayer(mybuildoptions);
            summary = report.summary;

            if (summary.result == BuildResult.Succeeded)
            {
                Debug.Log("Build succeeded MaxOSX: " + summary.totalSize + " bytes");
                win_console.MyWrite("Build succeeded MaxOSX: " + summary.totalSize + " bytes");
            }
            else
            {
                Debug.Log("Build failed.");
                win_console.MyWrite("Build Failed.");
            }
        }

        if (!single_build || (single_build_option == "SERVER"))
        {
            // *******************************
            // ******* Linux Server   ****
            mybuildoptions.locationPathName = "../builds/Linux Server/xRC Simulator";
            if (use_version)
            {
                mybuildoptions.locationPathName += " " + VERSION;
            }
            mybuildoptions.locationPathName += ".x86_64";

            mybuildoptions.target = BuildTarget.StandaloneLinux64;
            mybuildoptions.subtarget = (int) StandaloneBuildSubtarget.Server;
          

            win_console.MyWrite("Starting " + mybuildoptions.target.ToString());
            report = BuildPipeline.BuildPlayer(mybuildoptions);
            summary = report.summary;

            if (summary.result == BuildResult.Succeeded)
            {
                Debug.Log("Build succeeded Linux Server: " + summary.totalSize + " bytes");
                win_console.MyWrite("Build succeeded Linux Server: " + summary.totalSize + " bytes");
            }
            else
            {
                Debug.Log("Build failed.");
                win_console.MyWrite("Build Failed.");
            }
        }
    }
}
