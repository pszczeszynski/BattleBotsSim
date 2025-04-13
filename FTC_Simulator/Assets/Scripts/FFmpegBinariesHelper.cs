using System;
using System.IO;
using System.Runtime.InteropServices;
using FFmpeg.AutoGen;
using UnityEngine;

public class FFmpegBinariesHelper
{
    static public bool ffmpegOK = false;
    internal static void RegisterFFmpegBinaries()
    {
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
        {
            string current = Environment.CurrentDirectory;

            string probe;

            if( Application.isEditor )
            {
                probe = Path.Combine("Assets", "Plugins", "ffmpeg", "bin");
            }
            else
            {
                probe = Path.Combine("xRC Simulator_Data", "Plugins",  "x86_64");
            }

            while (current != null)
            {
                var ffmpegBinaryPath = Path.Combine(current, probe);

                if (Directory.Exists(ffmpegBinaryPath))
                {
                    ffmpeg.RootPath = ffmpegBinaryPath;

                    try
                    {


                        // Make sure the av_version_info works
                        UnityEngine.Debug.Log("FFMPEG Version: " + ffmpeg.av_version_info());

                        // If we got here then it appears to be working
                        ffmpegOK = true;
                    }
                    catch (Exception e)
                    {
                        UnityEngine.Debug.Log("FFMPEG failed!");
                    }

                    return;
                }

            }
        }
        else
            throw new NotSupportedException(); // fell free add support for platform of your choose
    }
}
