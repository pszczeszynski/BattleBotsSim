using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using System.IO;
using System.Security;

namespace Windows
{
	/// <summary>
	/// Creates a console window that actually works in Unity
	/// You should add a script that redirects output using Console.Write to write to it.
	/// </summary>
	[SuppressUnmanagedCodeSecurity]
	public class ConsoleWindow
	{
		TextWriter oldOutput;
		StreamWriter standardOutput;

		public void Initialize()
		{
			// Instead of redirecting, will just output to console when a write command happens
			//
			// Attach to any existing consoles we have
			// failing that, create a new one.
			//
			if (!AttachConsole(0x0ffffffff))
			{
				AllocConsole();
			}

			oldOutput = Console.Out;

			try
			{
				// At this point the console actually works and none of the items below are required
				// Perhaps setting the outputencoding automatically redirects stdout? In this case, we can't set it here


				/*		
				IntPtr stdHandle = GetStdHandle(STD_OUTPUT_HANDLE);
				Microsoft.Win32.SafeHandles.SafeFileHandle safeFileHandle = new Microsoft.Win32.SafeHandles.SafeFileHandle(stdHandle, true);
				FileStream fileStream = new FileStream(safeFileHandle, FileAccess.Write);

				System.Text.Encoding encoding = System.Text.Encoding.UTF8
				standardOutput = new MyFilteredStreamWriter(fileStream, encoding);
				standardOutput.AutoFlush = true;
				SetTitle("xRC Simulator");
				Console.SetOut(standardOutput);
				*/

			}
			catch (System.Exception e)
			{
				Debug.Log("Couldn't redirect output: " + e.Message);
			}
		}

		public void MyWrite(string msg)
		{
			Console.WriteLine(msg);
		}

		public void Shutdown()
		{
			// Console.SetOut(oldOutput);
			FreeConsole();
		}

		public void SetTitle(string strName)
		{
			SetConsoleTitleA(strName);
		}

		private const int STD_INPUT_HANDLE = -10;
		private const int STD_OUTPUT_HANDLE = -11;

		[DllImport("kernel32.dll", SetLastError = true)]
		static extern bool AttachConsole(uint dwProcessId);

		[DllImport("kernel32.dll", SetLastError = true)]
		static extern bool AllocConsole();

		[DllImport("kernel32.dll", SetLastError = true)]
		static extern bool FreeConsole();

		[DllImport("kernel32.dll", EntryPoint = "GetStdHandle", SetLastError = true, CharSet = CharSet.Auto, CallingConvention = CallingConvention.StdCall)]
		private static extern IntPtr GetStdHandle(int nStdHandle);

		[DllImport("kernel32.dll")]
		static extern bool SetConsoleTitleA(string lpConsoleTitle);
	}
}

// Overwriting StreamWriter's write function to allow filtering
/*
public class MyFilteredStreamWriter : StreamWriter
{
	public MyFilteredStreamWriter(Stream stream, System.Text.Encoding encoding) : base(stream, encoding)
	{

	}

	public override void Write(char[] buffer)
	{
		char[] output = ("Write chars " + (new String(buffer))).ToCharArray();
		base.Write(output);
	}

	public override void Write(string value)
	{
		base.Write("Write string:" + value);
	}

	public override void WriteLine(char[] buffer)
	{
		char[] output = ("Write Line chars " + (new String(buffer))).ToCharArray();
		base.WriteLine(output);
	}


	public override void WriteLine(string value)
	{
		base.WriteLine("WriteLine string: " + value);
	}
}
*/