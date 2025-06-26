using UnityEngine;
using System.IO.MemoryMappedFiles;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

using System;
using System.Diagnostics;
using FFmpeg.AutoGen;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;



public unsafe class CameraCapture : MonoBehaviour
{
    public Camera cameraToCapture; // Assign the camera you want to capture in the Inspector
    public string fileName;
    public int targetFrameRate = 150;

    Stopwatch stopwatch = new Stopwatch();
    private RenderTexture rt;
    private Texture2D tex;
    private AVFormatContext* formatContext;
    private AVCodecContext* codecContext;
    private AVStream* videoStream;
    private int frameCount = 0;
    private bool isFileOpen = false;
    private string outputPath = "output.avi";

    private MemoryMappedFile mmf;
    private MemoryMappedViewAccessor accessor;
    //private static int width = 1280;
    //private static int height = 720;
    private static int frameNumber = 1;
    private static int width = 1440;
    private static int height = 768;
    private static long bitrate = 15000000;
    // Buffer
    private byte[] frameDataArray;
    private AVPacket* packet;
    private byte[] yLUT, uLUT, vLUT;
    private bool useCompression = false;
    public bool forceGrayscale = true;
    private EventWaitHandle frameReadyEvent; // Event to signal new frame

    private Material blurMaterial; // Material for blur effect
    private RenderTexture tempRT; // Temporary RenderTexture for blur
    public float blurSize = 0.01f; // Blur size parameter



    void Start()
    {
        // create the render texture
        rt = new RenderTexture(width, height, 16, RenderTextureFormat.ARGB32);
        rt.Create();

        // Create a Texture2D to store the camera's render texture
        tex = new Texture2D(width, height, TextureFormat.RGBA32, false);

        // Create a temporary RenderTexture for blur processing
        tempRT = new RenderTexture(width, height, 16, RenderTextureFormat.ARGB32);
        tempRT.Create();

        // Create the blur material
        Shader blurShader = Shader.Find("Custom/GaussianBlur");
        if (blurShader == null)
        {
            UnityEngine.Debug.LogError("GaussianBlur shader not found. Ensure the shader is in the project.");
            return;
        }
        blurMaterial = new Material(blurShader);
        blurMaterial.SetFloat("_BlurSize", blurSize);


        int size = width * height * 4 * 4;
        // Create a memory-mapped file with a specified size
        mmf = MemoryMappedFile.CreateNew(fileName, size + 3 * sizeof(int));

        // Create a memory-mapped view accessor to write to the memory-mapped file
        accessor = mmf.CreateViewAccessor();

        // Create the named event for signaling new frames
        try
        {
            frameReadyEvent = new EventWaitHandle(false, EventResetMode.AutoReset, "FrameReadyEvent");
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"Failed to create EventWaitHandle: {ex.Message}");
            return;
        }

        // Make sure vSync is off since it can interfere with physics engines and maximum refresh rate
        QualitySettings.vSyncCount = 0;

        // Make sure our displayed frame rate is at least our targetFrameRate
        //if (Application.targetFrameRate < targetFrameRate)
        //{
        //    Application.targetFrameRate = targetFrameRate;
        //
        //}

        // Initialize FFmpeg to allow outputing avi files
        FFmpegBinariesHelper.RegisterFFmpegBinaries();


        // Initialize buffer for FFmpreg
        int ySize = width * height;
        int uvSize = ySize / 4;
        frameDataArray = new byte[ySize + 2 * uvSize];

        stopwatch.Start();
    }

    float previousDeltaTime = 0.01f;
    public bool StartRecording(string path, string filename, bool doCompression = false)
    {
        useCompression = doCompression;

        // Remeber the file to open
        outputPath = Path.Combine(path, filename);

        // Enforce minimum frame rate
        previousDeltaTime = Time.maximumDeltaTime;

        if (Time.maximumDeltaTime > 1.0f / ((float)targetFrameRate))
        {
            Time.maximumDeltaTime = 1.0f / ((float)targetFrameRate);
        }



        // Initialize FFmpeg
        return InitializeFFmpeg();
    }

    private AVFrame* frame;

    bool InitializeFFmpeg()
    {
        ffmpeg.avformat_network_init();


        // Choose the codec (e.g., H.264 or RAW)
        AVCodec* codec;

        if (useCompression) { codec = ffmpeg.avcodec_find_encoder(AVCodecID.AV_CODEC_ID_H264); }
        else {  codec = ffmpeg.avcodec_find_encoder(AVCodecID.AV_CODEC_ID_RAWVIDEO); }

        if (codec == null)
        {
            UnityEngine.Debug.LogError("Codec not found");
            return false;
        }

        formatContext = ffmpeg.avformat_alloc_context();
        formatContext->oformat = ffmpeg.av_guess_format("avi", null, null); // Use MP4 for H.264

        if (ffmpeg.avio_open(&formatContext->pb, outputPath, ffmpeg.AVIO_FLAG_WRITE) < 0)
        {
            UnityEngine.Debug.LogError("Could not open output file");
            return false;
        }

        codecContext = ffmpeg.avcodec_alloc_context3(codec);
        codecContext->bit_rate = bitrate;
        codecContext->width = rt.width;
        codecContext->height = rt.height;
        codecContext->time_base = new AVRational { num = 1, den = targetFrameRate };
        codecContext->gop_size = 10; // Group of Pictures size
        codecContext->max_b_frames = 1; // Number of B-frames

        if (forceGrayscale)
        {
            codecContext->pix_fmt = AVPixelFormat.AV_PIX_FMT_GRAY8;
        }
        else
        {
            codecContext->pix_fmt = AVPixelFormat.AV_PIX_FMT_YUV420P; // Required for H.264
        }

        // Set H.264 preset and tune (optional)
        if (useCompression)
        {
            ffmpeg.av_opt_set(codecContext->priv_data, "preset", "veryfast", 0);
            ffmpeg.av_opt_set(codecContext->priv_data, "tune", "zerolatency", 0);
        }

        if (ffmpeg.avcodec_open2(codecContext, codec, null) < 0)
        {
            UnityEngine.Debug.LogError("Could not open codec");
            return false;
        }

        videoStream = ffmpeg.avformat_new_stream(formatContext, null);
        videoStream->codecpar = ffmpeg.avcodec_parameters_alloc();
        ffmpeg.avcodec_parameters_from_context(videoStream->codecpar, codecContext);
        videoStream->time_base = codecContext->time_base;

        ffmpeg.avformat_write_header(formatContext, null);

        packet = ffmpeg.av_packet_alloc();

        if (useCompression)
        {
            frame = ffmpeg.av_frame_alloc();
            frame->format = (int)codecContext->pix_fmt;
            frame->width = codecContext->width;
            frame->height = codecContext->height;
            if( forceGrayscale)
            {
                ffmpeg.av_frame_get_buffer(frame, 1); // Allocate frame buffer

            }
            else
            {
                ffmpeg.av_frame_get_buffer(frame, 32); // Allocate frame buffer

            }
        }

        // Pre-compute LUTs
        yLUT = new byte[256 * 256 * 256];
        uLUT = new byte[256 * 256 * 256];
        vLUT = new byte[256 * 256 * 256];
        for (int r = 0; r < 256; r++)
        {
            for (int g = 0; g < 256; g++)
            {
                for (int b = 0; b < 256; b++)
                {
                    int index = r * 65536 + g * 256 + b;
                    yLUT[index] = (byte)((0.257 * r) + (0.504 * g) + (0.098 * b) + 16);
                    uLUT[index] = (byte)(-(0.148 * r) - (0.291 * g) + (0.439 * b) + 128);
                    vLUT[index] = (byte)((0.439 * r) - (0.368 * g) - (0.071 * b) + 128);
                }
            }
        }

        isFileOpen = true;
        return true;
    }

    void AddFrameToVideo(NativeArray<Color32> pixels)
    {
        int ySize = width * height;
        int uvSize = ySize / 4;

        unsafe
        {
            fixed (byte* frameData = frameDataArray)
            {
                byte* y = frameData;
                byte* u = frameData + ySize;
                byte* v = u + uvSize;

                // Flip the image vertically while converting to YUV420P
                for (int yCoord = 0; yCoord < height; yCoord++)
                {
                    for (int xCoord = 0; xCoord < width; xCoord++)
                    {
                        int srcIndex = yCoord * width + xCoord;
                        int destIndex = (height - 1 - yCoord) * width + xCoord; // Flip y-coordinate
                        Color32 pixel = pixels[srcIndex];

                        if (forceGrayscale)
                        {
                            // Convert RGB to grayscale using the luminance formula
                            byte gray = (byte)(0.299f * pixel.r + 0.587f * pixel.g + 0.114f * pixel.b);
                            y[destIndex] = gray;
                        }
                        else
                        {
                            int lutIndex = pixel.r * 65536 + pixel.g * 256 + pixel.b;
                            y[destIndex] = yLUT[lutIndex];

                            // Only one U and V per 2x2 block, flipped accordingly
                            if (yCoord % 2 == 0 && xCoord % 2 == 0)
                            {
                                int uvDestIndex = ((height - 1 - yCoord) / 2) * (width / 2) + (xCoord / 2); // Flip y-coordinate for UV planes
                                u[uvDestIndex] = uLUT[lutIndex];
                                v[uvDestIndex] = vLUT[lutIndex];
                            }
                        }
                    }
                }

                if (useCompression)
                {

                    // Prepare AVFrame data and linesize arrays
                    byte_ptrArray4 dstData = new byte_ptrArray4();
                    int_array4 dstLinesize = new int_array4();

                    // Fill the AVFrame data and linesize arrays
                    int ret = ffmpeg.av_image_fill_arrays(ref dstData, ref dstLinesize, frameData, codecContext->pix_fmt, width, height, 1);
                    if (ret < 0)
                    {
                        UnityEngine.Debug.LogError("Failed to fill image arrays");
                        return;
                    }


                    // Copy data pointers to the AVFrame
                    for (uint i = 0; i < ((forceGrayscale) ? 1 : 4); i++)
                    {
                        frame->data[i] = dstData[i]; // Use the indexer to access elements
                        frame->linesize[i] = dstLinesize[i];
                    }



                    frame->pts = frameCount;

                    // Encode the frame
                    if (ffmpeg.avcodec_send_frame(codecContext, frame) < 0)
                    {
                        UnityEngine.Debug.LogError("Error sending frame to encoder");
                        return;
                    }

                    while (true)
                    {
                        ret = ffmpeg.avcodec_receive_packet(codecContext, packet);
                        if (ret == ffmpeg.AVERROR(ffmpeg.EAGAIN) || ret == ffmpeg.AVERROR_EOF)
                        {
                            break;
                        }
                        else if (ret < 0)
                        {
                            UnityEngine.Debug.LogError("Error encoding frame");
                            return;
                        }

                        // Write the encoded packet to the file
                        packet->stream_index = videoStream->index;
                        ffmpeg.av_packet_rescale_ts(packet, codecContext->time_base, videoStream->time_base);
                        packet->pos = -1;

                        if (ffmpeg.av_interleaved_write_frame(formatContext, packet) < 0)
                        {
                            UnityEngine.Debug.LogError("Error while writing video frame");
                        }

                        ffmpeg.av_packet_unref(packet);
                    }
                }
                else // noCompression
                {
                    ffmpeg.av_init_packet(packet);

                    if (forceGrayscale)
                    {
                        ffmpeg.av_new_packet(packet, ySize);
                        packet->size = ySize;

                        Marshal.Copy(frameDataArray, 0, (IntPtr)packet->data, ySize);
                    }
                    else
                    {
                        ffmpeg.av_new_packet(packet, ySize + 2 * uvSize);
                        packet->size = ySize + 2 * uvSize;

                        Marshal.Copy(frameDataArray, 0, (IntPtr)packet->data, ySize + 2 * uvSize);

                    }

                    packet->pts = frameCount;
                    packet->dts = frameCount;

                    if (ffmpeg.av_interleaved_write_frame(formatContext, packet) < 0)
                    {
                        UnityEngine.Debug.LogError("Error while writing video frame");
                    }
                }
                frameCount++;
            }
        }
    }


    public void CloseFile()
    {
        if (!isFileOpen) { return; }

        // Return frame rate
        Time.maximumDeltaTime = previousDeltaTime;

        // Flush the encoder
        ffmpeg.avcodec_send_frame(codecContext, null);
        while (true)
        {
            int ret = ffmpeg.avcodec_receive_packet(codecContext, packet);
            if (ret == ffmpeg.AVERROR(ffmpeg.EAGAIN) || ret == ffmpeg.AVERROR_EOF)
            {
                break;
            }
            else if (ret < 0)
            {
                UnityEngine.Debug.LogError("Error flushing encoder");
                return;
            }

            packet->stream_index = videoStream->index;
            ffmpeg.av_packet_rescale_ts(packet, codecContext->time_base, videoStream->time_base);
            packet->pos = -1;

            if (ffmpeg.av_interleaved_write_frame(formatContext, packet) < 0)
            {
                UnityEngine.Debug.LogError("Error while writing video frame");
            }

            ffmpeg.av_packet_unref(packet);
        }

        ffmpeg.av_write_trailer(formatContext);

        unsafe
        {
            if (codecContext != null)
            {
                fixed (AVCodecContext** pCodecContext = &codecContext)
                {
                    ffmpeg.avcodec_free_context(pCodecContext);
                }
            }
            ffmpeg.avio_closep(&formatContext->pb);
        }

        ffmpeg.avformat_free_context(formatContext);

        isFileOpen = false;
    }







    void Update()
    {


        if (stopwatch.ElapsedMilliseconds < 1000/targetFrameRate)
        {
            return;
        }
        stopwatch.Restart();
        
        // UnityEngine.Debug.Log("Streaming FPS:" + 1.0f / Time.deltaTime);
        // ApplicationManager applicationManager = FindObjectOfType<ApplicationManager>();
        // // move MainCamera to this location
        // applicationManager.MainCamera.transform.position = transform.position;
        // applicationManager.MainCamera.transform.rotation = transform.rotation;
        // UnityEngine.Debug.Log("set position of: " + applicationManager.MainCamera);
        // // copy fov
        // applicationManager.MainCamera.GetComponent<Camera>().fieldOfView = cameraToCapture.fieldOfView;

        // Read the pixel data from the RenderTexture
        RenderTexture.active = rt;
        cameraToCapture.targetTexture = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;


        // Apply blur effect
        blurMaterial.SetFloat("_BlurSize", blurSize);
        Graphics.Blit(tex, tempRT, blurMaterial); // Apply blur to tempRT
        RenderTexture.active = tempRT; // Set tempRT as active to read the blurred result
        tex.ReadPixels(new Rect(0, 0, tempRT.width, tempRT.height), 0, 0); // Copy blurred result to tex
        tex.Apply(); // Update the Texture2D
        RenderTexture.active = null; // Clear active RenderTexture

        // Copy the pixel data to a NativeArray
        NativeArray<Color32> pixels = tex.GetRawTextureData<Color32>();

        // If dumping frames to video file, save it
        if( isFileOpen)
        {
            AddFrameToVideo(pixels);
        }

        frameNumber++;

        // the following code is necessary to avoid stuttering due to the garbage collector.
        unsafe
        {
            // acquire pointer to the shared memory region which we will write to
            byte* pointer = null;
            // Lock the shared memory buffer
            accessor.SafeMemoryMappedViewHandle.AcquirePointer(ref pointer);
            try
            {
                // Write width, height, and frameNumber at the start
                int* intPointer = (int*)pointer;
                intPointer[0] = width;        // Write width
                intPointer[1] = height;       // Write height
                intPointer[2] = frameNumber;  // Write frame number

                // compute total size to write
                int numBytes = pixels.Length * sizeof(Color32);
                // Get pointer to the image data (after 3 integers)
                byte* imagePointer = pointer + 3 * sizeof(int);
                // get pointer to the native array which we will copy from
                void* pixelsPtr = NativeArrayUnsafeUtility.GetUnsafePtr(pixels);
                // Copy the pixel data directly to the shared memory buffer
                Buffer.MemoryCopy(pixelsPtr, imagePointer, numBytes, numBytes);
            }
            finally
            {
                // Unlock the shared memory buffer
                accessor.SafeMemoryMappedViewHandle.ReleasePointer();
            }
        }

        // Signal the consumer that a new frame is ready
        try
        {
            frameReadyEvent.Set();
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"Failed to signal EventWaitHandle: {ex.Message}");
        }

        // below is the old code that stuttered
        // Color32[] pixelsArray = pixels.ToArray(); // Convert to Color32[]
        // accessor.WriteArray(0, pixelsArray, 0, pixelsArray.Length);
    }

    void OnDestroy()
    {
        if (rt != null)
        {
            rt.Release();
        }
        if (isFileOpen)
        {
            CloseFile();
        }
        // Clean up resources
        if (accessor != null)
        {
            accessor.Dispose();
        }
        if (frameReadyEvent != null)
        {
            frameReadyEvent.Dispose();
        }

    }

    void OnApplicationQuit()
    {
        // Dispose of the memory-mapped file and view accessor when the application quits
        accessor.Dispose();
        mmf.Dispose();
    }

    void AddFrameToVideo_RGB(NativeArray<Color32> pixels)
    {
        unsafe
        {
            // Allocate on the heap instead of the stack
            byte[] frameDataArray = new byte[pixels.Length * 3]; // RGB24
            fixed (byte* frameData = frameDataArray)
            {
                for (int i = 0; i < pixels.Length; i++)
                {
                    frameData[i * 3] = pixels[i].r;
                    frameData[i * 3 + 1] = pixels[i].g;
                    frameData[i * 3 + 2] = pixels[i].b;
                }

                AVPacket* packet = ffmpeg.av_packet_alloc();
                ffmpeg.av_new_packet(packet, pixels.Length * 3);

                // Copy data from managed array to unmanaged memory
                Marshal.Copy(frameDataArray, 0, (IntPtr)packet->data, pixels.Length * 3);

                packet->pts = frameCount;
                packet->dts = frameCount;

                if (ffmpeg.av_interleaved_write_frame(formatContext, packet) < 0)
                {
                    UnityEngine.Debug.LogError("Error while writing video frame");
                }

                ffmpeg.av_packet_free(&packet);
                frameCount++;
            }
        }
    }

 

}