//=============================================================================
// Copyright (c) 2001-2023 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
*  @example Acquisition.cpp
*
*  @brief Acquisition.cpp shows how to acquire images. It relies on
*  information provided in the Enumeration example. Also, check out the
*  ExceptionHandling and NodeMapInfo examples if you haven't already.
*  ExceptionHandling shows the handling of standard and Spinnaker exceptions
*  while NodeMapInfo explores retrieving information from various node types.
*
*  This example touches on the preparation and cleanup of a camera just before
*  and just after the acquisition of images. Image retrieval and conversion,
*  grabbing image data, and saving images are all covered as well.
*
*  Once comfortable with Acquisition, we suggest checking out
*  AcquisitionMultipleCamera, NodeMapCallback, or SaveToAvi.
*  AcquisitionMultipleCamera demonstrates simultaneously acquiring images from
*  a number of cameras, NodeMapCallback serves as a good introduction to
*  programming with callbacks and events, and SaveToAvi exhibits video creation.
*
*  Please leave us feedback at: https://www.surveymonkey.com/r/TDYMVAPI
*  More source code examples at: https://github.com/Teledyne-MV/Spinnaker-Examples
*  Need help? Check out our forum at: https://teledynevisionsolutions.zendesk.com/hc/en-us/community/topics
*/

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <conio.h>
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;

// opencv function to updat escreen without the windows waitKey delay
bool DoEvents()
{
    MSG msg;
    BOOL result;

    while (::PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE))
    {
        result = ::GetMessage(&msg, NULL, 0, 0);
        if (result == 0) // WM_QUIT
        {
            ::PostQuitMessage((int) msg.wParam);
            return false;
        }
        else if (result == -1)
            return true;    //error occured
        else
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
        }
    }

    return true;
}

// This function acquires and saves 10 images from a device.
int AcquireImages(CameraPtr pCam)
{
    int result = 0;

    try
    {
        // Get Image dimensions        
        int image_width = (int) pCam->Width.GetValue();   // Max 1440 for this camera
        int image_height = (int) pCam->Height.GetValue();   // Max 1080 for this camera

        // Create a window
        // cv::namedWindow("Camera Stream", cv::WINDOW_NORMAL);

        // Start acquisition
        pCam->BeginAcquisition();

        auto start = std::chrono::high_resolution_clock::now();
        float framecount = 0;
        int count = 0;
        cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

        while (!_kbhit())
        {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);

            if (duration.count() >= 1000)
            {
                cout << "fps = " << (framecount / float(duration.count()) * 1000.0) << endl;
                framecount = 0;
                start = now;
            }

            // cout << "TIME: " << duration.count() << endl;

            framecount += 1;

            // Get Next Image
            ImagePtr pResultImage = pCam->GetNextImage(500); // 0.5s timeout


            // Get char data
            unsigned char* img_data = (unsigned char*)pResultImage->GetData();             
            
            // Create a Mat
            cv::Mat bayerImage(image_height, image_width, CV_8UC1, img_data);

            cv::Mat colorImage;
            cv::cvtColor(bayerImage, colorImage, cv::COLOR_BayerRGGB2BGR);
            //string output = "output";
            //output += std::to_string(count);
            //output += ".jpg";
   

            //cv::imwrite(output, colorImage);
            count += 1;

            cv::imshow("Video", colorImage);

            DoEvents();
            // cv::waitKey(1);
           

            // Show Image 
            // cv::imshow("Camera Stream", rgbImage);



            // If not complete, get next
            //if (!pResultImage->IsIncomplete()) 
            //{ 
            //    cout << "incomplete" << endl;
            //    continue; 
            //}
            //else
            //{
            //    cout << ".";
            // }




            // Assuming image is 640 x 480
            // data[0] = Row 0, Column 0 = red pixel (R)
            // data[1] = Row 0, Column 1 = green pixel (G)
            // data[640] = Row 1, Column 0 = green pixel (G)
            // data[641] = Row 1, Column 1 = blue pixel (B)
            

        }

        pCam->EndAcquisition();        
        
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return result;
}



int ConfigureCamera(CameraPtr pCam)
{
    //
    // NOTE: In general, a fault will occur if you disable a function and try to set its options
    //
    try
    {
        pCam->Init();

        // Disable heartbeat (a GiGe standard requiring program to reconfirm periodically its alive)
        pCam->GevGVCPHeartbeatDisable();

        // ******** General Settings ***************
        // Set exposure mode
        //pCam->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);  // turn off auto
        pCam->ExposureMode.SetValue(ExposureMode_Timed); // set it to fixed time
        pCam->ExposureTime.SetValue(10002.0); // Set to 2ms. Longer is less noisy

        // Set Gain Mode
        pCam->GainAuto.SetValue(GainAutoEnums::GainAuto_Once); // Turn off auto gain
        // pCam->Gain.SetValue(8.0); // Set gain in dB (between 0 and 47.99)

        // Turn off Gama correction
        pCam->GammaEnable.SetValue(false);
        // pCam->Gamma.SetValue(1.0f, false); // Gamma correction if enabled. 0 to 4 nominal range

        // Turn off white balance
        pCam->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Off);

        // Allocate all bandwidth to this one camera 125000000 is max, reduce to 122000000 for margin
        pCam->DeviceLinkThroughputLimit.SetValue(122000000);

        // Turn off trigger
        pCam->TriggerMode.SetValue(TriggerMode_Off);

        // Turn off sequencer
        pCam->SequencerConfigurationMode.SetValue(SequencerConfigurationMode_Off);
        pCam->SequencerFeatureEnable.SetValue(false);
        // pCam->SequencerMode.SetValue( SequencerModeEnums::SequencerMode_Off);


        // ******** Image Settings ***************
        // Reducing image size saves bandwidth for faster refresh rate
        // We need to reset offsets first before changing this
        pCam->OffsetX.SetValue(0); 
        pCam->OffsetY.SetValue(0); 

        // Now setting width/height should cause issues
        int my_width = 1440;
        int my_height = 600;

        pCam->Width.SetValue(my_width);   // Max 1440 for this camera
        pCam->Height.SetValue(my_height);   // Max 1080 for this camera

        pCam->OffsetX.SetValue((1440 - my_width) / 2); // Set to center of ccd
        pCam->OffsetY.SetValue((1080 - my_height) / 2); // Set to center of ccd
        
        pCam->IspEnable.SetValue(false);  // Turn off image processing
        pCam->AdcBitDepth.SetValue(AdcBitDepth_Bit8);  // Set to 8-bit color resolution

        // ******* Output Data Settings *******

        pCam->PixelFormat.SetValue(PixelFormat_BayerRG8); // Only some of the formats work, this is one of them and is fast.

        // Compression may be useful, but not tested for delay
        pCam->ImageCompressionMode.SetValue(ImageCompressionModeEnums::ImageCompressionMode_Off);

        // Set Acquisition to continouse
        pCam->AcquisitionMode.SetValue(AcquisitionMode_Continuous);

        //
        // Make it always return latest image (This option is not available via the easy access mode)

        Spinnaker::GenApi::INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();
        CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        ptrHandlingMode->SetIntValue(StreamBufferHandlingMode_NewestOnly);


        // Save settings to user register 0
        // pCam->UserSetSelector = UserSetSelectorEnums::UserSetSelector_UserSet0;
        // pCam->UserSetSave();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "********* CONFIG ERROR *********" << endl;;
        cout << "Error: " << e.what() << endl << endl;;
        return -1;
    }

    return 0;
}

// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    // Get the first camera
    CameraPtr pCam = nullptr;
    pCam = camList.GetByIndex(0);
   
    // Initialize
    if (ConfigureCamera(pCam) < 0)
    {
        // Failed, exit
        pCam = nullptr;

        // Release system
        system->ReleaseInstance();
        getchar();

        return 0;
    }

    // Ready to get images:
    AcquireImages(pCam);

    // Done with it
    pCam->DeInit();
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();

    return 0;
}