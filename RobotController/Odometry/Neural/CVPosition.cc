#include "CVPosition.h"
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "../../UIWidgets/ClockWidget.h"
#include "../../CameraReceiver.h"
#include <nlohmann/json.hpp>
#include "../../RobotConfig.h"
#include <cstdlib> // For std::system

#define VELOCITY_RESET_FRAMES 100

// Function to create shared memory and return a pointer to it
void *createSharedMemory(std::string name, int size)
{
    std::wstring sharedFileNameW(name.begin(), name.end());
    LPCSTR sharedFileNameLPCWSTR = name.c_str();

    HANDLE hMapFile = CreateFileMappingA(
        INVALID_HANDLE_VALUE,   // Use paging file - shared memory
        NULL,                   // Default security attributes
        PAGE_READWRITE,         // Read/write access
        0,                      // Maximum object size (high-order DWORD)
        size,                   // Maximum object size (low-order DWORD)
        sharedFileNameLPCWSTR); // Name of the mapping object

    if (hMapFile == NULL)
    {
        std::cerr << "Could not create file mapping object: " << GetLastError() << std::endl;
        return nullptr;
    }

    void *pBuf = MapViewOfFile(hMapFile,            // Handle to mapping object
                               FILE_MAP_ALL_ACCESS, // Read/write permission
                               0,
                               0,
                               size);

    if (pBuf == nullptr)
    {
        std::cerr << "Could not map view of file: " << GetLastError() << std::endl;
        CloseHandle(hMapFile); // Handle cleanup
        return nullptr;
    }

    return pBuf;
}

void CVPosition::_InitSharedImage()
{
    int type = CV_8UC1; // Example type: 8-bit unsigned int with 3 channels
    int elementSize = CV_ELEM_SIZE(type); // Size of element depending on the cv::Mat type
    int imgSize = width * height * elementSize;

    // Create shared memory
    void* sharedData = createSharedMemory(memName, imgSize);

    // Initialize cv::Mat with the shared memory data
    sharedImage = cv::Mat(height, width, type, sharedData);
}

void CVPosition::_StartPython()
{
    return;
    // return;
    const std::string venv_path = "venv";            // Path to the venv directory
    const std::string script_path = "CVPosition.py"; // Path to the Python script
    // Create command string for Windows
    std::string command = "cmd.exe /C \"cd MachineLearning && " + venv_path + "\\Scripts\\activate && python " + script_path + " > NUL 2>&1\"";

    _pythonThread = std::thread([command]() {
        // Run the command
        int result = std::system(command.c_str());
        if (result != 0)
        {
            std::cerr << "Failed to run CVPosition.py" << std::endl;
        }
    });
}

CVPosition::CVPosition(ICameraReceiver *videoSource) : _pythonSocket("11116"), _lastData(CVPositionData()), OdometryBase(videoSource)
{
    _InitSharedImage();

    _StartPython();
}

CVPosition::~CVPosition()
{
}

// Start the thread
// Returns false if already running
void CVPosition::_ProcessNewFrame(cv::Mat frame, double frameTime)
{
    // write the id to the frame
    for (int i = 0; i < 4; i++)
    {
        frame.at<uchar>(0, i) = (frameID >> (8 * i)) & 0xFF;
    }

    // the next pixels are the time in milliseconds
    uint32_t timeMillis = (uint32_t) (frameTime * 1000.0);

    // add the time to the frame
    for (int i = 0; i < 4; i++)
    {
        frame.at<uchar>(0, i + 4) = (timeMillis >> (8 * i)) & 0xFF;
    }

    if (NEURAL_BRIGHTNESS_ADJUST != 0)
    {
        // brightness adjust +1
        cv::Mat adjusted;

        // Now adjust the new imagexz
        frame.convertTo(adjusted,  CV_16U, 1 + NEURAL_BRIGHTNESS_ADJUST / 10.0, 0); // Multiply by factor, no offset

        // Ensure pixel values are within [0, 255]
        cv::threshold(adjusted, adjusted, 255, 255, cv::THRESH_TRUNC); // Cap at 255

        // Set it back to 8 bit
        adjusted.convertTo(frame, CV_8U);


        // cv::imshow("adjusted", frame);
        // cv::waitKey(1);

    }

    // copy to shared memory
    frame.copyTo(sharedImage);

    bool pythonResponded = true;
    CVPositionData data = _GetDataFromPython(pythonResponded);

    if (!pythonResponded) {
        return;
    }

    // // make sure the data is newer than the last data
    // if (data.center.x <= 0 ||
    //     data.center.y <= 0 || data.center.x >= frame.cols ||
    //     data.center.y >= frame.rows || std::isnan(data.center.x) ||
    //     std::isnan(data.center.y) || std::isinf(data.center.x) ||
    //     std::isinf(data.center.y)) {
    //   data.valid = false;
    //   std::cout << "invalid 0" << std::endl;
    // }

    cv::Point2f velocity = cv::Point2f(0, 0);

    if (data.valid) {
        // If lastData isn't invalid, set to current position (0 velocity)
        if (_lastData.center.x <= 0 || _lastData.center.y <= 0 || _lastData.center.x >= frame.cols ||
            _lastData.center.y >= frame.rows) {
            _lastData.center = data.center;
        }

        _lastDataMutex.lock();
        // if we ever skip too much distance, reset to invalid
        if (cv::norm(data.center - _lastData.center) > _max_distance_thresh ||
            data.frameID > _lastData.frameID + VELOCITY_RESET_FRAMES) {
          data.valid = false;
          std::cout << "invalid 1" << std::endl;

          std::cout << "distance: " << cv::norm(data.center - _lastData.center) << std::endl;
          std::cout << "frameID: " << data.frameID << " lastFrameID: " << _lastData.frameID << std::endl;
        }
        // copy over the data
        _lastData = data;
        _lastVelocity = velocity;
        _lastDataMutex.unlock();
    }
    // if the data is valid, increment the valid frames counter
    if (data.valid) {
        _valid_frames_counter++;
    }

    data.valid = data.valid && _valid_frames_counter >= 3;

    _UpdateData(data, velocity);

}

void CVPosition::_UpdateData(CVPositionData data, cv::Point2f velocity)
{
    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    // Clear curr data
    _currDataRobot.Clear();
    _currDataRobot.id = data.frameID;             // Set id based on the frame
    _currDataRobot.frameID = frameID; 
    double posTime = data.time_millis / 1000.0;    // Set to new time
    
    if (data.valid) {
        cv::Rect defaultRect(static_cast<int>(data.center.x - 10), static_cast<int>(data.center.y - 10), 20, 20);
        _currDataRobot.SetPosition(data.center, velocity, defaultRect, posTime);
    } else {
        _currDataRobot.InvalidatePosition();
    }
    _currDataRobot.InvalidateAngle(); // always invalid angle since just position

    _currDataOpponent.Clear();
    _currDataOpponent.InvalidatePosition();
    _currDataOpponent.InvalidateAngle();
}

/**
 * there is a 5th field, which is the id
*/
std::vector<int> CVPosition::GetBoundingBox(int* outFrameID)
{
    _lastDataMutex.lock();
    CVPositionData actualData = _lastData;
    _lastDataMutex.unlock();

    // copy the actual id to the outFrameID
    if (outFrameID != nullptr)
    {
        *outFrameID = actualData.frameID;
    }

    return actualData.boundingBox;
}

cv::Point2f CVPosition::GetCenter(int* outFrameID)
{
    _lastDataMutex.lock();
    CVPositionData actualData = _lastData;
    _lastDataMutex.unlock();

    // copy the actual id to the outFrameID
    if (outFrameID != nullptr)
    {
        *outFrameID = actualData.frameID;
    }

    return actualData.center;
}

CVPositionData CVPosition::_GetDataFromPython(bool& outPythonResponded)
{
    // receive from socket
    std::string data = _pythonSocket.receive();

    if (data == "")
    {
        CVPositionData ret = _lastData;
        ret.valid = false;
        std::cout << "invalid -2" << std::endl;
        outPythonResponded = false;
        return ret;
    }
    outPythonResponded = true;

    // parse using json
    nlohmann::json j = nlohmann::json::parse(data);
    
    // get "bounding_box"
    nlohmann::json boundingBox = j["bounding_box"];

    // check if it's a string and equal to "invalid"
    if (boundingBox.is_string())
    {
        CVPositionData ret = _lastData;
        ret.valid = false;
        std::cout << "invalid -3" << std::endl;
        return ret;
    }

    // it's an array of floats, so we can just cast it to a vector
    std::vector<float> boundingBoxVec = boundingBox.get<std::vector<float>>();

    // convert to ints
    std::vector<int> intBoundingBox = {};
    for (int i = 0; i < boundingBoxVec.size(); i++)
    {
        intBoundingBox.push_back((int)boundingBoxVec[i]);
    }

    // get the "conf" it's a float
    float conf = j["conf"].get<float>();

    // get the "frame_id" it's an int
    uint32_t id = j["frame_id"].get<uint32_t>();

    // parse the time in milliseconds
    uint32_t time_milliseconds = j["time_milliseconds"].get<uint32_t>();

    CVPositionData ret;
    ret.boundingBox = intBoundingBox;
    ret.frameID = id;
    ret.time_millis = time_milliseconds;
    ret.center = cv::Point2f(intBoundingBox[0], intBoundingBox[1]);
    ret.valid = conf >= NN_MIN_CONFIDENCE;
    std::cout << "conf valid: " << ret.valid << std::endl;

    return ret;
}