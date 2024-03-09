#include "CVPosition.h"
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "../../UIWidgets/ClockWidget.h"
#include "../../CameraReceiver.h"
#include <nlohmann/json.hpp>
#include "../../RobotConfig.h"

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

CVPosition::CVPosition(ICameraReceiver *videoSource) : _pythonSocket("11116"), _lastData(CVPositionData()), OdometryBase(videoSource)
{
    _InitSharedImage();
}

// Start the thread
// Returns false if already running
void CVPosition::_ProcessNewFrame(cv::Mat frame, double frameTime)
{
    // writ the id to the frame
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

    // copy to shared memory
    frame.copyTo(sharedImage);

    CVPositionData data = _GetDataFromPython();

    // make sure the data is newer than the last data
    if (data.frameID <= _lastData.frameID)
    {
        return;
    }

    // compute velocity
    // If lastpos isn't invalid, set to current position (0 velocity)
    if( (lastPos.x < 0) || (lastPos.y < 0) )
    {
        lastPos = data.center;
    }

    if (abs(norm(data.center - lastPos)) > _min_distance_threshold)
    {
        _valid_frames_counter = 0;
    }
    else
    {
        _valid_frames_counter++;
    }

    cv::Point2f velocity = (data.center - lastPos) / (double) ((data.time_millis - _lastData.time_millis) / 1000.0);
    lastPos = data.center;

    if (data.frameID > _lastData.frameID + VELOCITY_RESET_FRAMES)
    {
        // force the velocity to be 0 if the data is too old
        velocity = cv::Point2f(0, 0);
        data.valid = false;
    }

    data.valid = data.valid && _valid_frames_counter >= 10;

    _UpdateData(data, velocity);

    // copy over the data
    _lastDataMutex.lock();
    _lastData = data;
    _lastDataMutex.unlock();
}

void CVPosition::_UpdateData(CVPositionData data, cv::Point2f velocity)
{
    // Get unique access
    std::unique_lock<std::mutex> locker(_updateMutex);

    // Clear curr data
    _currDataRobot.Clear();
    _currDataRobot.isUs = true; // Make sure this is set
    _currDataRobot.robotPosValid = data.valid;
    _currDataRobot.id = data.frameID;             // Set id based on the frame
    _currDataRobot.time = data.time_millis / 1000.0;    // Set to new time
    _currDataRobot.robotPosition = data.center; // Set new position
    _currDataRobot.robotVelocity = velocity;
    
    // compute velocity using the last position


    _currDataOpponent.Clear();
    _currDataOpponent.isUs = false; // Make sure this is set
    _currDataOpponent.robotPosValid = false;

    // Set our rotation
    _currDataRobot.robotAngleValid = false;
    
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

CVPositionData CVPosition::_GetDataFromPython()
{
    // receive from socket
    std::string data = _pythonSocket.receive();

    if (data == "")
    {
        CVPositionData ret = _lastData;
        ret.valid = false;
        return ret;
    }

    // parse using json
    nlohmann::json j = nlohmann::json::parse(data);
    
    // get "bounding_box"
    nlohmann::json boundingBox = j["bounding_box"];
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
    std::cout << "conf: " << conf << std::endl;
    ret.valid = conf >= NN_MIN_CONFIDENCE;

    return ret;
}