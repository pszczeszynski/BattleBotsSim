#include "CVPosition.h"
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "UIWidgets/ClockWidget.h"
#include "CameraReceiver.h"
#include <nlohmann/json.hpp>

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

CVPosition::CVPosition() : _pythonSocket("11116")
{
    // init bounding box to all 0s
    _boundingBox = {0, 0, 0, 0};

    _InitSharedImage();


    workerThread = std::thread([this]() {
        ICameraReceiver& camera = ICameraReceiver::GetInstance();
        long last_id = -1;
        std::vector<int> boundingBox;
        while (true)
        {
            // spin until new frame is ready
            if (!camera.NewFrameReady(last_id))
            {
                continue;
            }

            // get the next frame
            last_id = camera.GetFrame(sharedImage, last_id);

            boundingBox = ComputeRobotPosition();

            // copy over the bounding box
            boundingBoxMutex.lock();
            _boundingBox = boundingBox;
            boundingBoxMutex.unlock();
        }
    });
}

CVPosition& CVPosition::GetInstance()
{
    static CVPosition instance;
    return instance;
}

std::vector<int> CVPosition::GetBoundingBox()
{
    std::vector<int> boundingBox;
    boundingBoxMutex.lock();
    boundingBox = _boundingBox;
    boundingBoxMutex.unlock();
    return boundingBox;
}

std::vector<int> CVPosition::ComputeRobotPosition()
{
    // receive from socket
    std::string data = _pythonSocket.receive();

    if (data == "")
    {
        return _boundingBox;
    }

    // parse using json
    nlohmann::json j = nlohmann::json::parse(data);
    
    // get "bounding_box"
    nlohmann::json boundingBox = j["bounding_box"];
    // it's an array of floats, so we can just cast it to a vector
    std::vector<float> boundingBoxVec = boundingBox.get<std::vector<float>>();

    // convert to ints
    std::vector<int> intBoundingBox;
    for (int i = 0; i < boundingBoxVec.size(); i++)
    {
        intBoundingBox.push_back((int)boundingBoxVec[i]);
    }

    // get the "conf" it's a float
    float conf = j["conf"].get<float>();

    return intBoundingBox;
}