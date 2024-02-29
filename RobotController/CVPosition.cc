#include "CVPosition.h"
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "UIWidgets/ClockWidget.h"
#include "CameraReceiver.h"

CVPosition::CVPosition()
{
    // load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
    _net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    _net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    modelShape = cv::Size(1280, 1280);

    workerThread = std::thread([this]() {
        ICameraReceiver& camera = ICameraReceiver::GetInstance();
        long last_id = -1;
        cv::Mat frame;
        std::vector<int> boundingBox;
        while (true)
        {
            long id = camera.GetFrame(frame, last_id); // Block until a new frame is present which may be forever
            last_id = id;
            boundingBox = ComputeRobotPosition(frame);
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

std::vector<int> CVPosition::ComputeRobotPosition(cv::Mat& fieldImage)
{
    static ClockWidget clock{"CVPosition"};
    clock.markStart();


    static ClockWidget preprocessClock{"CVPosition preprocess"};
    preprocessClock.markStart();

    cv::Mat img = fieldImage;

    // if the image is not grayscale, convert it to grayscale
    if (img.type() != CV_8UC1)
    {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }

    if (img.size() != cv::Size(1280, 1280))
    {
        static ClockWidget resizeClock{"CVPosition resize"};
        resizeClock.markStart();
        cv::resize(img, img, cv::Size(1280, 1280));
        resizeClock.markEnd();
    }

    static ClockWidget mergeClock{"CVPosition merge and convert"};
    mergeClock.markStart();
    cv::merge(std::vector<cv::Mat>{img, img, img}, img);

    img.convertTo(img, CV_32F);
    img /= 255.0;
    mergeClock.markEnd();

    static ClockWidget blobClock{"CVPosition blobFromImage & set input"};
    blobClock.markStart();
    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1.0);

    _net.setInput(blob);
    blobClock.markEnd();

    std::vector<cv::Mat> outputs;
    preprocessClock.markEnd();

    static ClockWidget forardClock{"CVPosition _net.forward"};
    forardClock.markStart();
    _net.forward(outputs, _net.getUnconnectedOutLayersNames());
    forardClock.markEnd();

    cv::Mat predictions = outputs[0];

    int rows = predictions.size[2];
    int dimensions = predictions.size[1];

    predictions = predictions.reshape(1, dimensions);
    cv::transpose(predictions, predictions);

    float *data = (float *)predictions.data;

    int x;
    int y;
    int w;
    int h;

    float max_score = 0;


    for (int i = 0; i < rows; i++)
    {
        float score = data[4];
        if (score > max_score)
        {
            max_score = score;
            x = int(data[0]);
            y = int(data[1]);
            w = int(data[2]);
            h = int(data[3]);
        }
        data += dimensions;
    }

    std::vector<int> boundingBox = {x, y, w, h};

    clock.markEnd();
    return boundingBox;
}