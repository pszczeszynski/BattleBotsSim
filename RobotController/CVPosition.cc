#include "CVPosition.h"
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <iostream>

CVPosition::CVPosition()
{
    // load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
    modelShape = cv::Size(1280, 1280);
}
CVPosition& CVPosition::GetInstance()
{
    static CVPosition instance;
    return instance;
}

std::vector<int> CVPosition::ComputeRobotPosition(cv::Mat& fieldImage)
{
    cv::Mat img = fieldImage;

    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    cv::merge(std::vector<cv::Mat>{img, img, img}, img);
    cv::resize(img, img, cv::Size(1280, 1280));

    img.convertTo(img, CV_32F);
    img /= 255.0;

    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1.0);
    _net.setInput(blob);

    std::vector<cv::Mat> outputs;
    _net.forward(outputs, _net.getUnconnectedOutLayersNames());

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

    for (int i = 0; i < rows; i++) {
        float score = data[4];
        if (score > max_score) { 
            max_score = score;
            x = int(data[0]);
            y = int(data[1]);
            w = int(data[2]);
            h = int(data[3]);
        }
        data += dimensions;
    }

    x = x * 720 / 1280;
    y = y * 720 / 1280;
    w = w * 720 / 1280;
    h = h * 720 / 1280;

    std::vector<int> boundingBox = {x, y, w, h};

    return boundingBox;
}