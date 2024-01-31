#include "CVPosition.h"


CVPosition::CVPosition()
{
    // load the model
    _net = cv::dnn::readNetFromONNX(MODEL_PATH);
}

CVPosition& CVPosition::GetInstance()
{
    static CVPosition instance;
    return instance;
}

cv::Point2f CVPosition::ComputeRobotPosition(cv::Mat& fieldImage)
{
    // todo

    return cv::Point2f(0, 0);
}