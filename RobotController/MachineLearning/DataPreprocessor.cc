#include <iostream>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "../VisionPreprocessor.h"  // Assuming you have this header file
#include "../RobotConfig.h"
#include <fstream>
#include <nlohmann/json.hpp>  // Assuming you have the nlohmann JSON library

#define CROP_SIZE 128

int main()
{
    // Set input and output directories
    const std::string inputDir = "./TrainingData/TrainingInputs/";
    const std::string outputDir = "./TrainingData/TrainingInputsProjected/";
    const std::string keysDir = "./TrainingData/TrainingKeys/";

    loadGlobalVariablesFromFile("../RobotConfig.txt");

    // Create an instance of VisionPreprocessor
    VisionPreprocessor preprocessor;

    std::cout << "Processing images..." << std::endl;

    // Iterate over all files in the directory
    for (const auto& entry : std::filesystem::directory_iterator(inputDir))
    {
        std::cout << "Processing " << entry.path() << std::endl;
        // Check if it's a regular file (not a subdirectory)
        if (entry.is_regular_file())
        {
            std::string filepath = entry.path().string();
            std::string filename = entry.path().filename().string();
            std::string keypath = keysDir + filename.substr(0, filename.find_last_of('.')) + ".json";


            // Read the image
            cv::Mat image = cv::imread(filepath, cv::IMREAD_COLOR);
            std::cout << "image dimensions: " << image.cols << "x" << image.rows << std::endl;
            if (image.empty())
            {
                std::cerr << "Failed to load image: " << filepath << std::endl;
                continue;
            }
            std::cout << "src.total(): " << image.total() << std::endl;

            // Process the image
            cv::Mat processedImage;
            preprocessor.Preprocess(image, processedImage);


            // load file at keypath as json (format: {"position":{"x":606.1824951171875,"y":321.9252014160156},"rotation":85.47148895263672})
            std::ifstream keyFile(keypath);
            if (!keyFile.is_open())
            {
                std::cerr << "Failed to open key file: " << keypath << std::endl;
                continue;
            }
            nlohmann::json keyJson;
            keyFile >> keyJson;

            // get the position as a cv::Point2f
            cv::Point2f position(keyJson["position"]["x"], keyJson["position"]["y"]);

            // transform that point using the VisionPreprocessor::TransformPoint method to get location on processedImage
            cv::Point2f transformedPoint = preprocessor.TransformPoint(position);  // Assuming VisionPreprocessor has a TransformPoint method

            // add random shift up to 10 pixels in each direction
            const int RANDOM_SHIFT_MAX = 10;
            transformedPoint.x += rand() % (2 * RANDOM_SHIFT_MAX) - RANDOM_SHIFT_MAX;
            transformedPoint.y += rand() % (2 * RANDOM_SHIFT_MAX) - RANDOM_SHIFT_MAX;

            // crop processedImage to CROP_SIZE by CROP_SIZE around that point
            cv::Rect roi(transformedPoint.x - CROP_SIZE/2, transformedPoint.y - CROP_SIZE/2, CROP_SIZE, CROP_SIZE);

            // make sure the roi is within the bounds of the image
            roi.x = std::max(0, roi.x);
            roi.y = std::max(0, roi.y);
            roi.width = std::min(processedImage.cols - roi.x, roi.width);
            roi.height = std::min(processedImage.rows - roi.y, roi.height);

            cv::Mat croppedImage = processedImage(roi);

            // fill in remaining with black so that the image is always CROP_SIZE by CROP_SIZE
            cv::Mat black = cv::Mat::zeros(CROP_SIZE, CROP_SIZE, CV_8UC3);
            croppedImage.copyTo(black(cv::Rect(0, 0, croppedImage.cols, croppedImage.rows)));

            // Save the processed image
            if (!cv::imwrite(outputDir + filename, black))
            {
                std::cerr << "Failed to save processed image: " << outputDir + filename << std::endl;
            }
        }
    }

    return 0;
}
