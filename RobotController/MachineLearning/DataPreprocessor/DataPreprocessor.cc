#include <iostream>
#include <string>
#include <filesystem>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "../../VisionPreprocessor.h"  // Assuming you have this header file
#include "../../RobotConfig.h"
#include <fstream>
#include <nlohmann/json.hpp>  // Assuming you have the nlohmann JSON library

#define CROP_SIZE 128

#define POSITION
#define OVERWRITE

int main()
{
    // Set input and output directories
    const std::string inputDir = "../TrainingData/TrainingInputs/";
    #ifdef POSITION
    const std::string outputDir = "../TrainingData/TrainingInputsPosition/";
    #else
    const std::string outputDir = "../TrainingData/TrainingInputsProjected/";
    #endif
    const std::string keysDir = "../TrainingData/TrainingKeys/";

    loadGlobalVariablesFromFile("../../RobotConfig.txt");

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
            
            float h = keyJson["position"]["w"];
            float w = keyJson["position"]["z"];

            // transform that point using the VisionPreprocessor::TransformPoint method to get location on processedImage
            cv::Point2f transformedPoint = preprocessor.TransformPoint(position);  // Assuming VisionPreprocessor has a TransformPoint method

            cv::Point2f tl(position.x - w/2, position.y - h/2);
            cv::Point2f tr(position.x + w/2, position.y - h/2);
            cv::Point2f bl(position.x - w/2, position.y + h/2);
            cv::Point2f br(position.x + w/2, position.y + h/2);

            cv::Point2f tl_transformed = preprocessor.TransformPoint(tl);
            cv::Point2f tr_transformed = preprocessor.TransformPoint(tr);
            cv::Point2f bl_transformed = preprocessor.TransformPoint(bl);
            cv::Point2f br_transformed = preprocessor.TransformPoint(br);

            float transformed_w = std::max({tl_transformed.x, tr_transformed.x, bl_transformed.x, br_transformed.x}) - std::min({tl_transformed.x, tr_transformed.x, bl_transformed.x, br_transformed.x});
            float transformed_h = std::max({tl_transformed.y, tr_transformed.y, bl_transformed.y, br_transformed.y}) - std::min({tl_transformed.y, tr_transformed.y, bl_transformed.y, br_transformed.y});

            // add transformedPoint to json
            keyJson["transformedPoint"]["x"] = transformedPoint.x;
            keyJson["transformedPoint"]["y"] = transformedPoint.y;
            keyJson["transformedPoint"]["z"] = transformed_w;
            keyJson["transformedPoint"]["w"] = transformed_h;

            // add random shift up to 10 pixels in each direction
            const int RANDOM_SHIFT_MAX = 10;

            cv::Point2f pointWithRandomization = transformedPoint;
            pointWithRandomization.x += rand() % (2 * RANDOM_SHIFT_MAX) - RANDOM_SHIFT_MAX;
            pointWithRandomization.y += rand() % (2 * RANDOM_SHIFT_MAX) - RANDOM_SHIFT_MAX;


#ifndef POSITION
            // crop processedImage to CROP_SIZE by CROP_SIZE around that point
            cv::Rect roi(pointWithRandomization.x - CROP_SIZE/2, pointWithRandomization.y - CROP_SIZE/2, CROP_SIZE, CROP_SIZE);

            // make sure the roi is within the bounds of the image
            roi.x = std::max(0, roi.x);
            roi.y = std::max(0, roi.y);
            roi.width = std::min(processedImage.cols - roi.x, roi.width);
            roi.height = std::min(processedImage.rows - roi.y, roi.height);

            // fill in remaining with black so that the image is always CROP_SIZE by CROP_SIZE
            cv::Mat croppedImageSized = cv::Mat::zeros(CROP_SIZE, CROP_SIZE, CV_8UC3);

            // Check if the roi is completely outside the bounds of the image
            if (roi.width <= 0 || roi.height <= 0)
            {
                std::cerr << "Warning: Crop is completely outside the image bounds!" << std::endl;
            }
            else
            {
                // Copy the cropped image to the black image
                cv::Mat croppedImage = processedImage(roi);
                croppedImage.copyTo(croppedImageSized(cv::Rect(0, 0,
                                                               croppedImage.cols,
                                                               croppedImage.rows)));
            }

#else
            cv::Mat& croppedImageSized = processedImage;
#endif

            // Save the processed image
            if (!cv::imwrite(outputDir + filename, croppedImageSized))
            {
                std::cerr << "Failed to save processed image: " << outputDir + filename << std::endl;
            }

            // save the json
            std::ofstream keyFileOut(keypath);
            if (!keyFileOut.is_open())
            {
                std::cerr << "Failed to open key file: " << keypath << std::endl;
                continue;
            }

            // write the json to the file
            keyFileOut << keyJson.dump(4);

            // close the file
            keyFileOut.close();
        }
    }

    return 0;
}
