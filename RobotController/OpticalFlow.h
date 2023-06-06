
#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include "CornerTracker.h"
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ValueBin.h"
#include "Corner.h"

class OpticalFlow
{
	// after how many frames should we find features again
	int FRAMES_PER_TRACK = 1;
	int imageWidth = 0;
	int imageHeight = 0;

	int numFrames = 0;

	// the corners are reset every frame for ct
	CornerTracker ct;

	cv::cuda::GpuMat droneMask;

	// amount of movement this frame
	cv::Point2f translateDelta = cv::Point2f(0, 0);
	cv::Point2f trackedCameraPosition = cv::Point2f(0, 0);

	// the origin of the grid (from the center of the screen)
	cv::Point2f gridPosition = cv::Point2f(0, 0);


	double angleDelta = 0;
	double trackedCameraAngle = 0;
	double scaleDelta = 1.0;
	double trackedScale = 1.0;

	double CalculateAngleDelta();
	double CalculateScaleDelta();

	void CalculateTrackerBins();
	void DrawTrackerVelocityDistribution(cv::Mat&);
	
	cv::Point2f CalculateTranslationChangeXY(cv::Mat&);

	void DrawVisualizerGrid(cv::Mat&);

public:
	double GetXPosition();
	double GetYPosition();
	double GetScale();
	double GetRotation();
	void SetRotation(double);
	
	void PerformMotionDetection(cv::cuda::GpuMat&, cv::Mat&,
		bool trackTranslation = true,
		bool trackRotation = true,
		bool trackScale = true);
	void InitializeMotionDetection(cv::cuda::GpuMat&);
	double CalcConfidence(cv::Mat&);
	OpticalFlow();
};

#endif //OPTICALFLOW_H
