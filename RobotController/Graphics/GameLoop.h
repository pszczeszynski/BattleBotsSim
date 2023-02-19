/**
 * This handles all the game logic, and initialization
 */
#pragma once
#include "Model/OpenGLModel.h"
#include "Camera.h"
#include "GameLoop.h"
#include "Window/Window.h"
#include "../RobotStateParser.h"
#include <mutex>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace glm;

class GameLoop
{
public:
	GameLoop(int width, int height, Engine::Window* window);
	void update();
	void cleanUp();
	void init();

	void SetPointCloudVerts(std::vector<Point>& vertices, std::vector<cv::Vec3b>& colors);

	//dimensions of the window
	const int WIDTH;
	const int HEIGHT;
	
private:
	void updateCameraPosition();

	//MODELS
	OpenGLModel* pointCloudGameObject;
	Camera* myCamera;
	Engine::Window* myWindow;
	Clock* myClock;



	std::mutex pointCloudLock;
	// Point cloud
	std::vector<GLfloat> pointCloud;
};

