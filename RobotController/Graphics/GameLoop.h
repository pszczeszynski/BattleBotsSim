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

	GameLoop(int width, int height, Engine::Window *window);
	void update();
	void cleanUp();
	void init();


	struct Line
	{
		cv::Point3f start;
		cv::Point3f end;
		cv::Vec3b color;
	};

	void SetPointCloudVerts(std::vector<cv::Point3f> &vertices, std::vector<cv::Vec3b> &colors);
	void SetOpponentPosition(vec3 position);
	void SetPathPlanningLines(std::vector<Line> &lines);

	// dimensions of the window
	const int WIDTH;
	const int HEIGHT;

private:
	void UpdateCameraPosition();
	void drawPointCloud();
	void drawOpponentPosition();
	void drawPathPlanning();

	void initPointCloud();
	void initOpponentRobot();
	void initPathPlanning();

	//////////// MODELS ////////////////
	// opponent
	OpenGLModel *opponentRobotGameObject;
	glm::vec3 opponentPosition;
	std::mutex opponentPositionMutex;

	// path planning
	OpenGLModel *pathPlanningGameObject;
	std::vector<GLfloat> pathPlanningVerts;

	// point cloud
	std::mutex pointCloudLock;
	std::vector<GLfloat> pointCloudVerts;
	OpenGLModel *pointCloudGameObject;
	////////////////////////////////////

	Camera *myCamera;
	Engine::Window *myWindow;
	Clock *myClock;
};
