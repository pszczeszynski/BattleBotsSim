#include "GameLoop.h"
#include <vector>
#include "cubeVertices.h"

#include <SFML/System/Err.hpp>
#include <iostream>
#include <SFML/Window/Keyboard.hpp>
#include <opencv2/core/core.hpp>

bool GameLoop::run = false;
bool GameLoop::backup = false;

/**
 * \brief initializes the gameloop object
 */
GameLoop::GameLoop(int width, int height, Engine::Window *window) : WIDTH(width),
																	HEIGHT(height),
																	myWindow(window)
{
	// we can only enable this after we initialize
	// won't draw vertices that are behind
	glEnable(GL_DEPTH_TEST);

	// disables that input error message on windows 7
	sf::err().rdbuf(NULL);

	// create a new camera
	myCamera = new Camera(vec3(0, -15.0f, 5.0f),
						  vec3(0, 15.0f, -5.0f), WIDTH, HEIGHT);

	initPointCloud();
	initOpponentRobot();
	initPathPlanning();
}

void GameLoop::initOpponentRobot()
{
	// these will be the vertices of the cube
	vector<GLfloat> &cubeVertices = CUBE_VERTICES;

	opponentRobotGameObject = new OpenGLModel(1, 0);

	const float scale = 2.0f;
	// set random colors
	for (int i = 0; i < cubeVertices.size() / 8; i++)
	{
		// scale down
		cubeVertices[i * 8 + 0] *= scale;
		cubeVertices[i * 8 + 1] *= scale;
		cubeVertices[i * 8 + 2] *= scale;

		cubeVertices[i * 8 + 3] = (i % 10) / 10.0;
		cubeVertices[i * 8 + 4] = ((i + 3) % 10) / 10.0;
		cubeVertices[i * 8 + 5] = ((i + 6) % 10) / 10.0;
	}
	opponentRobotGameObject->setVertices(&cubeVertices[0], cubeVertices.size() * sizeof(cubeVertices[0]));
	opponentRobotGameObject->createShadersFromSource("Graphics/Shader/Basic.shader", true);
	opponentRobotGameObject->createProgramFromShaders();
	opponentRobotGameObject->addTexture("Graphics/Pictures/sample.png", "texKitten");
	opponentRobotGameObject->rebindAllAttribtes();
	opponentRobotGameObject->bindToCamera(myCamera);
}

void GameLoop::initPointCloud()
{
	pointCloudGameObject = new OpenGLModel(1, 0);
	pointCloudGameObject->createShadersFromSource("Graphics/Shader/Basic.shader", true);
	pointCloudGameObject->createProgramFromShaders();
	pointCloudGameObject->rebindAllAttribtes();
	// pointCloudGameObject->addTexture("Graphics/Pictures/sample.png", "texKitten");
	pointCloudGameObject->bindToCamera(myCamera);
}

void GameLoop::initPathPlanning()
{
	pathPlanningGameObject = new OpenGLModel(1, 0);
	pathPlanningGameObject->createShadersFromSource("Graphics/Shader/Basic.shader", true);
	pathPlanningGameObject->createProgramFromShaders();
	pathPlanningGameObject->rebindAllAttribtes();
	// pathPlanningGameObject->addTexture("Graphics/Pictures/sample.png", "texKitten");
	pathPlanningGameObject->bindToCamera(myCamera);
}

void GameLoop::SetPointCloudVerts(std::vector<cv::Point3f> &vertices, std::vector<cv::Vec3b> &colors)
{
	pointCloudLock.lock();
	pointCloudVerts.clear(); // delete all vectory contents

	for (int i = 0; i < vertices.size(); i++)
	{
		Point thisPoint = {vertices[i].x, vertices[i].y, vertices[i].z};

		// pos
		pointCloudVerts.emplace_back(thisPoint.x);
		pointCloudVerts.emplace_back(-thisPoint.y);
		pointCloudVerts.emplace_back(thisPoint.z);

		// color
		pointCloudVerts.emplace_back((double)colors[i][2] / 255.0);
		pointCloudVerts.emplace_back((double)colors[i][1] / 255.0);
		pointCloudVerts.emplace_back((double)colors[i][0] / 255.0);

		// tex coord
		pointCloudVerts.emplace_back((double)1.0);
		pointCloudVerts.emplace_back((double)0.0);
	}
	pointCloudVerts.shrink_to_fit(); // free unused memory
	pointCloudLock.unlock();
}

void GameLoop::SetPathPlanningLines(std::vector<Line> &lines)
{
	pointCloudLock.lock();
	pathPlanningVerts = {};
	// for each point in the lines array
	for (int i = 0; i < lines.size(); i++)
	{
		// START //
		// pos
		pathPlanningVerts.emplace_back(lines[i].start.x);
		pathPlanningVerts.emplace_back(-lines[i].start.y); // TODO: don't hard-code polarity, make more clear
		pathPlanningVerts.emplace_back(lines[i].start.z);
		// color
		pathPlanningVerts.emplace_back((double)lines[i].color[0] / 255.0);
		pathPlanningVerts.emplace_back((double)lines[i].color[1] / 255.0);
		pathPlanningVerts.emplace_back((double)lines[i].color[2] / 255.0);

		// tex coord
		pathPlanningVerts.emplace_back((double)1.0);
		pathPlanningVerts.emplace_back((double)0.0);

		// END //
		// pos
		pathPlanningVerts.emplace_back(lines[i].end.x);
		pathPlanningVerts.emplace_back(-lines[i].end.y);
		pathPlanningVerts.emplace_back(lines[i].end.z);
		// color
		pathPlanningVerts.emplace_back((double)lines[i].color[0] / 255.0);
		pathPlanningVerts.emplace_back((double)lines[i].color[1] / 255.0);
		pathPlanningVerts.emplace_back((double)lines[i].color[2] / 255.0);
		// tex coord
		pathPlanningVerts.emplace_back((double)1.0);
		pathPlanningVerts.emplace_back((double)0.0);
	}
	pointCloudLock.unlock();
}

void GameLoop::SetOpponentPosition(vec3 position)
{
	pointCloudLock.lock();
	opponentPosition = position;
	pointCloudLock.unlock();
}

void GameLoop::UpdateCameraPosition()
{
	const float MOVEMENT_SPEED = 0.2f;
	vec3 deltaMovement = vec3(0, 0, 0);

	// move left and right
	if (myWindow->keyIsDown(sf::Keyboard::Key::A))
	{
		deltaMovement.x = -1;
	}
	if (myWindow->keyIsDown(sf::Keyboard::Key::D))
	{
		deltaMovement.x = 1;
	}

	// move in and out
	if (myWindow->keyIsDown(sf::Keyboard::Key::W))
	{
		deltaMovement.z = -1;
	}
	if (myWindow->keyIsDown(sf::Keyboard::Key::S))
	{
		deltaMovement.z = 1;
	}

	// move up and down
	if (myWindow->keyIsDown(sf::Keyboard::Key::Q))
	{
		deltaMovement.y = 1;
	}
	if (myWindow->keyIsDown(sf::Keyboard::Key::E))
	{
		deltaMovement.y = -1;
	}

	if (myWindow->keyIsDown(sf::Keyboard::Key::L))
	{
		cameraLock = true;
	}

	if (myWindow->keyIsDown(sf::Keyboard::Key::U))
	{
		cameraLock = false;
	}

	if (myWindow->keyIsDown(sf::Keyboard::Key::Space))
	{
		GameLoop::run = true;
	}

	if (myWindow->keyIsDown(sf::Keyboard::Key::I))
	{
		GameLoop::run = false;
	}

	GameLoop::backup = myWindow->keyIsDown(sf::Keyboard::Key::B);


	if (!cameraLock)
	{
		myCamera->setOrientation(vec3(myWindow->getMouseX(), 0.2, -myWindow->getMouseY()));
		myCamera->translateRelative(deltaMovement * MOVEMENT_SPEED);
	}
}

void GameLoop::drawPathPlanning()
{
	int vertexCount = 0;
	pointCloudLock.lock();
	pathPlanningGameObject->setVertices(&pathPlanningVerts[0], pathPlanningVerts.size() * sizeof(pathPlanningVerts[0]));
	vertexCount = pathPlanningVerts.size();
	pointCloudLock.unlock();

	// update projection and view matrices since the camera position might have changed
	pathPlanningGameObject->update();
	// set the orientation to be upright
	pathPlanningGameObject->setOrientation(vec3(0, 1, 0));
	pathPlanningGameObject->makeSureBoundToVao();
	pathPlanningGameObject->readyTextures();

	glLineWidth(10.0f); // set the line width to be 10 pixels

	glDrawArrays(GL_LINES, 0, vertexCount / 8);
}

void GameLoop::drawPointCloud()
{
	// update the pointcloud
	int pointCloudSize = 0;
	pointCloudLock.lock();
	pointCloudGameObject->setVertices(&pointCloudVerts[0], pointCloudVerts.size() * sizeof(pointCloudVerts[0]));
	pointCloudSize = pointCloudVerts.size();
	pointCloudLock.unlock();

	// update projection and view matrices since the camera position might have changed
	pointCloudGameObject->update();
	// set the orientation to be upright
	pointCloudGameObject->setOrientation(vec3(0, 1, 0));
	pointCloudGameObject->makeSureBoundToVao();
	pointCloudGameObject->readyTextures();

	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

	GLfloat attenuationParams[] = {1.0, 0.0, 0.01};
	glPointParameterf(GL_POINT_SIZE_MIN, 1.0); // Minimum point size
	glPointParameterf(GL_POINT_SIZE_MAX, 100.0); // Maximum point size
	glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION, attenuationParams);


	glPointSize(2.5f); // set the size of points to 10 pixels

	glDrawArrays(GL_POINTS, 0, pointCloudSize / 8);
}

void GameLoop::drawOpponentPosition()
{
	opponentPositionMutex.lock();
	vec3 currPos = opponentPosition;
	opponentPositionMutex.unlock();

	// swap x and z
	float savedY = currPos.y;
	currPos.y = currPos.z;
	currPos.z = savedY;

	opponentRobotGameObject->setPosition(currPos);
	double time = myClock->getElapsedTime();
	opponentRobotGameObject->update();
	// rotate it with time so it looks cool
	opponentRobotGameObject->setOrientation(vec3(sin(time * 6), cos(time * 6), cos(time * 6)));
	// we might have just drawn another obejct, so we must rebind to the vao to tell opengl which vertices to draw
	opponentRobotGameObject->makeSureBoundToVao();
	opponentRobotGameObject->readyTextures();
	glLineWidth(7.0f); // set the line width to be 7 pixels
	// TODO: make it not just use the CUBE_VERTICES size and have the object have a draw method
	glDrawArrays(GL_LINE_LOOP, 0, CUBE_VERTICES.size() / 8);
}

/**
 * Call this every update
 */
void GameLoop::update()
{
	// set the clear color: r, g, b, a
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Allow player movement
	UpdateCameraPosition();

	drawPointCloud();
	drawOpponentPosition();
	drawPathPlanning();
}

/**
 * This will free the memory we have used
 */
void GameLoop::cleanUp()
{
	pointCloudGameObject->cleanUp();
	opponentRobotGameObject->cleanUp();
	pathPlanningGameObject->cleanUp();
}

/**
 * \brief
 * called when the game starts
 */
void GameLoop::init()
{
	myClock = new Clock();
	myClock->markStart();
}
