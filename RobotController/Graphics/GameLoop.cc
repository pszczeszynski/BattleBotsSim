#include "GameLoop.h"
#include <vector>
#include "cubeVertices.h"

#include <SFML/System/Err.hpp>
#include <iostream>
#include <SFML/Window/Keyboard.hpp>

/**
 * \brief initializes the gameloop object
 */
GameLoop::GameLoop(int width, int height, Engine::Window* window) :
	WIDTH(width),
	HEIGHT(height),
	myWindow(window)
{
	//we can only enable this after we initialize
	//won't draw vertices that are behind
	glEnable(GL_DEPTH_TEST);

	//disables that input error message on windows 7
	sf::err().rdbuf(NULL);
	
	//these will be the vertices of the cube
	vector<GLfloat>& catVertices = CUBE_VERTICES;
	
	//these will be the vertices of the better cube
	vector<GLfloat>& dogVertices = CUBE_VERTICES;
	//create a new camera
	myCamera = new Camera(vec3(0, -15.0f, 5.0f),
		vec3(0, 15.0f, -5.0f), WIDTH, HEIGHT);
	
	//this is our openGL wrapper
	pointCloudGameObject = new OpenGLModel(1, 0);
	pointCloudGameObject->addVertices(&catVertices[0],
		catVertices.size() * sizeof(catVertices[0]));
	pointCloudGameObject->createShadersFromSource("Graphics/Shader/Basic.shader", true);
	pointCloudGameObject->createProgramFromShaders();
	pointCloudGameObject->bindToPositionAttribute();
	pointCloudGameObject->bindToColorAttribute();
	pointCloudGameObject->bindToTextureCoordinateAttribute();
	pointCloudGameObject->addTexture("Graphics/Pictures/sample.png", "texKitten");
	pointCloudGameObject->bindToCamera(myCamera);
}

void GameLoop::SetPointCloudVerts(std::vector<Point> &vertices)
{
	pointCloudLock.lock();
	pointCloud = {};
	for (int i = 0; i < vertices.size(); i++)
	{
		Point thisPoint = vertices[i];
		// scale it down
		thisPoint.x /= 500.0f;
		thisPoint.y /= 500.0f;
		thisPoint.z /= 500.0f;

		// pos
		// std::cout << "pushing back: " << thisPoint.x << std::endl;
		pointCloud.push_back(thisPoint.x);
		pointCloud.push_back(-thisPoint.y);
		pointCloud.push_back(thisPoint.z);
		
		
		// color
		pointCloud.push_back(1);//(i % 10) / 10.0f);
		pointCloud.push_back(0);//((i + 3) % 10) / 10.0f);
		pointCloud.push_back(1);//((i + 6) % 10) / 10.0f);

		// tex coord
		pointCloud.push_back(0.0f);
		pointCloud.push_back(0.0f);
	}
	pointCloudLock.unlock();
}

void GameLoop::updateCameraPosition()
{
	const float MOVEMENT_SPEED = 0.3f;
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

	// move up and down`
	if (myWindow->keyIsDown(sf::Keyboard::Key::Q))
	{
		deltaMovement.y = 1;
	}
	if (myWindow->keyIsDown(sf::Keyboard::Key::E))
	{
		deltaMovement.y = -1;
	}

	myCamera->setOrientation(vec3(myWindow->getMouseX(), 0.4, -myWindow->getMouseY()));
	myCamera->translateRelative(deltaMovement * MOVEMENT_SPEED);
}

/**
 * Call this every update
 */
void GameLoop::update()
{

	// Allow player movement
	updateCameraPosition();


	// update the pointcloud
	int pointCloudSize = 0;
	pointCloudLock.lock();
	pointCloudGameObject->addVertices(&pointCloud[0], pointCloud.size() * sizeof(pointCloud[0]));
	pointCloudGameObject->bindToPositionAttribute();
	pointCloudSize = pointCloud.size();
	pointCloudLock.unlock();

	double time = myClock->getElapsedTime();

	//set the clear color: r, g, b, a
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



	pointCloudGameObject->update();

	time = 0;
	// set pointcloud orientation
	double x = cos((float) (time * radians(180.0f))) * cos(0);
	double y = sin((float) (time * radians(180.0f))) * cos(0);
	double z = sin(0);
	pointCloudGameObject->setOrientation(vec3(0, 1, 0));
	
	pointCloudGameObject->makeSureBoundToVao();
	pointCloudGameObject->readyTextures();
	
	
	glPointSize(5.0f);  // set the size of points to 10 pixels

	//draw the point cloud
	// glDrawArrays(GL_TRIANGLES, 0, pointCloudSize / 8);
	glDrawArrays(GL_POINTS, 0, pointCloudSize / 8);
}

/**
 * This will free the memory we have used
 */
void GameLoop::cleanUp()
{
	pointCloudGameObject->cleanUp();
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
