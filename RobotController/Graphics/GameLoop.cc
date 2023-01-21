#include "GameLoop.h"
#include <vector>
#include "cubeVertices.h"

#include <SFML/System/Err.hpp>

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
	catBox = new OpenGLModel(1, 0);
	catBox->addVertices(&catVertices[0],
		catVertices.size() * sizeof(catVertices[0]));
	catBox->createShadersFromSource("Shader/Basic.shader", true);
	catBox->createProgramFromShaders();
	catBox->bindToPositionAttribute();
	catBox->bindToColorAttribute();
	catBox->bindToTextureCoordinateAttribute();
	catBox->addTexture("Pictures/sample.png", "texKitten");
	catBox->bindToCamera(myCamera);
	
	dogBox = new OpenGLModel(1, 1);
	dogBox->addVertices(&dogVertices[0],
		dogVertices.size() * sizeof(dogVertices[0]));
	dogBox->createShadersFromSource("Shader/Basic.shader", true);
	dogBox->createProgramFromShaders();
	dogBox->bindToPositionAttribute();
	dogBox->bindToColorAttribute();
	dogBox->bindToTextureCoordinateAttribute();
	dogBox->addTexture("Pictures/sample2.png", "texPuppy");
	dogBox->bindToCamera(myCamera);
}

/**
 * Call this every update
 */
void GameLoop::update()
{
	double time = myClock->getElapsedTime();
	
	//set the clear color: r, g, b, a
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	catBox->update();
	dogBox->update();
	double x = cos((float) (time * radians(180.0f))) * cos(0);
	double y = sin((float) (time * radians(180.0f))) * cos(0);
	double z = sin(0);
	catBox->setOrientation(vec3(x, y, z));
	
	catBox->makeSureBoundToVao();
	catBox->readyTextures();
	//draw the cube
	glDrawArrays(GL_TRIANGLES, 0, 36);
	
	dogBox->setPosition(vec3(0, 4.0f, 0));
	dogBox->setOrientation(vec3(z, y, x));
	dogBox->makeSureBoundToVao();
	dogBox->readyTextures();
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

/**
 * This will free the memory we have used
 */
void GameLoop::cleanUp()
{
	catBox->cleanUp();
	dogBox->cleanUp();
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
