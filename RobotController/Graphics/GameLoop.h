/**
 * This handles all the game logic, and initialization
 */
#pragma once
#include "Model/OpenGLModel.h"
#include "Camera.h"
#include "GameLoop.h"
#include "Window/Window.h"

using namespace std;
using namespace glm;

class GameLoop
{
public:
	GameLoop(int width, int height, Engine::Window* window);
	void update();
	void cleanUp();
	void init();

	//dimensions of the window
	const int WIDTH;
	const int HEIGHT;
	
private:
	//MODELS
	OpenGLModel* catBox;
	OpenGLModel* dogBox;
	Camera* myCamera;
	Engine::Window* myWindow;
	Clock* myClock;
};

