/**
 * Main.cpp
 * This is the entry point for the application
 */
#include <iostream>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <SFML/Window.hpp>
#include <chrono>
#include "GameLoop.h"
#include "Window/Window.h"

using namespace std;
using namespace glm;

//GLint doesn't have different sizes on different compilers whereas int does
const GLint WIDTH = 1920, HEIGHT = 1080;

int main() {
	const Engine::WindowSettings myWindowSettings = { WIDTH, HEIGHT };
	Engine::Window myWindow = Engine::Window(myWindowSettings);
	myWindow.Show();

	//this will handle all the logic of our game
	GameLoop gameLoop(WIDTH, HEIGHT, &myWindow);
	//bind gameLoop's update function to the window
	myWindow.addLoopFunction([&gameLoop]() { gameLoop.update(); });
	myWindow.addInitFunction([&gameLoop]() { gameLoop.init(); });
	//this will start calling the gameLoop update and also polling events
	myWindow.startLoop();

	return EXIT_SUCCESS;
}
