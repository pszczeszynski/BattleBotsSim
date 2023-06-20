#pragma once
/**
 * The Window class will manage the window of the game and also call the loop function when the game is started
 */
#include <functional>
#include "Clock.h"
#include <SFML/Window/Window.hpp>
#include <map>
#include <SFML/Window/Keyboard.hpp>

namespace Engine
{
	struct WindowSettings
	{
		int width, height;
		const char* title;
	};
	
	class Window
	{
	public:
		/**
		 * \brief
		 * When you create a window, you need to give it it's settings, and a loop function
		 */
		Window(const WindowSettings& windowSettings);

		int Width;
		int Height;
		const char* Title;

		inline int getWidth() { return Width; }
		inline int getHeight() { return Height; }

		void Show();
		void addLoopFunction(const std::function<void()> func);
		void addInitFunction(const std::function<void()> initFunction);
		void startLoop();

		bool keyIsDown(sf::Keyboard::Key);
		float getMouseX();
		float getMouseY();

	private:
		// maps keys to bool, which is true of the key is currently pressed down
		std::map<sf::Keyboard::Key, bool> keyMap;
		// mouse position
		float mouseX;
		float mouseY;

		std::function<void()> loopFunctionCallback;
		std::function<void()> initFunctionCallback;
		sf::Window* window;
	};
}
