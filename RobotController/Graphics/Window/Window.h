#pragma once
/**
 * The Window class will manage the window of the game and also call the loop function when the game is started
 */
#include <functional>
#include "Clock.h"
#include <SFML/Window/Window.hpp>

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
		Window(const WindowSettings& windowSettings)
			: Width(windowSettings.width), Height(windowSettings.height), Title(windowSettings.title)
		{
		}

		int Width;
		int Height;
		const char* Title;

		inline int getWidth() { return Width; }
		inline int getHeight() { return Height; }

		void Show();
		void addLoopFunction(const std::function<void()> func);
		void addInitFunction(const std::function<void()> initFunction);
		void startLoop();

	private:
		std::function<void()> loopFunctionCallback;
		std::function<void()> initFunctionCallback;
		sf::Window* window;
	};
}
