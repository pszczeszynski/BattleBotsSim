#include "Window.h"
#include <SFML/Window/ContextSettings.hpp>
#include <GL/glew.h>
#include <SFML/Window/Window.hpp>
#include "Clock.h"
#include <SFML/Window/Event.hpp>
#include <SFML/System/Sleep.hpp>

namespace Engine
{
	/**
	 * \brief
	 * This will actually show the window 
	 */
	void Window::Show()
	{
		sf::ContextSettings settings;
		settings.depthBits = 24;
		settings.stencilBits = 8;
		//settings opengl versions (MajorVersion.MinorVersion) -> 3.3
		settings.majorVersion = 3;
		settings.minorVersion = 3;
		glewExperimental = GL_TRUE;//use OpenGL extensions

		//setup the window
		window = new sf::Window(sf::VideoMode(Width, Height, 32),
			Title, sf::Style::Titlebar | sf::Style::Close, settings);

		glewInit();
	}

	/**
	 * \brief
	 * adds a callback function to be called each loop update
	 */
	void Window::addLoopFunction(const std::function<void()> func)
	{
		loopFunctionCallback = func;
	}

	/**
	 * \brief
	 * This will be called once the loop has started
	 */
	void Window::addInitFunction(const std::function<void()> initFunction)
	{
		initFunctionCallback = initFunction;
	}

	/**
	 * \brief
	 * Call this to start calling the loopFunctionCallback
	 */
	void Window::startLoop()
	{
		initFunctionCallback();
		while (window->isOpen())
		{
			sf::Event windowEvent;
			while (window->pollEvent(windowEvent)) {
				//close the window if it is a close event
				if (windowEvent.type == sf::Event::Closed)
				{
					window->close();
					break;
				}
			}
			loopFunctionCallback();
			window->display();
			sf::sleep(sf::milliseconds(15));
		}
	}
}
