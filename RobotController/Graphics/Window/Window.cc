#include "Window.h"
#include <SFML/Window/ContextSettings.hpp>
#include <GL/glew.h>
#include <SFML/Window/Window.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/System/Sleep.hpp>
#include <iostream>
namespace Engine
{
	Window::Window(const WindowSettings &windowSettings)
		: Width(windowSettings.width), Height(windowSettings.height), Title(windowSettings.title)
	{
	}

	/**
	 * \brief
	 * This will actually show the window
	 */
	void Window::Show()
	{


		sf::ContextSettings settings;
		settings.depthBits = 24;
		settings.stencilBits = 8;
		// settings opengl versions (MajorVersion.MinorVersion) -> 3.3
		settings.majorVersion = 3;
		settings.minorVersion = 3;
		glewExperimental = GL_TRUE; // use OpenGL extensions


		// setup the window
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
			while (window->pollEvent(windowEvent))
			{
				// close the window if it is a close event
				if (windowEvent.type == sf::Event::Closed)
				{
					window->close();
					break;
				}

				// IF key is pressed, update the keymap for that key to true
				if (windowEvent.type == sf::Event::KeyPressed)
				{
					keyMap[windowEvent.key.code] = true;
				}

				// IF key is released, update the keymap for that key to false
				if (windowEvent.type == sf::Event::KeyReleased)
				{
					keyMap[windowEvent.key.code] = false;
				}

				// update the mouse position if it has moved
				if (windowEvent.type == sf::Event::MouseMoved)
				{
					// Get the current mouse position in window coordinates
					sf::Vector2i pixelPos = sf::Mouse::getPosition(*window);

					// Normalize the mouse coordinates to [0, 1]
					mouseX = static_cast<float>(pixelPos.x) / static_cast<float>(window->getSize().x);
					mouseY = static_cast<float>(pixelPos.y) / static_cast<float>(window->getSize().y);

					// Now normalize to be from -1 to 1
					mouseX = mouseX * 2 - 1;
					mouseY = mouseY * 2 - 1;
				}
			}
			loopFunctionCallback();
			window->display();
			sf::sleep(sf::milliseconds(15));
		}
	}

	/**
	 * @brief Returns true of the given key is currently down
	 *
	 * @param key the key to check
	 */
	bool Window::keyIsDown(sf::Keyboard::Key key)
	{
		return keyMap[key];
	}

	float Window::getMouseX()
	{
		return mouseX;
	}

	float Window::getMouseY()
	{
		return mouseY;
	}
}
