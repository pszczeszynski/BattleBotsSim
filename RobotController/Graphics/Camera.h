#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Model/Object.h"

using namespace glm;

class Camera : public Object
{
public:
	float FAR_CLIPPING_PLANE = 100.0f;
	float NEAR_CLIPPING_PLANE = 0.1f;
	int screenWidth;
	int screenHeight;
	
	mat4 viewMatrix;
	mat4 projectionMatrix;

	// creates a new camera object
	Camera(vec3 startPos, vec3 startLook, int mScreenWidth, int mScreenHeight)
		: Object(startPos, startLook), // Initialize our position and orientation
		  screenWidth(mScreenWidth),
		  screenHeight(mScreenHeight)
	{
		// update the view matrix
		getViewMatrix();
	}

	// gets the transform matrix
	mat4* getViewMatrix();
	//gets the projection matrix to the screen
	mat4* getProjectionMatrix();

	void positionChanged() override;
	void orientationChanged() override;
};

