#include "Camera.h"

/**
 * \brief
 * gets the matrix that transforms 3d coordinates into relative
 * coordinates to the cameraand its orientation
 */
mat4* Camera::getViewMatrix()
{
	viewMatrix = lookAt(position, position + orientation, vec3(0, 0, 1));
	return &viewMatrix;
}

/**
 * \brief
 * Gets the matrix that turns a world coordinate into a screen coordinate
 * \return a pointer to the projection matrix
 */
mat4* Camera::getProjectionMatrix()
{
	projectionMatrix = perspective(radians(45.0f), 
		static_cast<float>(screenWidth) / screenHeight, NEAR_CLIPPING_PLANE, FAR_CLIPPING_PLANE);
	return &projectionMatrix;
}

void Camera::positionChanged()
{
	getViewMatrix();
}

void Camera::orientationChanged()
{
	getViewMatrix();
}