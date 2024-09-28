#include "Object.h"
#include <glm/ext/matrix_transform.inl>
#include <iostream>
#define PI 3.14159265358979323846264338

// sets the position and updates the view matrix
void Object::setPosition(vec3 newPos)
{
	position = newPos;
	positionChanged();
}

// same deal
void Object::setOrientation(vec3 orientation)
{
	this->orientation = orientation;
	orientationChanged();
}

/**
 * \brief
 * Translates in world space
 */
void Object::translateAbsolute(vec3 amount)
{
	position += amount;
}

/**
 * \brief
 * moves the object relative to its orientation
 */
void Object::translateRelative(vec3 amount)
{
	mat4 rotation = lookAt(vec3(0, 0, 0), orientation, vec3(0, 0, 1));
	vec4 translation = vec4(amount, 1);
	vec3 addToPosition = vec3(inverse(rotation) * translation);
	position += addToPosition;
}
