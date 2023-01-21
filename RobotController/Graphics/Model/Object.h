#pragma once
//Object is a base class of anything in the 3d world
//it has a position and rotation
#include <glm/glm.hpp>

using namespace glm;

class Object
{
public:
	//our position
	vec3 position;
	//where we are looking at relative to ourself
	vec3 orientation;

	//objects must have a startPos and a start look location
	explicit Object(vec3 startPos = vec3(0,0,0), vec3 startLook = vec3(0, 1, 0))
		: position(startPos), orientation(startLook) {

	}
	//sets the position and updates the view matrix
	void setPosition(vec3 newPos);
	//same deal
	void setOrientation(vec3 orientation);
	/**
	 * \brief
	 * moves the object in world space
	 */
	void translateAbsolute(vec3 amount);
	
	void translateRelative(vec3 amount);

private:
	//called when position is changed
	virtual void positionChanged() {}
	virtual void orientationChanged() {}
};

