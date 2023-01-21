#pragma once
#include <GL/glew.h>
#include <string>
#include "../Camera.h"
using namespace std;

//this is an openGL wrapper so the code is easier to understand
class OpenGLModel : public Object
{
private:
	//how many textures we have generated so far
	int texturesGenerated = 0;

	int startIndexTex = 0;

	static int totalTexturesGenerated;
public:
	//this is the current vao that we are bound to so we don't switch when we don't have to
	static GLuint currentVAO;
	//this will save the state of pretty much everything we do with opengl
	//in this model so we can bind to different ones when we want to change
	//different models
	GLuint vao;

	//this is our vertex buffer object
	GLuint vbo;

	//references to the vertex and fragment shaders
	GLuint vertexShader;
	GLuint fragmentShader;
	//link to the shader program
	GLuint shaderProgram;
	//link to the pos attribute
	GLuint posAttrib;
	//link to the color attribute
	GLuint colorAttrib;

	//the link to the model transform matrix
	GLint uniModel;
	GLint uniView;
	GLint uniProj;
	
	//holds all the textures
	GLuint* textures;
	//the total number of textures we will generate
	int totalNumTextures;

	//link to the camera
	Camera* myCamera;

	//creates a new graphics functions object
	OpenGLModel(int numTextures, int start);

	//this will make sure we are bound to our vao. call this anytime
	//you change anything about this model (opengl stuff)
	void makeSureBoundToVao() const;

	GLuint* createShadersFromSource(const char* shaderFile, bool print);
	void printShaderCompileStatus(GLuint shader);
	GLuint* createProgramFromShaders();

	void addVertices(GLfloat* vertices, int sizeOfArray);

	//specifies how the vertex shader input position is grabbed from the vertices
	void bindToPositionAttribute();
	//binds to the color attribute
	void bindToColorAttribute();
	//same deal
	void bindToTextureCoordinateAttribute();
	//use this to add a texture, just give the file name (assuming it's in the root dir)
	//and what the shader attribute is called
	void addTexture(const char* fileName, const char* shaderAttribName);

	//initializes uniforms for the model, view and projection matrix (using camera location)
	void bindToCamera(Camera* camera);
	//updates the vertex shader matrices
	void update();
	
private:
	void setModelTransform(mat4 transform);
	void positionChanged() override;
	void orientationChanged() override;
public: 
	//updates the transform
	void updateTransform();

	//this should be avoided with the vao
	void makeSureActive();

	void readyTextures();
	
	void cleanUp();
};

