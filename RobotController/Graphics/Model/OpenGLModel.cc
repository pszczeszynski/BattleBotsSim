#include "OpenGLModel.h"
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <SOIL2.h>
#include <glm/gtc/matrix_transform.hpp>
#include "../Camera.h"
#include <fstream>
#include <sstream>

using namespace std;
using namespace glm;

// make sure this is declared
GLuint OpenGLModel::currentVAO = -100;
int OpenGLModel::totalTexturesGenerated = 0;

// position, color, tex coord = 8 floats
const int NUM_FLOATS_PER_VERTEX = 8;
const int VERTEX_SHADER_STRIDE = NUM_FLOATS_PER_VERTEX * sizeof(GLfloat);

OpenGLModel::OpenGLModel(int numTextures, int start) : totalNumTextures(numTextures)
{
	startIndexTex = start;

	// Create the Vertex Array Object
	glGenVertexArrays(1, &vao);

	makeSureBoundToVao();
	// Initialize the array holds links to all the textures
	textures = new GLuint[numTextures];
}

/**
 * \brief
 * makes sure the openGL state machine is using our vertex array object
 */
void OpenGLModel::makeSureBoundToVao() const
{
	if (currentVAO != vao)
	{
		glBindVertexArray(vao);
		currentVAO = vao;
	}
}

// returns a GLuint* which points to an array of two GLuints that are references to the vertex and fragment
GLuint *OpenGLModel::createShadersFromSource(const char *shaderFile, bool print)
{
	// PARSING FILES
	std::ifstream stream(shaderFile);

	enum class ShaderType
	{
		NONE = -1,
		VERTEX = 0,
		FRAGMENT = 1
	};

	std::string line;
	std::stringstream ss[2];
	ShaderType type = ShaderType::NONE;
	while (getline(stream, line))
	{
		if (line.find("#shader") != std::string::npos)
		{
			if (line.find("vertex") != std::string::npos)
				type = ShaderType::VERTEX;
			else if (line.find("fragment") != std::string::npos)
				type = ShaderType::FRAGMENT;
		}
		else
		{
			ss[static_cast<int>(type)] << line << '\n';
		}
	}

	string vertexSourceString = ss[static_cast<int>(ShaderType::VERTEX)].str();
	string fragmentSourceString = ss[static_cast<int>(ShaderType::FRAGMENT)].str();
	const char *vertexSource = vertexSourceString.c_str();
	const char *fragmentSource = fragmentSourceString.c_str();
	std::cout << "VERTEX: " << std::endl
			  << vertexSource << std::endl;
	std::cout << "FRAGMENT: " << std::endl
			  << fragmentSource << std::endl;

	// i don't think we need this but just in case;
	makeSureBoundToVao();
	// create and get a reference to a new vertex shader
	this->vertexShader = glCreateShader(GL_VERTEX_SHADER);

	// we can just pass a reference to the shader to write to it unlike a vbo.
	// Our reference was given by the GLuint vertexShader
	// the last parameter could contain an array of source code string lengths.
	// But NULL specifies look until the NULL terminator is reached
	glShaderSource(vertexShader, 1, &vertexSource, NULL);

	// compile the shader into code that can be executed by the graphics card
	glCompileShader(vertexShader);

	if (print)
	{
		// print if the shader compilation was successful
		printShaderCompileStatus(vertexShader);
	}

	// create and get a reference to a new fragment shader
	this->fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
	glCompileShader(fragmentShader);

	if (print)
	{
		// print if the shader compilation was successful
		printShaderCompileStatus(fragmentShader);
	}

	// put them into a new array
	GLuint *returnMe = new GLuint[2];

	returnMe[0] = vertexShader;
	returnMe[1] = fragmentShader;
	return returnMe;
}

// prints the result of compiling a shader
void OpenGLModel::printShaderCompileStatus(GLuint shader)
{
	// get if the vertex shader compiled successfully
	GLint shaderStatus;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &shaderStatus);
	// print if it compiled successfully
	cout << "Vertex Shader Status: " + (string)(shaderStatus == GL_TRUE ? "success\n" : "failure\n");
	// get the last 512 characters of the compile buffer
	char vertexShaderCompileBuffer[512];
	glGetShaderInfoLog(shader, 512, NULL, vertexShaderCompileBuffer);
	// print in case there were errors
	cout << vertexShaderCompileBuffer + (string) "\n";

	// only stop if it was unsuccessful
	if (shaderStatus != GL_TRUE)
	{
		// stop to let the user press something
		system("pause");
	}
}

// creates a program out of the two shaders vertex and fragment shader
GLuint *OpenGLModel::createProgramFromShaders()
{
	// always do this first
	makeSureBoundToVao();

	// now that we have compiled the shaders, we need to make a program out of them
	shaderProgram = glCreateProgram();
	// attach the vertex shader to the shaderProgram
	glAttachShader(shaderProgram, vertexShader);
	// attach the fragment shader
	glAttachShader(shaderProgram, fragmentShader);

	// I don't think we need this line.
	glBindFragDataLocation(shaderProgram, 0, "outColor");

	// now we can link the program
	glLinkProgram(shaderProgram);
	// only one program can be used at a time, just like a vertex buffer
	glUseProgram(shaderProgram);

	// if you want here's a link to the program
	return &shaderProgram;
}

// pass in the vertices along with their size because we can't get the size from here
void OpenGLModel::addVertices(GLfloat *vertices, int sizeOfArray)
{
	makeSureBoundToVao();

	// generate the vbo
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);

	glBufferData(GL_ARRAY_BUFFER, sizeOfArray, vertices, GL_STATIC_DRAW);
}

// binds to the position attribute in the shader
void OpenGLModel::bindToPositionAttribute()
{
	makeSureBoundToVao();

	posAttrib = glGetAttribLocation(shaderProgram, "position");
	glEnableVertexAttribArray(posAttrib);

	// stride 0 and offset 0. The information will be stored in the VAO. this makes switching between different vertex data
	// and vertex formats as easy as binding a different VAO! The VAO doesn't store any vertex data by itself, it just
	// references the VBOs we've created and how to retrieve the attribute values from them
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, VERTEX_SHADER_STRIDE, 0);
}

// binds to the color attribute
void OpenGLModel::bindToColorAttribute()
{
	makeSureBoundToVao();

	colorAttrib = glGetAttribLocation(shaderProgram, "vertexColor");
	glEnableVertexAttribArray(colorAttrib);
	glVertexAttribPointer(colorAttrib, 3, GL_FLOAT, GL_FALSE, VERTEX_SHADER_STRIDE, (void *)(3 * sizeof(GLfloat)));
}

// binds to the texture coordinate attribute
// TODO: all these should probably have configurable offsets and all
void OpenGLModel::bindToTextureCoordinateAttribute()
{
	makeSureBoundToVao();

	GLint texAttrib = glGetAttribLocation(shaderProgram, "texcoord");
	glEnableVertexAttribArray(texAttrib);
	glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, VERTEX_SHADER_STRIDE, (void *)(6 * sizeof(GLfloat)));
}

// adds a texture to this object
void OpenGLModel::addTexture(const char *fileName, const char *shaderAttribName)
{
	// return if we have already generated all the textures
	if (texturesGenerated >= totalNumTextures)
	{
		return;
	}
	// make sure we are on the correct VAO
	makeSureBoundToVao();

	// call the glGenTextures
	glGenTextures(1, &textures[texturesGenerated]);

	int width, height;
	// this will set up the image data
	unsigned char *image;

	glActiveTexture(GL_TEXTURE0 + totalTexturesGenerated);

	// bind the texture to the active one before performing operations on it. Right now the other one is still bound I think.
	glBindTexture(GL_TEXTURE_2D, textures[texturesGenerated]);
	// grab the image data from soil
	image = SOIL_load_image(fileName, &width, &height, 0, SOIL_LOAD_RGB);
	// now we load it to the texture, since image contains the pixels
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	// clean up the image data since we have already loaded to the texture
	SOIL_free_image_data(image);
	glUniform1i(glGetUniformLocation(shaderProgram, shaderAttribName), texturesGenerated);

	// set the ends of the texture to be repeated
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	// set the scaling mode to be linear for decreasing and increasing scaling. This results in blurring, not pixelated look
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// we have generated a texture so increment that
	texturesGenerated++;
	totalTexturesGenerated++;
}

/**
 * \brief
 * links the model to the camera
 */
void OpenGLModel::bindToCamera(Camera *camera)
{
	// don't think we need this but just in case
	makeSureBoundToVao();

	myCamera = camera;
	// the transformation will be communicated to the vertex shader via a uniform
	uniModel = glGetUniformLocation(shaderProgram, "model");
	// now create a uniform that communicates that
	uniView = glGetUniformLocation(shaderProgram, "view");

	uniProj = glGetUniformLocation(shaderProgram, "proj");
	update(); // this will initialize the uniforms
}

/**
 * \brief
 * Call this each update
 * This will update the matrices from the Camera
 */
void OpenGLModel::update()
{
	makeSureBoundToVao();
	glUniformMatrix4fv(uniProj, 1, GL_FALSE, value_ptr(*myCamera->getProjectionMatrix()));
	glUniformMatrix4fv(uniView, 1, GL_FALSE, value_ptr(*myCamera->getViewMatrix()));
}

void OpenGLModel::setModelTransform(mat4 transform)
{
	makeSureBoundToVao();

	glUniformMatrix4fv(uniModel, 1, GL_FALSE, value_ptr(transform));
}

/**
 *\brief
 * We need to update the transform when the position or orientation is changed
 */
void OpenGLModel::positionChanged()
{
	updateTransform();
}
/**
 *\brief
 * We need to update the transform when the position or orientation is changed
 */
void OpenGLModel::orientationChanged()
{
	updateTransform();
}

// call this everytime you set rotation or position
void OpenGLModel::updateTransform()
{
	mat4 positionMatrix = mat4(1.0f);
	positionMatrix = translate(positionMatrix, position);
	mat4 orientationMatrix = lookAt(vec3(0, 0, 0), orientation, vec3(0, 0, 1));
	mat4 overall = positionMatrix * orientationMatrix;

	setModelTransform(overall);
}

void OpenGLModel::makeSureActive()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	bindToPositionAttribute();
	bindToColorAttribute();
	bindToTextureCoordinateAttribute();
}

void OpenGLModel::readyTextures()
{

	for (int i = 0; i < texturesGenerated; i++)
	{
		switch (i)
		{
		case 0:
			// set the first texture as active
			glActiveTexture(GL_TEXTURE0);
			break;
		case 1:
			glActiveTexture(GL_TEXTURE1);
			break;
		case 2:
			glActiveTexture(GL_TEXTURE2);
			break;
		case 3:
			glActiveTexture(GL_TEXTURE3);
			break;
		case 4:
			glActiveTexture(GL_TEXTURE4);
			break;
		default:
			glActiveTexture(GL_TEXTURE0);
			break;
		}

		glBindTexture(GL_TEXTURE_2D, textures[i]);
	}
}

void OpenGLModel::cleanUp()
{
	glDeleteTextures(totalNumTextures, textures);
	glDeleteProgram(shaderProgram);
	glDeleteShader(fragmentShader);
	glDeleteShader(vertexShader);
	glDeleteBuffers(1, &vbo);
	glDeleteVertexArrays(1, &vao);
}
