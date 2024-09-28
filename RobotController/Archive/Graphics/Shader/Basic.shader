#shader vertex
#version 150 core

in vec3 position;
in vec3 vertexColor;
in vec2 texcoord;
out vec3 fragmentColor;
out vec2 Texcoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform vec3 overrideColor;
	
void main()
{
	fragmentColor = vertexColor;
	Texcoord = texcoord;
	gl_Position = proj * view * model * vec4(position, 1.0);
};


#shader fragment
#version 150 core

in vec3 fragmentColor;
in vec2 Texcoord;
out vec4 outColor;
uniform sampler2D texKitten;
uniform sampler2D texPuppy;

void main()
{
	//vec4 colKitten = texture(texKitten, Texcoord);
	//vec4 colPuppy = texture(texPuppy, Texcoord);
	outColor = vec4(fragmentColor, 1);// * mix(colKitten, colPuppy, 0.5);
};