#version 330

layout (location = 0) in vec3 position0;

uniform mat4 worldMatrix;

void main()
{
	gl_Position = worldMatrix * vec4(position0,1);
}