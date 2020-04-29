#version 330

layout (location = 0) in vec3 position0;
layout (location = 2) in vec3 color2;

uniform mat4 modelViewProjectionMatrix;

layout(location=1) in vec3 normal;
smooth out vec3 vertexColour;
smooth out vec3 vertexNormal;

void main()
{
    vertexColour = color2;
    vertexNormal = normal;
    gl_Position = modelViewProjectionMatrix * vec4(position0,1);
}