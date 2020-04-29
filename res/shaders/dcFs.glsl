#version 330

smooth in vec3 vertexColour;
smooth in vec3 vertexNormal;

void main()
{
    vec3 lightDir = -normalize(vec3(1, 5, -5));
    float d = dot(vertexNormal, -lightDir);
    d = max(0.2, d);
    gl_FragColor = vec4(vertexColour, 1) * d;
}