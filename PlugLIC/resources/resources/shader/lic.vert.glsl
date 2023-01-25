#version 330 core

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

in vec3 position;

in vec2 texCoords;
out vec2 fragTexCoords;

void main()
{
    gl_Position = proj * view * model * vec4( position, 1.0 );
    fragTexCoords = texCoords;
}