#version 330 core

in vec2 fragTexCoords;

uniform sampler2D inTexture;

out vec4 out_color;

void main()
{
    out_color = texture( inTexture, fragTexCoords );
}
