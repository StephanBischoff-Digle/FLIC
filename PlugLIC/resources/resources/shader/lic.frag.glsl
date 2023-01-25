#version 330 core

#define ARC_LENGTH 50
#define TAU 6.283185307

in vec2 pixelPoints; // WIP
in vec2 pixelVectors; //WIP

in vec2 fragTexCoords;
uniform int screenWidth;
uniform int screenHeight;

uniform sampler2D inTexture; // noise

out vec4 out_color;

vec2 vec_field(vec2 uv) {
    float a = fract(uv.x * 2.) * TAU;
    float b = fract(uv.y * 1.) * TAU;
    return vec2(cos(a), cos(b));
}


void main()
{
    // Inits
    vec2 resolution = vec2(screenWidth,screenHeight);
    vec2 uv = fragTexCoords; 
    vec2 d = 0.5 / resolution; // half a pixel step
    float col = texture2D(inTexture, uv).r;

    int n = 0;

    // forward streamline integration
    vec2 s0 = uv;
    for (int i = 0; i < ARC_LENGTH/2; i++)
    {
        vec2 delta = vec_field(s0) * d;
        if(length(delta) < 0.00001) break;
        s0+=delta;

        col += texture2D(inTexture, s0).r;
        n++;
    }

    // backward streamline integration
    vec2 s1 = uv;
    for (int i = 0; i < ARC_LENGTH/2; i++)
    {
        vec2 delta = vec_field(s1) * d;
        if(length(delta) < 0.00001) break;
        s1-=delta;

        col += texture2D(inTexture, s1).r;
        n++;
    }

    // Final
    col /= float(n);

    out_color = vec4(vec3(col),1.);
    //out_color = texture(inTexture, fragTexCoords);
}