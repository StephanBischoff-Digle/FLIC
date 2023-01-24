#version 330 core

in vec2 fragTexCoords;

// NOTE: We might put the noise into the blue channel of the input texture.
uniform sampler2D inTexVec;         // RG -> XY of vector field
uniform sampler2D inTexNoise;       // noise (all channels)

uniform float arc_length;
uniform float scale;

out vec4 out_color;

vec2 tex2vec(vec2 coord) {
    return (texture(inTexVec, coord).rg - .5) * scale;
}

void main()
{
    float col = texture(inTexNoise, fragTexCoords).r;
    int n = 0;
    float d = .0001;

    if (length(texture(inTexVec, fragTexCoords).rg) < 0.000001) {
        out_color = vec4(0.);
        return;
    }

    // forward integration and convolution
    vec2 s0 = fragTexCoords;
    for (int i = 0; i < arc_length / 2.; ++i) {
        vec2 delta = tex2vec(s0) * d;
        if (length(delta) < 0.000001) break;
        s0 += delta;
        col += texture(inTexNoise, s0).r;
        n++;
    }

    // backward integration and convolution
    vec2 s1 = fragTexCoords;
    for (int i = 0; i < arc_length / 2.; ++i) {
        vec2 delta = tex2vec(s0) * d;
        if (length(delta) < 0.000001) break;
        s1 -= delta;
        col += texture(inTexNoise, s1).r;
        n++;
    }

    // box-kernel normalization
    col /= float(n);

    out_color = vec4(vec3(col), 1.);
}
