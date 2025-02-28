#version 330 core

// Inputs
in vec2 texCoords;

// Uniforms
uniform sampler2D rainTexture;

// Outputs
out vec4 FragColor;

void main() {
    vec4 texColor = texture(rainTexture, texCoords);

    // Discard black pixels
    if (texColor.rgb == vec3(0.0)) {
        discard;
    }

    FragColor = texColor;
}
