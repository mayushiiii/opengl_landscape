#version 330

// Input
in vec2 texture_coord;
in vec3 world_position;
in vec3 world_normal;

// Uniform properties
uniform sampler2D u_texture_0;
uniform vec3 overrideColor;     // Uniform for custom color
uniform bool useOverrideColor;  // Uniform toggle for using custom color

// Output
layout(location = 0) out vec4 out_world_position;
layout(location = 1) out vec4 out_world_normal;
layout(location = 2) out vec4 out_color;

void main()
{
    vec4 textureColor = texture(u_texture_0, texture_coord); // Fetch texture color

    if (useOverrideColor) {
        // Use the override color instead of the texture
        out_color = vec4(overrideColor, 1.0);
    } else {
        // Use the texture color
        out_color = textureColor;
    }

    out_world_position = vec4(world_position, 1);
    out_world_normal = vec4(normalize(world_normal), 0);
}
