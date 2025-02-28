#version 330

// Input
in vec3 world_position;
in vec3 world_normal;

// Uniform properties
uniform samplerCube texture_cubemap;
uniform int type;

uniform vec3 camera_position;

// Output
layout(location = 0) out vec4 out_color;


vec3 myReflect()
{
    // TODO(student): Compute the reflection color value
    return texture(texture_cubemap, reflect(normalize(world_position - camera_position), normalize(world_normal))).xyz;

}


vec3 myRefract(float refractive_index)
{
    // TODO(student): Compute the refraction color value
    return texture(texture_cubemap, refract(normalize(world_position - camera_position), normalize(world_normal), 1/refractive_index)).xyz;

}


void main()
{

    if (type == 0)
    {
        out_color = vec4(myReflect(), 0);
    }
    else
    {
        out_color = vec4(myRefract(1.1), 0);
    }
}
