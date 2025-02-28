#include "./landscape.h"

#include <vector>
#include <iostream>
#include <glm/gtc/noise.hpp>

#include "stb/stb_image.h"

using namespace std;
using namespace m2;

struct BezierWaterParticle
{
    glm::vec4 position;
    glm::vec4 speed;
    glm::vec4 initialPos;
    glm::vec4 initialSpeed;
    float delay;
    float initialDelay;
    float lifetime;
    float initialLifetime;

    BezierWaterParticle() {}

    BezierWaterParticle(const glm::vec4& pos, const glm::vec4& speed)
    {
        SetInitial(pos, speed);
    }

    void SetInitial(const glm::vec4& pos, const glm::vec4& speed,
        float delay = 0, float lifetime = 0)
    {
        position = pos;
        initialPos = pos;

        this->speed = speed;
        initialSpeed = speed;

        this->delay = delay;
        initialDelay = delay;

        this->lifetime = lifetime;
        initialLifetime = lifetime;
    }
};

ParticleEffect<BezierWaterParticle>* particleEffectBezier;

//Generates a random value between 0 and 1.
inline float Rand01()
{
    return rand() / static_cast<float>(RAND_MAX);
}

float random(const glm::vec2& st) {
    return glm::fract(sin(glm::dot(st, glm::vec2(12.9898, 78.233))) * 43758.5453123f);
}

/*
 *  To find out more about `FrameStart`, `Update`, `FrameEnd`
 *  and the order in which they are called, see `world.cpp`.
 */
float Noise(glm::vec2 st) {
    glm::vec2 i = floor(st);
    glm::vec2 f = fract(st);

    // Four corners in 2D of a tile
    float a = random(i);
    float b = random(i + glm::vec2(1.0, 0.0));
    float c = random(i + glm::vec2(0.0, 1.0));
    float d = random(i + glm::vec2(1.0, 1.0));

    // Smooth Interpolation

    // Cubic Hermine Curve.  Same as SmoothStep()
    glm::vec2 u = f * f * (glm::vec2(3.0f) - 2.0f * f);
    // u = smoothstep(0.,1.,f);

    // Mix 4 coorners percentages
    return glm::mix(a, b, u.x) +
        (c - a) * u.y * (1.0 - u.x) +
        (d - b) * u.x * u.y;
}

glm::vec3 Bezier(float t) {
    glm::vec3 p0(-15.0f, 10.0f, 0.0f);
    glm::vec3 p1(-10.0f, 6.0f, 0.0f);
    glm::vec3 p2(-6.0f, 3.0f, 0.0f);
    glm::vec3 p3(-4.0f, 0.0f, 0.0f);

	return p0 * glm::vec3(pow((1 - t), 3)) +
		p1 * glm::vec3((3 * t * pow((1 - t), 2))) +
		p2 * glm::vec3((3 * pow(t, 2) * (1 - t))) +
		p3 * glm::vec3(pow(t, 3));
}

float InterpolationBezier(glm::vec3 point, float& dist) {
    float min_distance = 1e10;
    float closest_t = 0.0;

    for (float t = 0.0; t <= 1.0; t += 0.01) {
        glm::vec3 bezier_point = Bezier(t);
        float distance = glm::sqrt((point.x - bezier_point.x) * (point.x - bezier_point.x) + (point.z - bezier_point.z) * (point.z - bezier_point.z));

        if (distance < min_distance) {
            min_distance = distance;
            closest_t = t;
        }
    }

    dist = min_distance;
    return closest_t;

}

float BezierHeight(glm::vec3 point) {
    float h = point.y;
	float dist = 0;
	float t = InterpolationBezier(point, dist);


    if (dist > 2.5f) {
        return h;
    }
    
    glm::vec3 b_closest = Bezier(t);
    float normalized = glm::clamp(dist / 4.0f, 0.0f, 1.0f);
    float y = glm::mix(b_closest.y, h, 1 - std::sin(M_PI * 0.5 - normalized * M_PI * 0.5));

    return y;
}

static float FindClosestBezierT(float x, float z) {
    float left = 0.0f, right = 1.0f;
    float closestT = 0.0f;
    float minDistance = std::numeric_limits<float>::max();

    for (int i = 0; i < 10; ++i) {
        float mid = (left + right) / 2.0f;

        glm::vec3 point = Bezier(mid);
        float distance = glm::distance(glm::vec2(x, z), glm::vec2(point.x, point.z));

        if (distance < minDistance) {
            minDistance = distance;
            closestT = mid;
        }

        if (x < point.x) {
            right = mid;
        }
        else {
            left = mid;
        }
    }
    return closestT;
}

static std::vector<glm::vec3> GenerateBezierPoints(int numPoints) {
    std::vector<glm::vec3> points;

    for (int i = 0; i <= numPoints; ++i) {
        float t = float(i) / (2 * numPoints);
        points.push_back(Bezier(t));    
    }

    return points;
}

void Lab5::ResetBezierParticles(int xSize, int ySize, int zSize)
{
    unsigned int nrParticles = 10000;

    particleEffectBezier = new ParticleEffect<BezierWaterParticle>();
    particleEffectBezier->Generate(nrParticles, true);

    unsigned int bezierPointCount = bezierPoints.size();


    auto particleSSBO = particleEffectBezier->GetParticleBuffer();
    BezierWaterParticle* data = const_cast<BezierWaterParticle*>(particleSSBO->GetBuffer());


    int xhSize = xSize / 2;
    int yhSize = ySize / 2;
    int zhSize = zSize / 2;

    for (unsigned int i = 0; i < nrParticles; i++)
    {

        int startIndex = rand() % (bezierPointCount - 1);
        glm::vec3 startPosition = bezierPoints[startIndex];
        glm::vec3 nextPosition = bezierPoints[startIndex + 1];

        glm::vec4 pos(1);
        pos.x = startPosition.x + 5;
        pos.y = startPosition.y + rand() % 3 - 13;
        pos.z = startPosition.z + float(rand() % 300) / 100.0f - 2.0f;

        glm::vec3 direction = glm::normalize(nextPosition - startPosition);
        glm::vec4 speed(0);
        speed.x = direction.x * 3.0f;
        speed.y = direction.y * 3.0f;
        speed.z = direction.z * 3.0f;

        float delay = (rand() % 100 / 100.0f) * 3.0f;

        data[i].SetInitial(pos, speed, delay);
    }

    particleSSBO->SetBufferData(data);
}



Mesh* Lab5::GenerateSurface(const char* name, float m, float n)
{
    std::vector<VertexFormat> vertices;
    std::vector<unsigned int> indices;
    glm::vec3 center(0, 0, 0);
    float radius = 0.3f * m;
    float h_max = 0.8f * m;

    float m2 = m / 2;
    float n2 = n / 2;
    int cnt = 0;
    for (float i = -m2; i < m2; i += 1.0f)
    {
        for (float j = -n2; j < n2; j += 1.0f)
        {
            vertices.push_back(VertexFormat(glm::vec3(i, 0, j)));
            cnt += 1;
        }
    }
    // Generate indices
    for (int i = 0; i < m - 1; ++i) {
        for (int j = 0; j < n - 1; ++j) {
            // Calculate the indices of the four corners of the current quad
            int topLeft = i * n + j;
            int topRight = topLeft + 1;
            int bottomLeft = (i + 1) * n + j;
            int bottomRight = bottomLeft + 1;

            // First triangle of the quad
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            // Second triangle of the quad
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }



    // Modify vertices using Perlin noise
    float noiseScale = 0.1f;  // Scale for noise frequency
    float noiseAmplitude = 2.0f;  // Scale for noise intensity

    for (auto& vertex : vertices) {
        glm::vec3 posXZ = glm::vec3(vertex.position.x, 0, vertex.position.z);

        float distance = glm::distance(posXZ, center);
        float d = glm::clamp(distance / radius, 0.0f, 1.0f);

        if (d < 1.0f) {
            vertex.position.y = (d * d / 2) * h_max;
        }
        else {
            vertex.position.y = (1.0f - (2.0f - d) * (2.0f - d) / 2) * h_max;
        }

        // Apply Perlin noise to modify height
        float noiseValue = glm::simplex(glm::vec2(vertex.position.x * noiseScale, vertex.position.z * noiseScale));

        vertex.position.y += noiseValue * noiseAmplitude;
        vertex.text_coord = glm::vec2(vertex.position.x, vertex.position.z);
    }

    for (auto& vertex : vertices) {
        if (vertex.position.y < 2 || vertex.position.x >= 10 || (vertex.position.z < -2 || vertex.position.z > 1)) continue;

        float t = FindClosestBezierT(vertex.position.x, vertex.position.z);
        glm::vec3 bezierPoint = Bezier(t);
        float d_bezier = glm::distance(glm::vec2(vertex.position.x, vertex.position.z), glm::vec2(bezierPoint.x, bezierPoint.z));

        vertex.position.y = glm::mix(vertex.position.y, bezierPoint.y, 1.0f - sin(glm::clamp(d_bezier / radius, 0.0f, 1.0f) * glm::pi<float>() / 2.0f));
    }

    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int i0 = indices[i];
        unsigned int i1 = indices[i + 1];
        unsigned int i2 = indices[i + 2];

        glm::vec3 v0 = vertices[i0].position;
        glm::vec3 v1 = vertices[i1].position;
        glm::vec3 v2 = vertices[i2].position;

        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;

        glm::vec3 triangleNormal = glm::normalize(glm::cross(edge1, edge2));

        vertices[i0].normal = triangleNormal;
        vertices[i0].color = triangleNormal;

        vertices[i1].normal = triangleNormal;
        vertices[i1].color = triangleNormal;

        vertices[i2].normal = triangleNormal;
        vertices[i2].color = triangleNormal;
    }


    Mesh* obs = new Mesh(name);
    obs->InitFromData(vertices, indices);
    obs->UseMaterials(false);
    return obs;
}

unsigned int Lab5::UploadCubeMapTexture(const std::string& pos_x, const std::string& pos_y, const std::string& pos_z, const std::string& neg_x, const std::string& neg_y, const std::string& neg_z)
{
    int width, height, chn;

    unsigned char* data_pos_x = stbi_load(pos_x.c_str(), &width, &height, &chn, 0);
    unsigned char* data_pos_y = stbi_load(pos_y.c_str(), &width, &height, &chn, 0);
    unsigned char* data_pos_z = stbi_load(pos_z.c_str(), &width, &height, &chn, 0);
    unsigned char* data_neg_x = stbi_load(neg_x.c_str(), &width, &height, &chn, 0);
    unsigned char* data_neg_y = stbi_load(neg_y.c_str(), &width, &height, &chn, 0);
    unsigned char* data_neg_z = stbi_load(neg_z.c_str(), &width, &height, &chn, 0);

    unsigned int textureID = 0;
    // TODO(student): Create the texture
    glGenTextures(1, &textureID);
    // TODO(student): Bind the texture
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (GLEW_EXT_texture_filter_anisotropic) {
        float maxAnisotropy;

        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisotropy);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAnisotropy);
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // TODO(student): Load texture information for each face
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data_pos_x);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data_pos_y);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data_pos_z);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data_neg_x);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data_neg_y);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data_neg_z);


    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
    if (GetOpenGLError() == GL_INVALID_OPERATION)
    {
        cout << "\t[NOTE] : For students : DON'T PANIC! This error should go away when completing the tasks." << std::endl;
    }

    // Free memory
    SAFE_FREE(data_pos_x);
    SAFE_FREE(data_pos_y);
    SAFE_FREE(data_pos_z);
    SAFE_FREE(data_neg_x);
    SAFE_FREE(data_neg_y);
    SAFE_FREE(data_neg_z);

    return textureID;
}

void Lab5::CreateFramebuffer(int width, int height)
{
    // TODO(student): In this method, use the attributes
    // 'framebuffer_object', 'color_texture'
    // declared in lab6.h

    // TODO(student): Generate and bind the framebuffer
    glGenFramebuffers(1, &framebuffer_object);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_object);


    // TODO(student): Generate and bind the color texture
    glGenTextures(1, &color_texture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, color_texture);



    // TODO(student): Initialize the color textures
    for (int i = 0; i < 6; i++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    }



    if (color_texture) {
        //cubemap params
        glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        if (GLEW_EXT_texture_filter_anisotropic) {
            float maxAnisotropy;

            glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisotropy);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAnisotropy);
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        // Bind the color textures to the framebuffer as a color attachments
        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_texture, 0);

        glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

        std::vector<GLenum> draw_textures;
        draw_textures.push_back(GL_COLOR_ATTACHMENT0);
        glDrawBuffers(draw_textures.size(), &draw_textures[0]);

    }

    // TODO(student): Generate and bind the depth texture
    glGenTextures(1, &depth_texture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, depth_texture);


    // TODO(student): Initialize the depth textures
    for (int i = 0; i < 6; i++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    }

    if (depth_texture) {
        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_texture, 0);
    }

    glCheckFramebufferStatus(GL_FRAMEBUFFER);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


void Lab5::LoadParticleShader(const std::string& name, const std::string& VS, const std::string& FS, const std::string& GS, bool hasGeomtery)
{
    std::string shaderPath = PATH_JOIN(window->props.selfDir, SOURCE_PATH::M2, "lab5", "shaders");

    // Create a shader program for particle system
    {
        printf("Load particle shader %s\n", name.c_str());
        Shader* shader = new Shader(name);
        shader->AddShader(PATH_JOIN(shaderPath, VS + ".VS.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, FS + ".FS.glsl"), GL_FRAGMENT_SHADER);
        if (hasGeomtery)
        {
            shader->AddShader(PATH_JOIN(shaderPath, GS + ".GS.glsl"), GL_GEOMETRY_SHADER);
        }

        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }
}

Lab5::Lab5()
{
    framebuffer_object = 0;
    color_texture = 0;
    depth_texture = 0;

    angle = 0;

    type = 0;
    pause = 0;
    m = 20;
    n = 20;

}


Lab5::~Lab5()
{
}


void Lab5::Init()
{
    outputType = 6;

    auto camera = GetSceneCamera();
    camera->SetPositionAndRotation(glm::vec3(0, 2, 3.5), glm::quat(glm::vec3(-20 * TO_RADIANS, 0, 0)));
    camera->Update();

    int screenWidth = window->GetResolution().x;
    int screenHeight = window->GetResolution().y;

    TextureManager::LoadTexture(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES), "ground.jpg");
    std::string texturePath = PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES, "cube");
    std::string shaderPath = PATH_JOIN(window->props.selfDir, SOURCE_PATH::M2, "lab5", "shaders");
    TextureManager::LoadTexture(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES), "blue.jpg");

    // Load a mesh from file into GPU memory
    {
        {
            Mesh* mesh = new Mesh("plane");
            mesh->LoadMesh(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::MODELS, "primitives"), "plane50.obj");
            mesh->UseMaterials(false);
            meshes[mesh->GetMeshID()] = mesh;
        }

        {
            Mesh* mesh = new Mesh("cube");
            mesh->LoadMesh(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::MODELS, "primitives"), "box.obj");
            mesh->UseMaterials(false);
            meshes[mesh->GetMeshID()] = mesh;
        }

        {
            Mesh* mesh = new Mesh("sphere");
            mesh->LoadMesh(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::MODELS, "primitives"), "sphere.obj");
            mesh->UseMaterials(false);
            meshes[mesh->GetMeshID()] = mesh;
        }

        {
            Mesh* mesh = new Mesh("quad");
            mesh->LoadMesh(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::MODELS, "primitives"), "quad.obj");
            mesh->UseMaterials(false);
            meshes[mesh->GetMeshID()] = mesh;
        }

        {
            Mesh* mesh = new Mesh("snowflake");
            mesh->LoadMesh(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::MODELS, "other"), "snow.obj");
            mesh->UseMaterials(false);
            meshes[mesh->GetMeshID()] = mesh;
        }

        // teren
        {
            Mesh* mesh = GenerateSurface("Mountain", m, n);
            meshes[mesh->GetMeshID()] = mesh;
        }
    }
    // Create a shader program for rendering cubemap texture
    {
        Shader* shader = new Shader("CubeMap");
        shader->AddShader(PATH_JOIN(shaderPath, "CubeMap.VS.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "CubeMap.FS.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Create a shader program for standard rendering
    {
        Shader* shader = new Shader("ShaderNormal");
        shader->AddShader(PATH_JOIN(shaderPath, "Normal.VS.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "Normal.FS.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Create a shader program for creating a CUBEMAP
    {
        Shader* shader = new Shader("Framebuffer");
        shader->AddShader(PATH_JOIN(shaderPath, "Framebuffer.VS.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "Framebuffer.FS.glsl"), GL_FRAGMENT_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "Framebuffer.GS.glsl"), GL_GEOMETRY_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }

    // Shader for bezier particle effects
    LoadParticleShader("Water", "ParticleBezier", "Particle_simple", "Particle", true); // de revenit

    generator_position = glm::vec3(0, 0, 0);
    offset = 0.05;
    
    cubeMapTextureID = UploadCubeMapTexture(
        PATH_JOIN(texturePath, "pos_x.jpg"),
        PATH_JOIN(texturePath, "pos_y.jpg"),
        PATH_JOIN(texturePath, "pos_z.jpg"),
        PATH_JOIN(texturePath, "neg_x.jpg"),
        PATH_JOIN(texturePath, "neg_y.jpg"),
        PATH_JOIN(texturePath, "neg_z.jpg"));

    TextureManager::LoadTexture(texturePath, "pos_y.jpg");


    LoadShader("Render2Texture");
    LoadShader("Composition");
    LoadShader("LightPass");

    {
        Shader* shader = new Shader("CubeMap");
        shader->AddShader(PATH_JOIN(shaderPath, "CubeMap.VS.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "CubeMap.FS.glsl"), GL_FRAGMENT_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }


    // Create a shader program for creating a CUBEMAP
    {
        Shader* shader = new Shader("Framebuffer");
        shader->AddShader(PATH_JOIN(shaderPath, "Framebuffer.VS.glsl"), GL_VERTEX_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "Framebuffer.FS.glsl"), GL_FRAGMENT_SHADER);
        shader->AddShader(PATH_JOIN(shaderPath, "Framebuffer.GS.glsl"), GL_GEOMETRY_SHADER);
        shader->CreateAndLink();
        shaders[shader->GetName()] = shader;
    }


    TextureManager::LoadTexture(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES), "cat.jpeg");
    TextureManager::LoadTexture(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES), "image.png");
    TextureManager::LoadTexture(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES), "snowflake.png");
    TextureManager::LoadTexture(PATH_JOIN(window->props.selfDir, RESOURCE_PATH::TEXTURES), "mountain2.jpg");

    auto resolution = window->GetResolution();

    frameBuffer = new FrameBuffer();
    frameBuffer->Generate(resolution.x, resolution.y, 3);
    //frameBuffer contains 3 textures (position, normal and color)

    lightBuffer = new FrameBuffer();
    lightBuffer->Generate(resolution.x, resolution.y, 1, false);
    //lightBuffer contains 1 texture (light accumulation)

    for (int i = 0; i < 60; ++i)
    {
        LightInfo lightInfo;

        // The chosen position is between (-10, 0, -10) and (10, 3, 10)
        // The chosen color is between (0, 0, 0) and (1, 1, 1).
        // The chosen radius is between 3 and 4.

        lightInfo.position = glm::vec3(0.0f);
        lightInfo.color = glm::vec3(0.0f);
        lightInfo.radius = 1.0f;

        lightInfo.position = glm::vec3(Rand01() * 20 - 10, Rand01() * 15 - 10, Rand01() * 20 - 10);
        //lightInfo.color = glm::vec3(Rand01(), Rand01(), Rand01());
        lightInfo.color = glm::vec3(1);
        lightInfo.radius = Rand01() + 4;


        lights.push_back(lightInfo);
    }
    
    bezierPoints = GenerateBezierPoints(100);
    CreateFramebuffer(1024, 1024);
    ResetBezierParticles(20, 15, 10);
}

void Lab5::CreateMesh(const char* name, const std::vector<VertexFormat>& vertices, const std::vector<unsigned int>& indices)
{
    unsigned int VAO = 0;
    // TODO(student): Create the VAO and bind it
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    unsigned int VBO = 0;
    // TODO(student): Create the VBO and bind it
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    // TODO(student): Send vertices data into the VBO buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices[0]) * vertices.size(), &vertices[0], GL_STATIC_DRAW);

    unsigned int IBO = 0;
    // TODO(student): Create the IBO and bind it
    glGenBuffers(1, &IBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);

    // TODO(student): Send indices data into the IBO buffer
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices[0]) * indices.size(), &indices[0], GL_STATIC_DRAW);
    // ========================================================================
    // This section demonstrates how the GPU vertex shader program
    // receives data. It will be learned later, when GLSL shaders will be
    // introduced. For the moment, just think that each property value from
    // our vertex format needs to be sent to a certain channel, in order to
    // know how to receive it in the GLSL vertex shader.

    // Set vertex position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), 0);

    // Set vertex normal attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)(sizeof(glm::vec3)));

    // Set texture coordinate attribute
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)(2 * sizeof(glm::vec3)));

    // Set vertex color attribute
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)(2 * sizeof(glm::vec3) + sizeof(glm::vec2)));
    // ========================================================================

    // TODO(student): Unbind the VAO
    glBindVertexArray(0);

    // Check for OpenGL errors
    if (GetOpenGLError() == GL_INVALID_OPERATION)
    {
        cout << "\t[NOTE] : For students : DON'T PANIC! This error should go away when completing the tasks." << std::endl;
        cout << "\t[NOTE] : For developers : This happens because OpenGL core spec >=3.1 forbids null VAOs." << std::endl;
    }

    // Mesh information is saved into a Mesh object
    meshes[name] = new Mesh(name);
    meshes[name]->InitFromBuffer(VAO, static_cast<unsigned int>(indices.size()));
}


void Lab5::FrameStart()
{
}


void Lab5::Update(float deltaTimeSeconds)
{
    ClearScreen();
    auto camera = GetSceneCamera();
    glm::vec3 cameraPosition = camera->m_transform->GetWorldPosition();
    glLineWidth(3);

    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_ONE, GL_ONE);
    glBlendEquation(GL_FUNC_ADD);

    for (auto& l : lights)
    {
        // the lights orbit around the center of the scene
        float rotationRadians = RADIANS(60) * deltaTimeSeconds;

        glm::mat4 rotateMatrix = glm::rotate(glm::mat4(1.0f), rotationRadians, glm::vec3(0, 1, 0));
        l.position = rotateMatrix * glm::vec4(l.position, 1.0f);
    }


    if (framebuffer_object) {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_object);
        // Set the clear color for the color buffer
        glClearColor(0, 0, 0, 1);
        // Clears the color buffer (using the previously set color) and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glViewport(0, 0, 1024, 1024);


        Shader* shader = shaders["Framebuffer"];
        shader->Use();

        glm::mat4 projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 100.0f);

        {
            glm::mat4 modelMatrix = glm::scale(glm::mat4(1), glm::vec3(30));

            glUniformMatrix4fv(shader->loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));
            glUniformMatrix4fv(shader->loc_view_matrix, 1, GL_FALSE, glm::value_ptr(camera->GetViewMatrix()));
            glUniformMatrix4fv(shader->loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(projection));

            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTextureID);
            glUniform1i(glGetUniformLocation(shader->program, "texture_cubemap"), 1);

            glUniform1i(glGetUniformLocation(shader->program, "cube_draw"), 1);

            meshes["cube"]->Render();
        }

        glBindTexture(GL_TEXTURE_CUBE_MAP, color_texture);
        glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

        //reset drawing to screen
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }


    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, window->GetResolution().x, window->GetResolution().y);
    glEnable(GL_DEPTH_TEST);
    // ------------------------------------------------------------------------
    // Deferred rendering pass
    {
        // Bind the framebuffer
        frameBuffer->Bind();

        // Clear the screen ONCE before rendering
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set the viewport
        glViewport(0, 0, window->GetResolution().x, window->GetResolution().y);

        // ---------------------------------------
        // Render convex cavity mesh
        auto shader = shaders["Render2Texture"];

        shader->Use();


        // Set uniforms for custom color
        glUniform3f(shader->GetUniformLocation("overrideColor"), 0.74f, 0.84f, 0.92f);
        glUniform1i(shader->GetUniformLocation("useOverrideColor"), 0);

        TextureManager::GetTexture("mountain2.jpg")->BindToTextureUnit(GL_TEXTURE0);
        RenderMesh(meshes["Mountain"], shader, glm::vec3(0, -10.5f, 0), glm::vec3(1));
        glUniform1i(shader->GetUniformLocation("useOverrideColor"), 0);
        // Disable override color for other objects
        glUniform1i(shader->GetUniformLocation("useOverrideColor"), 0);

        // Render lights or other objects
        //TextureManager::GetTexture("snowflake.png")->BindToTextureUnit(GL_TEXTURE0);
        for (auto& l : lights) {
            auto model = glm::translate(glm::mat4(1), l.position);
            model *= glm::scale(model, glm::vec3(0.5f));
            RenderMesh(meshes["snowflake"], shader, model);
        }

    }
    // ---------------------------------------
    // Render the cubemap
    {
        Shader* cubemapShader = shaders["ShaderNormal"];
        cubemapShader->Use();

        glm::mat4 modelMatrix = glm::scale(glm::mat4(1), glm::vec3(30));

        glUniformMatrix4fv(cubemapShader->loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));
        glUniformMatrix4fv(cubemapShader->loc_view_matrix, 1, GL_FALSE, glm::value_ptr(camera->GetViewMatrix()));
        glUniformMatrix4fv(cubemapShader->loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(camera->GetProjectionMatrix()));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTextureID);
        int loc_texture = cubemapShader->GetUniformLocation("texture_cubemap");
        glUniform1i(loc_texture, 0);

        meshes["cube"]->Render();
    }


    // Draw the reflection on the mesh
    {
        Shader* shader = shaders["CubeMap"];
        shader->Use();

        glm::mat4 modelMatrix = glm::scale(glm::mat4(1), glm::vec3(0.1f));

        glUniformMatrix4fv(shader->loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));
        glUniformMatrix4fv(shader->loc_view_matrix, 1, GL_FALSE, glm::value_ptr(camera->GetViewMatrix()));
        glUniformMatrix4fv(shader->loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(camera->GetProjectionMatrix()));

        auto cameraPosition = camera->m_transform->GetWorldPosition();

        if (!color_texture) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTextureID);
            int loc_texture = shader->GetUniformLocation("texture_cubemap");
            glUniform1i(loc_texture, 0);
        }

        if (color_texture) {
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_CUBE_MAP, color_texture);
            int loc_texture2 = shader->GetUniformLocation("texture_cubemap");
            glUniform1i(loc_texture2, 1);
        }


        int loc_camera = shader->GetUniformLocation("camera_position");
        glUniform3f(loc_camera, cameraPosition.x, cameraPosition.y, cameraPosition.z);

        glUniform1i(shader->GetUniformLocation("type"), type);

        shader = shaders["Render2Texture"];

        shader->Use();
        glUniform1i(shader->GetUniformLocation("useOverrideColor"), 0);

        TextureManager::GetTexture("pos_y.jpg")->BindToTextureUnit(GL_TEXTURE0);

        // Render lake in the middle of the cavity
        RenderMesh(meshes["plane"], shader, glm::vec3(0, -6.5f, 0), glm::vec3(0.35f));
    }
    // Particle system

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    {
        Shader* shader = shaders["Water"];
        shader->Use();

        // TODO(student): Send correct texture for rain
        TextureManager::GetTexture("blue.jpg")->BindToTextureUnit(GL_TEXTURE0);

        particleEffectBezier->Render(GetSceneCamera(), shader);

        // TODO(student): Send uniforms generator_position,
        // deltaTime and offset to the shader

        glUniform3fv(shader->GetUniformLocation("generator_position"), 1, glm::value_ptr(generator_position));
        glUniform1f(shader->GetUniformLocation("deltaTime"), deltaTimeSeconds);
        glUniform1f(shader->GetUniformLocation("offset"), offset);
        glUniform1i(shader->GetUniformLocation("pause"), pause);

    }
    //glDisable(GL_DEPTH_TEST);

    // ------------------------------------------------------------------------
    // Lighting pass
    {
        glm::vec3 ambientLight(0.2f);
        //Set the initial light accumulation in each pixel to be equal to the ambient light.
        lightBuffer->SetClearColor(glm::vec4(ambientLight.x, ambientLight.y, ambientLight.z, 1.0f));
        lightBuffer->Bind();
        glClearColor(0, 0, 0, 1);

        // Enable buffer color accumulation
        
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_ONE, GL_ONE);

        auto shader = shaders["LightPass"];
        shader->Use();

        {
            int texturePositionsLoc = shader->GetUniformLocation("texture_position");
            glUniform1i(texturePositionsLoc, 0);
            frameBuffer->BindTexture(0, GL_TEXTURE0);
        }

        {
            int textureNormalsLoc = shader->GetUniformLocation("texture_normal");
            glUniform1i(textureNormalsLoc, 1);
            frameBuffer->BindTexture(1, GL_TEXTURE0 + 1);
        }

        auto camera = GetSceneCamera();
        glm::vec3 cameraPos = camera->m_transform->GetWorldPosition();
        int loc_eyePosition = shader->GetUniformLocation("eye_position");
        glUniform3fv(loc_eyePosition, 1, glm::value_ptr(cameraPos));

        auto resolution = window->GetResolution();
        int loc_resolution = shader->GetUniformLocation("resolution");
        glUniform2i(loc_resolution, resolution.x, resolution.y);

        //Front face culling
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        for (auto& lightInfo : lights)
        {
            // TODO(student): Set the shader uniforms 'light_position', 'light_color' and 'light_radius'
            // with the values from the light source. Use shader 'shader'.

            int lightPosition = shader->GetUniformLocation("light_position");
            glUniform3fv(lightPosition, 1, glm::value_ptr(lightInfo.position));

            int loc_lightColor = shader->GetUniformLocation("light_color");
            glUniform3fv(loc_lightColor, 1, glm::value_ptr(lightInfo.color));

            int loc_lightRadius = shader->GetUniformLocation("light_radius");
            glUniform1f(loc_lightRadius, lightInfo.radius);

            // TODO(student): Draw the mesh "sphere" at the position of the light source
            // and scaled 2 times the light source radius.
            // Use RenderMesh(mesh, shader, position, scale). Use shader 'shader'.

            RenderMesh(meshes["sphere"], shader, lightInfo.position, glm::vec3(lightInfo.radius * 2));

        }

        glDisable(GL_CULL_FACE);

        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }


    // ------------------------------------------------------------------------
    // Composition pass
    {
        FrameBuffer::BindDefault();

        auto shader = shaders["Composition"];
        shader->Use();

        int outputTypeLoc = shader->GetUniformLocation("output_type");
        glUniform1i(outputTypeLoc, outputType);

        {
            int texturePositionsLoc = shader->GetUniformLocation("texture_position");
            glUniform1i(texturePositionsLoc, 1);
            frameBuffer->BindTexture(0, GL_TEXTURE0 + 1);
        }

        {
            int textureNormalsLoc = shader->GetUniformLocation("texture_normal");
            glUniform1i(textureNormalsLoc, 2);
            frameBuffer->BindTexture(1, GL_TEXTURE0 + 2);
        }

        {
            int textureColorLoc = shader->GetUniformLocation("texture_color");
            glUniform1i(textureColorLoc, 3);
            frameBuffer->BindTexture(2, GL_TEXTURE0 + 3);
        }

        {
            int textureDepthLoc = shader->GetUniformLocation("texture_depth");
            glUniform1i(textureDepthLoc, 4);
            frameBuffer->BindDepthTexture(GL_TEXTURE0 + 4);
        }

        {
            int textureLightLoc = shader->GetUniformLocation("texture_light");
            glUniform1i(textureLightLoc, 5);
            lightBuffer->BindTexture(0, GL_TEXTURE0 + 5);
        }

        // Render the object again but with different properties
        TextureManager::GetTexture("cat.jpeg")->BindToTextureUnit(GL_TEXTURE0);
        RenderMesh(meshes["quad"], shader, glm::vec3(0, 0, 0));
    }

 


}


void Lab5::FrameEnd()
{
    DrawCoordinateSystem();
}


void Lab5::LoadShader(const std::string &name)
{
    std::string shaderPath = PATH_JOIN(window->props.selfDir, SOURCE_PATH::M2, "lab5", "shaders");

  if (name == "Framebuffer") {
            {
                Shader* shader = new Shader(name);
                shader->AddShader(PATH_JOIN(shaderPath, name + ".VS.glsl"), GL_VERTEX_SHADER);
                shader->AddShader(PATH_JOIN(shaderPath, name + ".FS.glsl"), GL_FRAGMENT_SHADER);
                shader->AddShader(PATH_JOIN(shaderPath, name + ".GS.glsl"), GL_GEOMETRY_SHADER);

                shader->CreateAndLink();
                shaders[shader->GetName()] = shader;
            }
        }
        else
        {
            Shader* shader = new Shader(name);
            shader->AddShader(PATH_JOIN(shaderPath, name + ".VS.glsl"), GL_VERTEX_SHADER);
            shader->AddShader(PATH_JOIN(shaderPath, name + ".FS.glsl"), GL_FRAGMENT_SHADER);

            shader->CreateAndLink();
            shaders[shader->GetName()] = shader;
        }
}


/*
 *  These are callback functions. To find more about callbacks and
 *  how they behave, see `input_controller.h`.
 */


void Lab5::OnInputUpdate(float deltaTime, int mods)
{
    // Treat continuous update based on input
}


void Lab5::OnKeyPress(int key, int mods)
{
    // Add key press event

    // These are the key mappings for compositing different passes.
    // What does each key seem to activate? Where can you find the
    // answer? Examine the source code to find out!
    int index = key - GLFW_KEY_0;
    if (index >= 0 && index <= 9)
    {
        outputType = index;
    }
}


void Lab5::OnKeyRelease(int key, int mods)
{
    // Add key release event
}


void Lab5::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
    // Add mouse move event
}


void Lab5::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{
    // Add mouse button press event
}


void Lab5::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
    // Add mouse button release event
}


void Lab5::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
    // Treat mouse scroll event
}


void Lab5::OnWindowResize(int width, int height)
{
    // Treat window resize event
    frameBuffer->Resize(width, height, 32);
    lightBuffer->Resize(width, height, 32);
}
