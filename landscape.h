#pragma once

#include <string>
#include <vector>

#include "components/simple_scene.h"
#include "components/transform.h"
#include "core/gpu/frame_buffer.h"
#include "core/gpu/particle_effect.h"


namespace m2
{
    struct LightInfo
    {
        glm::vec3 position;
        glm::vec3 color;
        float radius;
    };

    class Lab5 : public gfxc::SimpleScene
    {
     public:
        Lab5();
        ~Lab5();

        void Init() override;

     private:
        void FrameStart() override;
        void Update(float deltaTimeSeconds) override;
        void FrameEnd() override;

        void OnInputUpdate(float deltaTime, int mods) override;
        void OnKeyPress(int key, int mods) override;
        void OnKeyRelease(int key, int mods) override;
        void OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY) override;
        void OnMouseBtnPress(int mouseX, int mouseY, int button, int mods) override;
        void OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods) override;
        void OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY) override;
        void OnWindowResize(int width, int height) override;

        void LoadShader(const std::string &fileName);

        void LoadParticleShader(const std::string& name,
            const std::string& VS, const std::string& FS, const std::string& GS = "",
            bool hasGeomtery = false);
        void CreateMesh(const char* name, const std::vector<VertexFormat>& vertices, const std::vector<unsigned int>& indices);
        unsigned int UploadCubeMapTexture(const std::string& pos_x, const std::string& pos_y, const std::string& pos_z, const std::string& neg_x, const std::string& neg_y, const std::string& neg_z);
        void CreateFramebuffer(int width, int height);
        GLuint reflectionFBO, reflectionTexture;
        int screenWidth , screenHeight;
        void Lab5::ResetBezierParticles(int xSize, int ySize, int zSize);
        Mesh* Lab5::GenerateSurface(const char* name, float m, float n);

     private:
        FrameBuffer *frameBuffer;
        FrameBuffer *lightBuffer;
        std::vector<LightInfo> lights;
        int outputType;

        int cubeMapTextureID;
        float angle;
        unsigned int framebuffer_object;
        unsigned int color_texture;
        unsigned int depth_texture;
        unsigned int type;
        glm::vec3 generator_position;
        GLenum polygonMode;
        int scene;
        std::vector<glm::vec3> bezierPoints;
        float offset;
        int pause; // de revazut
        int m, n;
    };
}   // namespace m2
