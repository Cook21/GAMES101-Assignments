//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Scene.hpp"
#include <cstdint>
#include <fstream>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
const int taskNum = 60;
Vector3f castRay(const Scene& scene, const Ray& ray, const float& spp)
{
    Vector3f currPixel;
    for (int k = 0; k < spp; k++) {
        currPixel += scene.castRay(ray, 0) / spp;
    }
    return currPixel;
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount, original:16
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    std::vector<std::future<Vector3f>> futbuffer(taskNum);
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            Vector3f dir = normalize(Vector3f(-x, y, 1));
            futbuffer[m % taskNum] = std::async(std::launch::async, castRay, std::ref(scene), Ray(eye_pos, dir), spp);
            m++;
            if (m % taskNum == 0) {
                int base = m - taskNum;
                for (int i = 0; i < taskNum; i++) {
                    framebuffer[base + i] = futbuffer[i].get();
                }
                UpdateProgress(m / (float)framebuffer.size());
            }
        }
    }
    int base = m - m%taskNum;
    for (int i = 0; i < m%taskNum; i++) {
        framebuffer[base + i] = futbuffer[i].get();
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
