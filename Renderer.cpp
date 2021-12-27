//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include <mutex>
#include <vector>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

std::mutex _mtx;
int total_completes;


void render_multi_thread(std::vector<Vector3f> &framebuffer, const Scene &scene,
                         const int spp, const int height_start, const int height_end)
{
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    for (int j = height_start; j < height_end; j++)
    {
        for (int i = 0; i < scene.width; i++)
        {
            int index = j * scene.width + i;
            for (int k = 0; k < spp; k++)
            {
                float x = (2 * (i + get_random_float()) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
                float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;
                Vector3f dir = normalize(Vector3f(-x, y, 1));
                framebuffer[index] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
        }
        _mtx.lock();
        total_completes++;
        UpdateProgress(total_completes / (float)scene.height);
        _mtx.unlock();
    }
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene, const int spp)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    total_completes = 0;
    int num_threads = std::thread::hardware_concurrency();
    int concurrent_lines = scene.height / num_threads;
    int remainder = scene.height % num_threads;
    int num_tasks = remainder == 0 ? num_threads : num_threads + 1;
    std::vector<std::thread> workers;

    std::cout << "SPP: " << spp << "\n";

    for (int i = 0; i < num_tasks; i++)
    {
        int height_start = i * concurrent_lines;
        int height_end = std::min(height_start + concurrent_lines, scene.height);
        std::cout << "id - " << i << ": " << height_start << " => " << height_end << std::endl;
        workers.push_back(std::thread(render_multi_thread, std::ref(framebuffer), std::ref(scene),
                                      spp, height_start, height_end));
    }

    for (int i = 0; i < workers.size(); i++)
    {
        workers.at(i).join();
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
