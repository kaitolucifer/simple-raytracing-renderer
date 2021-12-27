//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::Add(Object *object)
{
    objects.push_back(object);
    // 累加光源面积
    if (object->hasEmit())
    {
        emitArea += object->getArea();
    }
}

void Scene::Add(std::unique_ptr<Light> light)
{
    lights.push_back(std::move(light));
}

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    // 对光源总面积进行采样并抽选某个光源
    float p = get_random_float() * emitArea;
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                // 对选中的光源计算光源方向和概率（1 / 该光源面积）
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

Vector3f Scene::shade(Intersection &hit_obj, Vector3f wo) const
{
    // 击中光源的情况（递归的base condition）
    if (hit_obj.m->hasEmission())
        return hit_obj.m->getEmission();

    const float epsilon = .0005f;

    // 直接光照
    Vector3f l_dir;
    // 随机采样一根到某个光源的光线，并计算概率（1 / 该光源表面积）
    float light_pdf = .0f;
    Intersection hit_light;
    sampleLight(hit_light, light_pdf);
    Vector3f obj_to_light = hit_light.coords - hit_obj.coords;
    Vector3f obj_to_light_dir = obj_to_light.normalized();
    // 检查光线是否被遮挡
    Intersection t = intersect(Ray(hit_obj.coords, obj_to_light_dir));
    // 没有被遮挡的情况
    if (t.distance - obj_to_light.norm() > -epsilon)
    {
        // 计算brdf
        Vector3f f_r = hit_obj.m->eval(obj_to_light_dir, wo, hit_obj.normal);
        // 计算光线衰减项
        float r2 = dotProduct(obj_to_light, obj_to_light);
        // 计算点的法线和光方向的夹角cos
        float cos_theta = std::max(.0f, dotProduct(hit_obj.normal, obj_to_light_dir));
        // 计算光的逆方向和光源法线的夹角cos
        float cos_theta_prime = std::max(.0f, dotProduct(hit_light.normal, -obj_to_light_dir));
        l_dir = hit_light.emit * f_r * cos_theta * cos_theta_prime / r2 / light_pdf;
    }

    // 间接光照
    Vector3f l_indir;
    if (get_random_float() < RussianRoulette)
    {
        // 在点的半圆上采样一个反射方向
        Vector3f obj_to_next_obj_dir = hit_obj.m->sample(wo, hit_obj.normal).normalized();
        // 如果是合法的方向pdf为1 / (2 * PI)
        float pdf = hit_obj.m->pdf(wo, obj_to_next_obj_dir, hit_obj.normal);
        if (pdf > .0f)
        {
            Intersection hit_next_obj = intersect(Ray(hit_obj.coords, obj_to_next_obj_dir));
            // 击中物体并且是非光源的情况
            if (hit_next_obj.happened && !hit_next_obj.m->hasEmission())
            {
                // 计算brdf
                Vector3f f_r = hit_obj.m->eval(obj_to_next_obj_dir, wo, hit_obj.normal);
                // 计算点和反射方向的夹角cos
                float cos = std::max(.0f, dotProduct(obj_to_next_obj_dir, hit_obj.normal));
                l_indir = shade(hit_next_obj, -obj_to_next_obj_dir) * f_r * cos / pdf / RussianRoulette;
            }
        }
    }
    // 返回直接光照和间接光照的和
    return l_dir + l_indir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection hit_obj = intersect(ray);
    if (!hit_obj.happened)
        return {};
    return shade(hit_obj, -ray.direction);
}
