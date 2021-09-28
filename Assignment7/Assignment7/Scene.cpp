//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "global.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    float totalArea=emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                pdf=1/emit_area_sum;
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Intersection shadingPoint = intersect(ray);
    if (!shadingPoint.happened) {
        return { 0, 0, 0 };
    }
    if (depth == 0 && shadingPoint.m->hasEmission()) {
        return shadingPoint.m->getEmission();
    }
    //直接光照部分
    Intersection lightPoint;
    float pdf;
    sampleLight(lightPoint, pdf); //随机选光源，得到该光源PDF
    Vector3f directLight(0.0, 0.0, 0.0);
    Vector3f wi = -ray.direction;
    Vector3f wo = (lightPoint.coords - shadingPoint.coords).normalized();
    Vector3f normal = shadingPoint.normal;

    Ray testRay(shadingPoint.coords, wo, 0.0);
    auto testIntersect = intersect(testRay);
    if (testIntersect.happened && testIntersect.m->hasEmission()) {
        float distance = testIntersect.distance;
        Vector3f fr = shadingPoint.m->eval(wi, wo, normal);
        directLight
            = lightPoint.emit * fr
            * dotProduct(wo, normal) * dotProduct(-wo, lightPoint.normal)
            / (distance * distance * pdf);
            //std::cout << directLight << '\n';
    }

    //间接光照部分
    Vector3f indirectLight(0.0, 0.0, 0.0);

    if (get_random_float() < RussianRoulette) {
        wo = shadingPoint.m->sample(wi, normal);
        pdf = shadingPoint.m->pdf(wi, wo, normal);
        Vector3f fr = shadingPoint.m->eval(wi, wo, normal);
        Ray newRay(shadingPoint.coords, wo, 0.0);
        indirectLight = castRay(newRay, depth + 1) * fr * dotProduct(wo, normal) / pdf / RussianRoulette;
    }
    return directLight + indirectLight;
}