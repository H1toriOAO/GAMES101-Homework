//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f lDir, lIndir;

    Intersection interFirst = intersect(ray);

    // 如果射线未与场景中的物体发生碰撞
    if (!interFirst.happened) {
        return Vector3f();
    }

    // 如果射线刚好与光源碰撞
    if (interFirst.m->hasEmission()) {
        return interFirst.m->getEmission();
    }

    // 通过sampleLight，对光源进行采样
    // 返回了采样点和pdf的信息
    Intersection lightInter;
    float pdf;
    sampleLight(lightInter, pdf);
    Vector3f &lightPos = lightInter.coords;

    float dis = (lightPos - interFirst.coords).norm();
    Vector3f dir = (lightPos - interFirst.coords).normalized();

    // 通过新建的射线，检测该光线是否被遮挡
    Intersection interTest = intersect(Ray(interFirst.coords, dir));
    // 如果没有发生遮挡
    if (interTest.distance - dis > -0.00005f) {
        lDir = lightInter.emit
                * interFirst.m->eval(ray.direction, dir, interFirst.normal)
                * dotProduct(dir, interFirst.normal) * dotProduct(-dir, lightInter.normal)
                / (dis * dis * pdf);
    }

    // 通过 RR 判断是否进行非直接光照的采样
    if (get_random_float() < RussianRoulette) {
        Vector3f wi = interFirst.m->sample(ray.direction, interFirst.normal).normalized();
        Ray testIndir(interFirst.coords, wi);
        Intersection testIndirInter = intersect(testIndir);

        if (testIndirInter.happened && !testIndirInter.m->hasEmission()) {
            lIndir = castRay(testIndir, depth + 1)
                    * interFirst.m->eval(ray.direction, wi, interFirst.normal)
                    * dotProduct(wi, interFirst.normal)
                    / interFirst.m->pdf(ray.direction, wi, interFirst.normal)
                    / RussianRoulette;
        }
    }

    return lDir + lIndir;
}