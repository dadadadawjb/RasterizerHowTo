#ifndef RASTERIZERHOWTO_SHADER_H
#define RASTERIZERHOWTO_SHADER_H

#include "global.hpp"
#include "payload.hpp"

#include <eigen3/Eigen/Eigen>

static Eigen::Vector3f vertex_shader(const vertex_shader_payload &payload) {
    return payload.position;
}

static Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f ka = Eigen::Vector3f(PHONG_K_A_1, PHONG_K_A_2, PHONG_K_A_3);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(PHONG_K_S_1, PHONG_K_S_2, PHONG_K_S_3);

    auto l1 = light{{LIGHT1_POS_X, LIGHT1_POS_Y, LIGHT1_POS_Z}, {LIGHT1_I_R, LIGHT1_I_G, LIGHT1_I_B}};
    auto l2 = light{{LIGHT2_POS_X, LIGHT2_POS_Y, LIGHT2_POS_Z}, {LIGHT2_I_R, LIGHT2_I_G, LIGHT2_I_B}};
    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{LIGHT_I_R, LIGHT_I_G, LIGHT_I_B};

    Eigen::Vector3f eye_pos{EYE_POS_X, EYE_POS_Y, EYE_POS_Z};
    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_position;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light: lights) {
        // diffuse
        float r = (light.position - point).norm();
        Eigen::Vector3f l = (light.position - point).normalized();
        for (int i = 0; i < 3; ++i)
            result_color[i] += kd[i] * light.intensity[i] / std::pow(r, 2) * std::max(normal.dot(l), 0.f);
        // specular
        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (v + l).normalized();
        for (int i = 0; i < 3; ++i)
            result_color[i] += ks[i] * light.intensity[i] / std::pow(r, 2) * std::pow(std::max(normal.dot(h), 0.f), PHONG_P);
        // ambient
        for (int i = 0; i < 3; ++i)
            result_color[i] += ka[i] * amb_light_intensity[i];
    }

    return result_color * 255.f;
}

static Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
        return_color = payload.texture->getColor(payload.texture_coordinates[0], payload.texture_coordinates[1]);

    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(PHONG_K_A_1, PHONG_K_A_2, PHONG_K_A_3);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(PHONG_K_S_1, PHONG_K_S_2, PHONG_K_S_3);

    auto l1 = light{{LIGHT1_POS_X, LIGHT1_POS_Y, LIGHT1_POS_Z}, {LIGHT1_I_R, LIGHT1_I_G, LIGHT1_I_B}};
    auto l2 = light{{LIGHT2_POS_X, LIGHT2_POS_Y, LIGHT2_POS_Z}, {LIGHT2_I_R, LIGHT2_I_G, LIGHT2_I_B}};
    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{LIGHT_I_R, LIGHT_I_G, LIGHT_I_B};

    Eigen::Vector3f eye_pos{EYE_POS_X, EYE_POS_Y, EYE_POS_Z};
    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_position;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light: lights) {
        // the same as Blinn-Phong
        // diffuse
        float r = (light.position - point).norm();
        Eigen::Vector3f l = (light.position - point).normalized();
        for (int i = 0; i < 3; ++i)
            result_color[i] += kd[i] * light.intensity[i] / std::pow(r, 2) * std::max(normal.dot(l), 0.f);
        // specular
        Eigen::Vector3f v = (eye_pos - point).normalized();
        Eigen::Vector3f h = (v + l).normalized();
        for (int i = 0; i < 3; ++i)
            result_color[i] += ks[i] * light.intensity[i] / std::pow(r, 2) * std::pow(std::max(normal.dot(h), 0.f), PHONG_P);
        // ambient
        for (int i = 0; i < 3; ++i)
            result_color[i] += ka[i] * amb_light_intensity[i];
    }

    return result_color * 255.f;
}

static Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f ka = Eigen::Vector3f(PHONG_K_A_1, PHONG_K_A_2, PHONG_K_A_3);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(PHONG_K_S_1, PHONG_K_S_2, PHONG_K_S_3);

    auto l1 = light{{LIGHT1_POS_X, LIGHT1_POS_Y, LIGHT1_POS_Z}, {LIGHT1_I_R, LIGHT1_I_G, LIGHT1_I_B}};
    auto l2 = light{{LIGHT2_POS_X, LIGHT2_POS_Y, LIGHT2_POS_Z}, {LIGHT2_I_R, LIGHT2_I_G, LIGHT2_I_B}};
    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{LIGHT_I_R, LIGHT_I_G, LIGHT_I_B};

    Eigen::Vector3f eye_pos{EYE_POS_X, EYE_POS_Y, EYE_POS_Z};
    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_position;
    Eigen::Vector3f normal = payload.normal;

    // not understand yet
    float kh = 0.2, kn = 0.1;

    float x = normal[0], y = normal[1], z = normal[2];
    Eigen::Vector3f t = {x * y / sqrt(x*x + z*z), sqrt(x*x + z*z), z * y / sqrt(x*x + z*z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t[0], b[0], normal[0], t[1], b[1], normal[1], t[2], b[2], normal[2];
    float h, hw, hh;
    h = payload.texture->getColor(payload.texture_coordinates[0], payload.texture_coordinates[1]).norm();
    hw = payload.texture->getColor(payload.texture_coordinates[0] + 1.0 / payload.texture->width, payload.texture_coordinates[1]).norm();
    hh = payload.texture->getColor(payload.texture_coordinates[0], payload.texture_coordinates[1] + 1.0 / payload.texture->height).norm();
    
    float dU = kh * kn * (hw - h);
    float dV = kh * kn * (hh - h);
    Eigen::Vector3f ln = {-dU, -dV, 1};
    Eigen::Vector3f n = (TBN * ln).normalized();

    Eigen::Vector3f result_color = n;
    return result_color * 255.f;
}

static Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f ka = Eigen::Vector3f(PHONG_K_A_1, PHONG_K_A_2, PHONG_K_A_3);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(PHONG_K_S_1, PHONG_K_S_2, PHONG_K_S_3);

    auto l1 = light{{LIGHT1_POS_X, LIGHT1_POS_Y, LIGHT1_POS_Z}, {LIGHT1_I_R, LIGHT1_I_G, LIGHT1_I_B}};
    auto l2 = light{{LIGHT2_POS_X, LIGHT2_POS_Y, LIGHT2_POS_Z}, {LIGHT2_I_R, LIGHT2_I_G, LIGHT2_I_B}};
    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{LIGHT_I_R, LIGHT_I_G, LIGHT_I_B};

    Eigen::Vector3f eye_pos{EYE_POS_X, EYE_POS_Y, EYE_POS_Z};
    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_position;
    Eigen::Vector3f normal = payload.normal;

    // not understand yet
    float kh = 0.2, kn = 0.1;

    float x = normal[0], y = normal[1], z = normal[2];
    Eigen::Vector3f t = {x * y / sqrt(x*x + z*z), sqrt(x*x + z*z), z * y / sqrt(x*x + z*z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t[0], b[0], normal[0], t[1], b[1], normal[1], t[2], b[2], normal[2];
    float h, hw, hh;
    h = payload.texture->getColor(payload.texture_coordinates[0], payload.texture_coordinates[1]).norm();
    hw = payload.texture->getColor(payload.texture_coordinates[0] + 1.0 / payload.texture->width, payload.texture_coordinates[1]).norm();
    hh = payload.texture->getColor(payload.texture_coordinates[0], payload.texture_coordinates[1] + 1.0 / payload.texture->height).norm();

    float dU = kh * kn * (hw - h);
    float dV = kh * kn * (hh - h);
    Eigen::Vector3f ln = {-dU, -dV, 1};
    Eigen::Vector3f n = (TBN * ln).normalized();
    
    Eigen::Vector3f position = point + kn * h * normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light: lights) {
        // position and normal change
        // diffuse
        float r = (light.position - position).norm();
        Eigen::Vector3f l = (light.position - position).normalized();
        for (int i = 0; i < 3; ++i)
            result_color[i] += kd[i] * light.intensity[i] / std::pow(r, 2) * std::max(n.dot(l), 0.f);
        // specular
        Eigen::Vector3f v = (eye_pos - position).normalized();
        Eigen::Vector3f h = (v + l).normalized();
        for (int i = 0; i < 3; ++i)
            result_color[i] += ks[i] * light.intensity[i] / std::pow(r, 2) * std::pow(std::max(n.dot(h), 0.f), PHONG_P);
        // ambient
        for (int i = 0; i < 3; ++i)
            result_color[i] += ka[i] * amb_light_intensity[i];
    }

    return result_color * 255.f;
}

#endif // RASTERIZERHOWTO_SHADER_H
