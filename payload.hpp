#ifndef RASTERIZERHOWTO_PAYLOAD_H
#define RASTERIZERHOWTO_PAYLOAD_H

#include "Texture.hpp"

#include <eigen3/Eigen/Eigen>

struct light {
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

struct fragment_shader_payload {
    Eigen::Vector3f view_position;
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f texture_coordinates;
    Texture *texture;

    fragment_shader_payload() {
        texture = nullptr;
    }
    fragment_shader_payload(const Eigen::Vector3f &col, const Eigen::Vector3f &nor,const Eigen::Vector2f &tc, Texture *tex) :
         color(col), normal(nor), texture_coordinates(tc), texture(tex) {}
};

struct vertex_shader_payload {
    Eigen::Vector3f position;
};

#endif // RASTERIZERHOWTO_PAYLOAD_H
