#ifndef RASTERIZERHOWTO_TRANSFORMATIOn_H
#define RASTERIZERHOWTO_TRANSFORMATIOn_H

#include <eigen3/Eigen/Eigen>

#include "global.hpp"

Eigen::Matrix4f get_model_matrix(float s_x, float s_y, float s_z, float t_x, float t_y, float t_z, float r_x, float r_y, float r_z) {
    // first scale, second translate, third rotate, where rotate with static first x second y third z
    Eigen::Matrix4f scale;
    scale << s_x, 0, 0, 0, 
             0, s_y, 0, 0, 
             0, 0, s_z, 0, 
             0, 0, 0, 1;
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, t_x, 
                 0, 1, 0, t_y, 
                 0, 0, 1, t_z, 
                 0, 0, 0, 1;
    Eigen::Matrix4f rotate;
    Eigen::Matrix4f rotate_x, rotate_y, rotate_z;
    rotate_x << 1, 0, 0, 0, 
                0, std::cos(r_x / 180.0 * MY_PI), (-1) * std::sin(r_x / 180.0 * MY_PI), 0, 
                0, std::sin(r_x / 180.0 * MY_PI), std::cos(r_x / 180.0 * MY_PI), 0, 
                0, 0, 0, 1;
    rotate_y << std::cos(r_y / 180.0 * MY_PI), 0, std::sin(r_y / 180.0 * MY_PI), 0, 
                0, 1, 0, 0, 
                (-1) * std::sin(r_y / 180.0 * MY_PI), 0, std::cos(r_y / 180.0 * MY_PI), 0, 
                0, 0, 0, 1;
    rotate_z << std::cos(r_z / 180.0 * MY_PI), (-1) * std::sin(r_z / 180.0 * MY_PI), 0, 0, 
                std::sin(r_z / 180.0 * MY_PI), std::cos(r_z / 180.0 * MY_PI), 0, 0, 
                0, 0, 1, 0, 
                0, 0, 0, 1;
    rotate = rotate_z * rotate_y * rotate_x;

    Eigen::Matrix4f model = rotate * translate * scale;
    return model;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_position, Eigen::Vector3f eye_g, Eigen::Vector3f eye_t) {
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_position[0], 
                 0, 1, 0, -eye_position[1], 
                 0, 0, 1, -eye_position[2], 
                 0, 0, 0, 1;
    Eigen::Matrix4f rotate;
    Eigen::Vector3f eye_x = eye_g.cross(eye_t);
    rotate << eye_x[0], eye_x[1], eye_x[2], 0, 
              eye_t[0], eye_t[1], eye_t[2], 0, 
              (-1) * eye_g[0], (-1) * eye_g[1], (-1) * eye_g[2], 0, 
              0, 0, 0, 1;

    Eigen::Matrix4f view = rotate * translate;
    return view;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    // zNear and zFar as positive, l = -r, b = -t
    float n = (-1) * zNear;
    float f = (-1) * zFar;
    float t = abs(n) * std::tan(eye_fov / 2.0 / 180.0 * MY_PI);
    float b = (-1) * t;
    float r = t * aspect_ratio;
    float l = (-1) * r;
    Eigen::Matrix4f persp2ortho;
    persp2ortho << n, 0, 0, 0, 
                   0, n, 0, 0, 
                   0, 0, n + f, (-1) * n * f, 
                   0, 0, 1, 0;
    Eigen::Matrix4f translate, scale, ortho;
    scale << 2.0 / (r - l), 0, 0, 0, 
             0, 2.0 / (t - b), 0, 0, 
             0, 0, 2.0 / (n - f), 0, 
             0, 0, 0, 1;
    translate << 1, 0, 0, (-1) * (r + l) / 2.0, 
                 0, 1, 0, (-1) * (t + b) / 2.0, 
                 0, 0, 1, (-1) * (n + f) / 2.0, 
                 0, 0, 0, 1;
    ortho = scale * translate;

    Eigen::Matrix4f projection = ortho * persp2ortho;
    return projection;
}

Eigen::Matrix4f get_rotation_matrix(Eigen::Vector3f axis, float angle) {
    Eigen::Matrix3f N;
    N << 0, (-1) * axis[2], axis[1], axis[2], 0, (-1) * axis[0], (-1) * axis[1], axis[0], 0;
    Eigen::Matrix3f rotation_3 = std::cos(angle * 180.0 / MY_PI) * Eigen::Matrix3f::Identity() + 
                                 (1 - std::cos(angle * 180.0 / MY_PI)) * axis * axis.transpose() + 
                                 std::sin(angle * 180.0 / MY_PI) * N;
    
    Eigen::Matrix4f rotation;
    rotation << rotation_3(0, 0), rotation_3(0, 1), rotation_3(0, 2), 0, 
                rotation_3(1, 0), rotation_3(1, 1), rotation_3(1, 2), 0, 
                rotation_3(2, 0), rotation_3(2, 1), rotation_3(2, 2), 0, 
                0, 0, 0, 1;
    return rotation;
}

#endif // RASTERIZERHOWTO_TRANSFORMATIOn_H
