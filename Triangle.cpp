#include <algorithm>
#include <array>
#include <stdexcept>

#include "Triangle.hpp"

Triangle::Triangle() {
    v[0] << 0, 0, 0, 1;
    v[1] << 0, 0, 0, 1;
    v[2] << 0, 0, 0, 1;

    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int index, Eigen::Vector4f vertex) {
    v[index] = vertex;
}

void Triangle::setNormal(int index, Eigen::Vector3f n) {
    normal[index] = n;
}

void Triangle::setNormals(const std::array<Eigen::Vector3f, 3> &normals) {
    normal[0] = normals[0];
    normal[1] = normals[1];
    normal[2] = normals[2];
}

void Triangle::setColor(int index, float r, float g, float b) {
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) 
        || (b < 0.0) || (b > 255.)) {
        throw std::runtime_error("Invalid color values");
    }

    color[index] = Eigen::Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
}

void Triangle::setColors(const std::array<Eigen::Vector3f, 3> &colors) {
    auto first_color = colors[0];
    setColor(0, colors[0][0], colors[0][1], colors[0][2]);
    setColor(1, colors[1][0], colors[1][1], colors[1][2]);
    setColor(2, colors[2][0], colors[2][1], colors[2][2]);
}

void Triangle::setTexCoord(int index, float s, float t) {
    tex_coords[index] = Eigen::Vector2f(s, t);
}

std::array<Eigen::Vector4f, 3> Triangle::toVector4() const {
    std::array<Eigen::Vector4f, 3> result;
    std::transform(std::begin(v), std::end(v), result.begin(), [](auto &vec) {
        return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
    });
    return result;
}

std::array<Eigen::Vector3f, 3> Triangle::colorToVector() const {
    std::array<Eigen::Vector3f, 3> result;
    std::transform(std::begin(color), std::end(color), result.begin(), [](auto &vec) {
        return Eigen::Vector3f(vec.x(), vec.y(), vec.z());
    });
    return result;
}

std::array<Eigen::Vector2f, 3> Triangle::textureToVector() const {
    std::array<Eigen::Vector2f, 3> result;
    std::transform(std::begin(tex_coords), std::end(tex_coords), result.begin(), [](auto &vec) {
        return Eigen::Vector2f(vec.x(), vec.y());
    });
    return result;
}

std::array<Eigen::Vector3f, 3> Triangle::normalToVector() const {
    std::array<Eigen::Vector3f, 3> result;
    std::transform(std::begin(normal), std::end(normal), result.begin(), [](auto &vec) {
        return Eigen::Vector3f(vec.x(), vec.y(), vec.z());
    });
    return result;
}
