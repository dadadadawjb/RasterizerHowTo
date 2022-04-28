#ifndef RASTERIZERHOWTO_TRIANGLE_H
#define RASTERIZERHOWTO_TRIANGLE_H

#include "Texture.hpp"

#include <eigen3/Eigen/Eigen>

class Triangle {
public:
    Eigen::Vector4f v[3];               // the original coordinates of the triangle, v0, v1, v2 in counter clockwise order
    Eigen::Vector3f color[3];           // color
    Eigen::Vector2f tex_coords[3];      // texture u,v
    Eigen::Vector3f normal[3];          // normal vector

    Texture *texture = nullptr;
    Triangle();

    Eigen::Vector4f a() const { return v[0]; }
    Eigen::Vector4f b() const { return v[1]; }
    Eigen::Vector4f c() const { return v[2]; }

    void setVertex(int index, Eigen::Vector4f vertex);
    void setNormal(int index, Eigen::Vector3f n);
    void setNormals(const std::array<Eigen::Vector3f, 3>& normals);
    void setColor(int index, float r, float g, float b);
    void setColors(const std::array<Eigen::Vector3f, 3>& colors);
    void setTexCoord(int index, float s, float t);
    
    std::array<Eigen::Vector4f, 3> toVector4() const;
    std::array<Eigen::Vector3f, 3> colorToVector() const;
    std::array<Eigen::Vector2f, 3> textureToVector() const;
    std::array<Eigen::Vector3f, 3> normalToVector() const;
};

#endif // RASTERIZERHOWTO_TRIANGLE_H
