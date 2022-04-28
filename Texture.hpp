#ifndef RASTERIZERHOWTO_TEXTURE_H
#define RASTERIZERHOWTO_TEXTURE_H

#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

class Texture {
private:
    cv::Mat image_data;

public:
    int width, height;

public:
    Texture(const std::string &name);

    Eigen::Vector3f getColor(float u, float v);

};

#endif // RASTERIZERHOWTO_TEXTURE_H
