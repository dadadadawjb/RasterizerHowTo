#include "global.hpp"
#include "Triangle.hpp"

Texture::Texture(const std::string &name) {
    image_data = cv::imread(name);
    cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
    width = image_data.cols;
    height = image_data.rows;
}

Eigen::Vector3f Texture::getColor(float u, float v) {
    if (!BILINEAR) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    } else {
        u = std::min(1.f, std::max(0.f, u));
        v = std::min(1.f, std::max(0.f, v));
        float u_img = u * width;
        float v_img = (1 - v) * height;
        int u_img_l = std::max(std::floor(u_img), 0.f);
        int u_img_r = std::min(u_img_l + 1, width);
        int v_img_t = std::max(std::floor(v_img), 0.f);
        int v_img_d = std::min(v_img_t + 1, height);
        float s = u_img - u_img_l;
        float t = v_img_d - v_img;
        auto color01 = image_data.at<cv::Vec3b>(v_img_t, u_img_l);
        auto color11 = image_data.at<cv::Vec3b>(v_img_t, u_img_r);
        auto color00 = image_data.at<cv::Vec3b>(v_img_d, u_img_l);
        auto color10 = image_data.at<cv::Vec3b>(v_img_d, u_img_r);
        auto color_temp1 = color00 + s * (color10 - color00);
        auto color_temp2 = color01 + s * (color11 - color01);
        auto color = color_temp1 + t * (color_temp2 - color_temp1);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
}
