#pragma once

#include "Triangle.hpp"
#include "Shader.hpp"

#include <algorithm>
#include <eigen3/Eigen/Eigen>

namespace rst {
//////////---------- Buffers ----------//////////
enum class Buffers {Color = 1, Depth = 2};

inline Buffers operator|(Buffers a, Buffers b) {
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b) {
    return Buffers((int)a & (int)b);
}

//////////---------- Primitives ----------//////////
enum class Primitive {Line, Triangle};

//////////---------- Vertices Buffer ----------//////////
struct pos_buf_id {
    int pos_id = 0;
};

struct ind_buf_id {
    int ind_id = 0;
};

struct col_buf_id {
    int col_id = 0;
};

struct nor_buf_id {
    int norm_id = 0;
};

//////////---------- Rasterizer ----------//////////
class rasterizer {
private:
    int width, height;

    Eigen::Matrix4f model;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

    std::vector<Eigen::Vector3f> frame_buffer;
    std::vector<float> depth_buffer;

    std::optional<Texture> texture;

    int next_id = 0;
    // each id represents a triangle
    std::map<int, std::vector<Eigen::Vector3f>> positions_buffer;
    std::map<int, std::vector<Eigen::Vector3f>> colors_buffer;
    std::map<int, std::vector<Eigen::Vector3i>> indices_buffer;
    std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

    int normal_id = -1;

    std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;
    std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;

public:
    rasterizer(int w, int h);

    pos_buf_id load_positions(const std::vector<Eigen::Vector3f> &positions);
    col_buf_id load_colors(const std::vector<Eigen::Vector3f> &colors);
    nor_buf_id load_normals(const std::vector<Eigen::Vector3f> &normals);
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i> &indices);

    void set_model(const Eigen::Matrix4f &m);
    void set_view(const Eigen::Matrix4f &v);
    void set_projection(const Eigen::Matrix4f &p);

    void set_texture(Texture t) { texture = t; }

    void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
    void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);

    void clear(Buffers buffer);

    void draw(pos_buf_id pos_buffer, col_buf_id col_buffer, ind_buf_id ind_buffer, Primitive type);
    void draw(std::vector<Triangle *> &TriangleList);

    void set_pixel(const Eigen::Vector2i &point, const Eigen::Vector3f &color);

    std::vector<Eigen::Vector3f> &get_frame_buffer() { return frame_buffer; }

private:
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle &t);
    void rasterize_triangle(const Triangle &t);
    void rasterize_triangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &view_pos);
    int get_next_id() { return next_id++; }
    int get_index(int x, int y) const { return (height - y) * width + x; };
    // assume the origin is at left-bottom, while opencv gives left-top
};
} // namespace rst
