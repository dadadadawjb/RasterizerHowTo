#include <algorithm>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

#include "global.hpp"
#include "rasterizer.hpp"

Eigen::Vector4f to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Eigen::Vector3f *_v) {
    Eigen::Vector3f flag01, flag12, flag20;
    Eigen::Vector3f v01 = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0};
    Eigen::Vector3f vtest0 = {x - _v[0].x(), y - _v[0].y(), 0};
    flag01 = v01.cross(vtest0);
    Eigen::Vector3f v12 = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0};
    Eigen::Vector3f vtest1 = {x - _v[1].x(), y - _v[1].y(), 0};
    flag12 = v12.cross(vtest1);
    Eigen::Vector3f v20 = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0};
    Eigen::Vector3f vtest2 = {x - _v[2].x(), y - _v[2].y(), 0};
    flag20 = v20.cross(vtest2);

    // allow edge case
    if (flag01.dot(flag12) >= 0 && flag12.dot(flag20) >= 0 && flag20.dot(flag01) >= 0)
        return true;
    else
        return false;
}

static bool insideTriangle(int x, int y, const Eigen::Vector4f *_v){
    // use int not float for pixel center, not understand yet
    Eigen::Vector3f v[3];
    for (int i = 0; i < 3; ++i)
        v[i] = {_v[i].x(), _v[i].y(), 1.0};
    Eigen::Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Eigen::Vector3f p(x, y, 1.0);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
        return true;
    else
        return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Eigen::Vector4f *v) {
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1, c2, c3};
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f &vert1, const Eigen::Vector3f &vert2, const Eigen::Vector3f &vert3, float weight) {
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f &vert1, const Eigen::Vector2f &vert2, const Eigen::Vector2f &vert3, float weight) {
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

rst::rasterizer::rasterizer(int w, int h): width(w), height(h) {
    frame_buffer.resize(w * h);
    depth_buffer.resize(w * h);

    texture = std::nullopt;
}

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    int id = get_next_id();
    positions_buffer.emplace(id, positions);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &colors) {
    int id = get_next_id();
    colors_buffer.emplace(id, colors);

    return {id};
}

rst::nor_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals) {
    int id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    int id = get_next_id();
    indices_buffer.emplace(id, indices);

    return {id};
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader) {
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader) {
    fragment_shader = frag_shader;
}


void rst::rasterizer::clear(rst::Buffers buffer) {
    if ((buffer &rst::Buffers::Color) == rst::Buffers::Color)
        std::fill(frame_buffer.begin(), frame_buffer.end(), Eigen::Vector3f{0, 0, 0});
    
    if ((buffer &rst::Buffers::Depth) == rst::Buffers::Depth)
        std::fill(depth_buffer.begin(), depth_buffer.end(), std::numeric_limits<float>::infinity());
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::col_buf_id col_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type) {
    if (type != rst::Primitive::Triangle)
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");

    auto &buf = positions_buffer[pos_buffer.pos_id];
    auto &ind = indices_buffer[ind_buffer.ind_id];
    auto &col = colors_buffer[col_buffer.col_id];

    // not understand yet
    float f1 = (Z_FAR - Z_NEAR) / 2.0;
    float f2 = (Z_FAR + Z_NEAR) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i: ind) {
        Triangle t;

        // do MVP transformation
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        for (auto &vec: v)
            vec /= vec.w();

        // do viewport transformation
        for (auto &vert: v) {
            vert.x() = 0.5 * width * vert.x() + 0.5 * width;
            vert.y() = 0.5 * height * vert.y() + 0.5 * height;
            vert.z() = (-1) * vert.z() * f1 + f2;
        }

        for (int k = 0; k < 3; ++k) {
            t.setVertex(k, v[k]);
            auto col_k = col[i[k]];
            t.setColor(k, col_k[0], col_k[1], col_k[2]);
        }

        if (OUTSIDE)
            rasterize_wireframe(t);
        if (INSIDE)
            rasterize_triangle(t);
    }
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {
    // not understand yet
    float f1 = (Z_FAR - Z_NEAR) / 2.0;
    float f2 = (Z_FAR + Z_NEAR) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto &t: TriangleList) {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };

        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        for (auto &vert: v) {
            vert.x() = 0.5 * width * vert.x() + 0.5 * width;
            vert.y() = 0.5 * height * vert.y() + 0.5 * height;
            vert.z() = (-1) * vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            // screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i) {
            // view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, TRIS_R, TRIS_G, TRIS_B);
        newtri.setColor(1, TRIS_R, TRIS_G, TRIS_B);
        newtri.setColor(2, TRIS_R, TRIS_G, TRIS_B);

        if (OUTSIDE)
            rasterize_wireframe(newtri);
        else
            rasterize_triangle(newtri, viewspace_pos);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t) {
    draw_line(t.c().head<3>(), t.a().head<3>());
    draw_line(t.c().head<3>(), t.b().head<3>());
    draw_line(t.b().head<3>(), t.a().head<3>());
}

void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end) {
    // Bresenham's line drawing algorithm
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {LINES_R, LINES_G, LINES_B};

    int x, y, xe, ye;

    int dx = x2 - x1, dy = y2 - y1;
    int dx1 = std::fabs(dx), dy1 = std::fabs(dy);
    int px = 2 * dy1 - dx1, py = 2 * dx1 - dy1;

    if (dy1 <= dx1) {
        if (dx >= 0) {
            x = x1;
            y = y1;
            xe = x2;
        } else {
            x = x2;
            y = y2;
            xe = x1;
        }

        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (int i = 0; x < xe; ++i) {
            ++x;
            if (px < 0) {
                px = px + 2 * dy1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                    ++y;
                else
                    --y;
                
                px = px + 2 * (dy1 - dx1);
            }
            // delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    } else {
        if (dy >= 0) {
            x = x1;
            y = y1;
            ye = y2;
        } else {
            x = x2;
            y = y2;
            ye = y1;
        }

        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (int i = 0; y < ye; ++i) {
            ++y;
            if (py <= 0) {
                py = py + 2 * dx1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                    ++x;
                else
                    --x;
                
                py = py + 2 * (dx1 - dy1);
            }
            // delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
}

void rst::rasterizer::rasterize_triangle(const Triangle &t) {
    auto v = t.toVector4();
    Eigen::Vector3f _v3[3] = {{v.at(0).x(), v.at(0).y(), v.at(0).z()}, 
                              {v.at(1).x(), v.at(1).y(), v.at(1).z()}, 
                              {v.at(2).x(), v.at(2).y(), v.at(2).z()}};
    
    float xmin = std::min(std::min(v.at(0).x(), v.at(1).x()), v.at(2).x());
    float xmax = std::max(std::max(v.at(0).x(), v.at(1).x()), v.at(2).x());
    float ymin = std::min(std::min(v.at(0).y(), v.at(1).y()), v.at(2).y());
    float ymax = std::max(std::max(v.at(0).y(), v.at(1).y()), v.at(2).y());

    if (!MSAA) {
        int bbox_x_min = std::ceil(xmin);
        int bbox_x_max = std::floor(xmax);
        int bbox_y_min = std::ceil(ymin);
        int bbox_y_max = std::floor(ymax);

        for (int x = bbox_x_min; x <= bbox_x_max; ++x) {
            for (int y = bbox_y_min; y <= bbox_y_max; ++y) {
                if (insideTriangle(x + 0.5, y + 0.5, _v3)) {
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // Z-buffering
                    int index = get_index(x, y);
                    if (z_interpolated < depth_buffer[index]) {
                        auto colors = t.colorToVector();
                        Eigen::Vector3f color_interpolated = interpolate(alpha, beta, gamma, colors[0], colors[1], colors[2], 1.0);
                        set_pixel({x, y}, 255.0 * color_interpolated);
                        depth_buffer[index] = z_interpolated;
                    }
                }
            }
        }
    } else {
        int bbox_x_min = std::floor(xmin);
        int bbox_x_max = std::ceil(xmax);
        int bbox_y_min = std::floor(ymin);
        int bbox_y_max = std::ceil(ymax);

        for (int x = bbox_x_min; x <= bbox_x_max; ++x) {
            for (int y = bbox_y_min; y <= bbox_y_max; ++y) {
                bool flag1 = insideTriangle(x + 0.25, y + 0.25, _v3);
                bool flag2 = insideTriangle(x + 0.75, y + 0.25, _v3);
                bool flag3 = insideTriangle(x + 0.25, y + 0.75, _v3);
                bool flag4 = insideTriangle(x + 0.75, y + 0.75, _v3);
                if (flag1 || flag2 || flag3 || flag4) {
                    float depth = 0;
                    bool flag = false;
                    Eigen::Vector3f color_interpolated = {0, 0, 0};
                    if (flag1) {
                        float alpha1, beta1, gamma1;
                        std::tie(alpha1, beta1, gamma1) = computeBarycentric2D(x + 0.25, y + 0.25, t.v);
                        float w_reciprocal1 = 1.0/(alpha1 / v[0].w() + beta1 / v[1].w() + gamma1 / v[2].w());
                        float z_interpolated1 = alpha1 * v[0].z() / v[0].w() + beta1 * v[1].z() / v[1].w() + gamma1 * v[2].z() / v[2].w();
                        z_interpolated1 *= w_reciprocal1;
                        if (flag) {
                            depth = std::min(z_interpolated1, depth);
                        } else {
                            depth = z_interpolated1;
                            flag = true;
                        }
                        auto colors1 = t.colorToVector();
                        color_interpolated += interpolate(alpha1, beta1, gamma1, colors1[0], colors1[1], colors1[2], 1);
                    }
                    if (flag2) {
                        float alpha2, beta2, gamma2;
                        std::tie(alpha2, beta2, gamma2) = computeBarycentric2D(x + 0.75, y + 0.25, t.v);
                        float w_reciprocal2 = 1.0/(alpha2 / v[0].w() + beta2 / v[1].w() + gamma2 / v[2].w());
                        float z_interpolated2 = alpha2 * v[0].z() / v[0].w() + beta2 * v[1].z() / v[1].w() + gamma2 * v[2].z() / v[2].w();
                        z_interpolated2 *= w_reciprocal2;
                        if (flag) {
                            depth = std::min(z_interpolated2, depth);
                        } else {
                            depth = z_interpolated2;
                            flag = true;
                        }
                        auto colors2 = t.colorToVector();
                        color_interpolated += interpolate(alpha2, beta2, gamma2, colors2[0], colors2[1], colors2[2], 1);
                    }
                    if (flag3) {
                        float alpha3, beta3, gamma3;
                        std::tie(alpha3, beta3, gamma3) = computeBarycentric2D(x + 0.25, y + 0.75, t.v);
                        float w_reciprocal3 = 1.0/(alpha3 / v[0].w() + beta3 / v[1].w() + gamma3 / v[2].w());
                        float z_interpolated3 = alpha3 * v[0].z() / v[0].w() + beta3 * v[1].z() / v[1].w() + gamma3 * v[2].z() / v[2].w();
                        z_interpolated3 *= w_reciprocal3;
                        if (flag) {
                            depth = std::min(z_interpolated3, depth);
                        } else {
                            depth = z_interpolated3;
                            flag = true;
                        }
                        auto colors3 = t.colorToVector();
                        color_interpolated += interpolate(alpha3, beta3, gamma3, colors3[0], colors3[1], colors3[2], 1);
                    }
                    if (flag4) {
                        float alpha4, beta4, gamma4;
                        std::tie(alpha4, beta4, gamma4) = computeBarycentric2D(x + 0.75, y + 0.75, t.v);
                        float w_reciprocal4 = 1.0/(alpha4 / v[0].w() + beta4 / v[1].w() + gamma4 / v[2].w());
                        float z_interpolated4 = alpha4 * v[0].z() / v[0].w() + beta4 * v[1].z() / v[1].w() + gamma4 * v[2].z() / v[2].w();
                        z_interpolated4 *= w_reciprocal4;
                        if (flag) {
                            depth = std::min(z_interpolated4, depth);
                        } else {
                            depth = z_interpolated4;
                            flag = true;
                        }
                        auto colors4 = t.colorToVector();
                        color_interpolated += interpolate(alpha4, beta4, gamma4, colors4[0], colors4[1], colors4[2], 1);
                    }

                    // Z-buffering
                    int index = get_index(x, y);
                    if (depth < depth_buffer[index]) {
                        set_pixel({x, y}, 255.0 * color_interpolated / 4.0);
                        depth_buffer[index] = depth;
                    }
                }
            }
        }
    }
}

void rst::rasterizer::rasterize_triangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &view_pos) {
    // more standard than the function above
    auto v = t.toVector4();
    Eigen::Vector4f _v4[3] = {{v.at(0).x(), v.at(0).y(), v.at(0).z(), v.at(0).w()}, 
                              {v.at(1).x(), v.at(1).y(), v.at(1).z(), v.at(1).w()}, 
                              {v.at(2).x(), v.at(2).y(), v.at(2).z(), v.at(2).w()}};
    
    float xmin = std::min(std::min(v.at(0).x(), v.at(1).x()), v.at(2).x());
    float xmax = std::max(std::max(v.at(0).x(), v.at(1).x()), v.at(2).x());
    float ymin = std::min(std::min(v.at(0).y(), v.at(1).y()), v.at(2).y());
    float ymax = std::max(std::max(v.at(0).y(), v.at(1).y()), v.at(2).y());

    int bbox_x_min = std::ceil(xmin);
    int bbox_x_max = std::floor(xmax);
    int bbox_y_min = std::ceil(ymin);
    int bbox_y_max = std::floor(ymax);

    // not implement MSAA
    for (int x = bbox_x_min; x <= bbox_x_max; ++x) {
        for (int y = bbox_y_min; y <= bbox_y_max; ++y) {
            if (insideTriangle(x + 0.5, y + 0.5, _v4)) {
                float alpha, beta, gamma;
                std::tie(alpha, beta, gamma) = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // Z-buffering
                int index = get_index(x, y);
                if (z_interpolated < depth_buffer[index]) {
                    auto colors = t.colorToVector();
                    auto normals = t.normalToVector();
                    auto textures = t.textureToVector();
                    Eigen::Vector3f color_interpolated = interpolate(alpha, beta, gamma, colors[0], colors[1], colors[2], 1);
                    Eigen::Vector3f normal_interpolated = interpolate(alpha, beta, gamma, normals[0], normals[1], normals[2], 1).normalized();
                    Eigen::Vector2f texture_interpolated = interpolate(alpha, beta, gamma, textures[0], textures[1], textures[2], 1);

                    auto shadingcoordinates_interpolated = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);

                    fragment_shader_payload payload(color_interpolated, normal_interpolated, texture_interpolated, texture ? &*texture : nullptr);
                    payload.view_position = shadingcoordinates_interpolated;
                    auto pixel_color = fragment_shader(payload);
                    set_pixel({x, y}, pixel_color);
                    depth_buffer[index] = z_interpolated;
                }
            }
        }
    }
}

void rst::rasterizer::set_pixel(const Eigen::Vector2i& point, const Eigen::Vector3f& color) {
    if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
        return;
    
    int index = get_index(point.x(), point.y());
    frame_buffer[index] = color;
}
