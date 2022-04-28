#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <opencv4/opencv2/opencv.hpp>

#include "global.hpp"
#include "OBJ_Loader.h"
#include "transformation.hpp"
#include "Triangle.hpp"
#include "payload.hpp"
#include "Shader.hpp"
#include "rasterizer.hpp"


int main(int argc, const char **argv) {
    if (DEMO) {
        // initialize rasterizer
        rst::rasterizer r(WIDTH, HEIGHT);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        std::cout << "Rasterizer initialized" << std::endl;

        // set eye/camera
        Eigen::Vector3f eye_position = {EYE_POS_X, EYE_POS_Y, EYE_POS_Z};
        Eigen::Vector3f eye_g = {EYE_G_X, EYE_G_Y, EYE_G_Z};
        Eigen::Vector3f eye_t = {EYE_T_X, EYE_T_Y, EYE_T_Z};

        // set primitives
        std::vector<Eigen::Vector3f> positions{{TRI1_1_X, TRI1_1_Y, TRI1_1_Z}, {TRI1_2_X, TRI1_2_Y, TRI1_2_Z}, {TRI1_3_X, TRI1_3_Y, TRI1_3_Z}, 
                                            {TRI2_1_X, TRI2_1_Y, TRI2_1_Z}, {TRI2_2_X, TRI2_2_Y, TRI2_2_Z}, {TRI2_3_X, TRI2_3_Y, TRI2_3_Z}};
        std::vector<Eigen::Vector3f> colors{{TRI1_1_R, TRI1_1_G, TRI1_1_B}, {TRI1_2_R, TRI1_2_G, TRI1_2_B}, {TRI1_3_R, TRI1_3_G, TRI1_3_B}, 
                                            {TRI2_1_R, TRI2_1_G, TRI2_1_B}, {TRI2_2_R, TRI2_2_G, TRI2_2_B}, {TRI2_3_R, TRI2_3_G, TRI2_3_B}};
        std::vector<Eigen::Vector3i> indices{{0, 1, 2}, {3, 4, 5}};
        auto pos_id = r.load_positions(positions);
        auto col_id = r.load_colors(colors);
        auto ind_id = r.load_indices(indices);
        std::cout << "Set up scene" << std::endl;

        // viewing transformation
        r.set_model(get_model_matrix(MODEL_S_X, MODEL_S_Y, MODEL_S_Z, MODEL_T_X, MODEL_T_Y, MODEL_T_Z, MODEL_R_X, MODEL_R_Y, MODEL_R_Z));
        r.set_view(get_view_matrix(eye_position, eye_g, eye_t));
        r.set_projection(get_projection_matrix(FOV, ASPECT, Z_NEAR, Z_FAR));
        std::cout << "Set up transformations" << std::endl;

        // draw
        r.draw(pos_id, col_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(WIDTH, HEIGHT, CV_32FC3, r.get_frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(FILENAME, image);
        std::cout << "Draw done" << std::endl;
    } else {
        // initialize rasterizer
        rst::rasterizer r(WIDTH, HEIGHT);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_texture(Texture(HAMP_FILE));  // hamp texture by default

        std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;  // phong shader by default
        if (argc == 2) {
            if (std::string(argv[1]) == "normal") {
                std::cout << "Rasterizing using the normal shader" << std::endl;
                active_shader = normal_fragment_shader;
            } else if (std::string(argv[1]) == "phong") {
                std::cout << "Rasterizing using the phong shader" << std::endl;
                active_shader = phong_fragment_shader;
            } else if (std::string(argv[1]) == "texture") {
                std::cout << "Rasterizing using the texture shader" << std::endl;
                active_shader = texture_fragment_shader;
                r.set_texture(Texture(TEXTURE_FILE));
            } else if (std::string(argv[1]) == "bump") {
                std::cout << "Rasterizing using the bump shader" << std::endl;
                active_shader = bump_fragment_shader;
            } else if (std::string(argv[1]) == "displacement") {
                std::cout << "Rasterizing using the displacement shader" << std::endl;
                active_shader = displacement_fragment_shader;
            }
        }
        r.set_vertex_shader(vertex_shader);
        r.set_fragment_shader(active_shader);
        std::cout << "Rasterizer initialized" << std::endl;

        // set eye/camera
        Eigen::Vector3f eye_position = {EYE_POS_X, EYE_POS_Y, EYE_POS_Z};
        Eigen::Vector3f eye_g = {EYE_G_X, EYE_G_Y, EYE_G_Z};
        Eigen::Vector3f eye_t = {EYE_T_X, EYE_T_Y, EYE_T_Z};

        // Load .obj File
        std::vector<Triangle *> TriangleList;
        objl::Loader Loader;
        bool loadout = Loader.LoadFile(OBJ_FILE);
        for (auto mesh:Loader.LoadedMeshes) {
            for (int i = 0; i < mesh.Vertices.size(); i += 3) {
                Triangle *t = new Triangle();
                for (int j = 0; j < 3; ++j) {
                    t->setVertex(j, Eigen::Vector4f(mesh.Vertices[i+j].Position.X, mesh.Vertices[i+j].Position.Y, mesh.Vertices[i+j].Position.Z, 1.0));
                    t->setNormal(j, Eigen::Vector3f(mesh.Vertices[i+j].Normal.X, mesh.Vertices[i+j].Normal.Y, mesh.Vertices[i+j].Normal.Z));
                    t->setTexCoord(j, mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y);
                }
                TriangleList.push_back(t);
            }
        }
        std::cout << "Set up scene" << std::endl;
        
        // viewing transformation
        r.set_model(get_model_matrix(MODEL_S_X, MODEL_S_Y, MODEL_S_Z, MODEL_T_X, MODEL_T_Y, MODEL_T_Z, MODEL_R_X, MODEL_R_Y, MODEL_R_Z));
        r.set_view(get_view_matrix(eye_position, eye_g, eye_t));
        r.set_projection(get_projection_matrix(FOV, ASPECT, Z_NEAR, Z_FAR));
        std::cout << "Set up transformations" << std::endl;

        // draw
        r.draw(TriangleList);
        cv::Mat image(WIDTH, HEIGHT, CV_32FC3, r.get_frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(FILENAME, image);
        std::cout << "Draw done" << std::endl;
    }

    return 0;
}
