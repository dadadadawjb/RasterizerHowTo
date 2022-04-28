#ifndef RASTERIZERHOWTO_GLOBAL_H
#define RASTERIZERHOWTO_GLOBAL_H

#define FILENAME "output.png"
#define OBJ_FILE "../models/spot/spot_triangulated_good.obj"
#define HAMP_FILE "../models/spot/hmap.jpg"
#define TEXTURE_FILE "../models/spot/spot_texture.png"
constexpr bool DEMO = false;
constexpr bool OUTSIDE = false;
constexpr bool INSIDE = true;
constexpr bool MSAA = true;
constexpr bool BILINEAR = true;
#define WIDTH 700
#define HEIGHT 700
#define FOV 45
#define ASPECT 1
#define Z_NEAR 0.1
#define Z_FAR 50
#define EYE_POS_X 0
#define EYE_POS_Y 0
#define EYE_POS_Z 5
#define EYE_G_X 0
#define EYE_G_Y 0
#define EYE_G_Z -1
#define EYE_T_X 0
#define EYE_T_Y 1
#define EYE_T_Z 0
#define TRI1_1_X 2
#define TRI1_1_Y 0
#define TRI1_1_Z -2
#define TRI1_2_X 0
#define TRI1_2_Y 2
#define TRI1_2_Z -2
#define TRI1_3_X -2
#define TRI1_3_Y 0
#define TRI1_3_Z -2
#define TRI2_1_X 3.5
#define TRI2_1_Y -1
#define TRI2_1_Z -5
#define TRI2_2_X 2.5
#define TRI2_2_Y 1.5
#define TRI2_2_Z -5
#define TRI2_3_X -1
#define TRI2_3_Y 0.5
#define TRI2_3_Z -5
#define TRI1_1_R 217.0
#define TRI1_1_G 238.0
#define TRI1_1_B 185.0
#define TRI1_2_R 217.0
#define TRI1_2_G 238.0
#define TRI1_2_B 185.0
#define TRI1_3_R 217.0
#define TRI1_3_G 238.0
#define TRI1_3_B 185.0
#define TRI2_1_R 185.0
#define TRI2_1_G 217.0
#define TRI2_1_B 238.0
#define TRI2_2_R 185.0
#define TRI2_2_G 217.0
#define TRI2_2_B 238.0
#define TRI2_3_R 185.0
#define TRI2_3_G 217.0
#define TRI2_3_B 238.0
#define MODEL_S_X 1
#define MODEL_S_Y 1
#define MODEL_S_Z 1
#define MODEL_T_X 0
#define MODEL_T_Y 0
#define MODEL_T_Z 0
#define MODEL_R_X 0
// #define MODEL_R_Y 0
#define MODEL_R_Y 140
#define MODEL_R_Z 0
#define LINES_R 255.0
#define LINES_G 255.0
#define LINES_B 255.0
#define TRIS_R 148.0
#define TRIS_G 121.0
#define TRIS_B 92.0
#define PHONG_K_A_1 0.005
#define PHONG_K_A_2 0.005
#define PHONG_K_A_3 0.005
#define PHONG_K_S_1 0.7937
#define PHONG_K_S_2 0.7937
#define PHONG_K_S_3 0.7937
#define PHONG_P 150
#define LIGHT1_POS_X 20
#define LIGHT1_POS_Y 20
#define LIGHT1_POS_Z 20
#define LIGHT2_POS_X -20
#define LIGHT2_POS_Y 20
#define LIGHT2_POS_Z 0
#define LIGHT1_I_R 500
#define LIGHT1_I_G 500
#define LIGHT1_I_B 500
#define LIGHT2_I_R 500
#define LIGHT2_I_G 500
#define LIGHT2_I_B 500
#define LIGHT_I_R 10
#define LIGHT_I_G 10
#define LIGHT_I_B 10

constexpr double MY_PI = 3.1415926535;

#endif // RASTERIZERHOWTO_GLOBAL_H
