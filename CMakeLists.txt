cmake_minimum_required(VERSION 3.10)
project(RasterizerHowTo)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(RasterizerHowTo main.cpp global.hpp OBJ_Loader.h payload.hpp Texture.hpp Texture.cpp Shader.hpp Triangle.hpp Triangle.cpp transformation.hpp rasterizer.hpp rasterizer.cpp)
target_link_libraries(RasterizerHowTo ${OpenCV_LIBRARIES})
