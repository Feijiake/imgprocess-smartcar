cmake_minimum_required(VERSION 3.27)
project(chitu_clion_19_46)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

# 定义源文件和头文件列表
set(SOURCES
        main.cpp
)

set(HEADERS
        img_processing/img_process_app.h
)

# 添加可执行文件
add_executable(chitu_clion_19_46 ${SOURCES} ${HEADERS})

# 添加头文件目录
target_include_directories(chitu_clion_19_46 PRIVATE
        ${PROJECT_SOURCE_DIR}/MCU_TOOL
        ${PROJECT_SOURCE_DIR}/img_processing
)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    target_include_directories(chitu_clion_19_46 PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(chitu_clion_19_46 PRIVATE ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "OpenCV library not found")
endif()
