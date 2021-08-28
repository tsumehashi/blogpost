cmake_minimum_required(VERSION 2.8)
set(PROJ_NAME main)
project(${PROJ_NAME} CXX C)

set(EIGEN_DIR /usr/local/include/eigen3 CACHE STRING "The directory of the Eigen library") # eigen 3.4
set(UCNOID_DIR ucnoid CACHE STRING "The directory of the ucnoid")
set(PTGL_INC_DIR ptgl CACHE STRING "The directory of the ptgl headers")
set(PTGL_SRC_DIR ptgl CACHE STRING "The directory of the ptgl sources")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto -fexceptions -Wall -DEIGEN_NO_DEBUG -std=c++17")

file(GLOB_RECURSE PTGL_LIB_SRC ${PTGL_SRC_DIR}/ptgl/*.cpp)

# ptgl thirdparty
set(PTGL_THIRDPARTY_SRC
    ${PTGL_SRC_DIR}/thirdparty/Core/nanovg/src/nanovg.c
    ${PTGL_SRC_DIR}/thirdparty/Loader/STL/stl_parser/parse_stl.cpp
)

include_directories(
    ${EIGEN_DIR}
    ${PTGL_INC_DIR}
    ${UCNOID_DIR}/include
)

add_executable(${PROJ_NAME}
    main.cpp
    SceneBodyItem.cpp
    ${PTGL_LIB_SRC}
    ${PTGL_THIRDPARTY_SRC}
)

if (EMSCRIPTEN)
    set_target_properties(${PROJ_NAME} PROPERTIES LINK_FLAGS "--embed-file SR1-2D_ext.body")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    target_link_libraries(${PROJ_NAME}
        stdc++fs
        GL
        GLEW
        glfw
        pthread
    )
endif()
