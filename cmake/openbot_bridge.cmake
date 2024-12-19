cmake_minimum_required(VERSION 3.11)

include(ExternalProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_custom_target(openbot_bridge)
set(install_dir /usr/local)
set(openbot_bridge_INCLUDE_DIR ${install_dir}/include)
set(openbot_bridge_LIBRARY_DIR ${install_dir}/lib)


ExternalProject_Add(
    bridge_proto
    GIT_REPOSITORY https://github.com/AibotBeginer/openbot_bridge.git
    GIT_TAG develop
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/third/openbot_bridge
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${install_dir} -DBUILD_SHARED_LIBS=ON
)
add_dependencies(openbot_bridge bridge_proto)
