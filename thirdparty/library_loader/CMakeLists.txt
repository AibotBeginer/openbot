cmake_minimum_required(VERSION 3.5)
project(library_loader CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Poco REQUIRED COMPONENTS Foundation)

include_directories(include 
  ${Boost_INCLUDE_DIRS} 
  ${Poco_INCLUDE_DIRS}
)

set(SRCS
  src/class_loader.cpp
  src/class_loader_core.cpp
  src/meta_object.cpp
  src/multi_library_class_loader.cpp
)

set(HDRS
  include/library_loader/class_loader.hpp
  include/library_loader/class_loader_core.hpp
  include/library_loader/exceptions.hpp
  include/library_loader/meta_object.hpp
  include/library_loader/multi_library_class_loader.hpp
  include/library_loader/register_macro.hpp
)

add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Poco_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT LibraryLoaderExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "include/*.hpp" "include/*.h")
set(HDRS ${ALL_LIBRARY_HDRS})

foreach(HDR ${ALL_LIBRARY_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION ${DIR}
  )
endforeach()

set(LIBRARY_LOADER_CMAKE_DIR share/library_loader/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  library_loader-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/library_loader/library_loader-config.cmake
  PATH_VARS LIBRARY_LOADER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/library_loader
)

install(
  EXPORT LibraryLoaderExport
  DESTINATION share/library_loader/cmake/
  FILE LibraryLoaderTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/library_loader/library_loader-config.cmake
  DESTINATION share/library_loader/
)
