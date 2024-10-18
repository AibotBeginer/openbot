cmake_minimum_required(VERSION 3.5)
project(class_loader CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
  include/class_loader/class_loader.hpp
  include/class_loader/class_loader_core.hpp
  include/class_loader/exceptions.hpp
  include/class_loader/meta_object.hpp
  include/class_loader/multi_library_class_loader.hpp
  include/class_loader/register_macro.hpp
)

add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Poco_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
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
