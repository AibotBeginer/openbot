cmake_minimum_required(VERSION 3.5)
project(pluginlib)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# find_package(class_loader REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(TinyXML2 REQUIRED)

include_directories(include  
  ${Boost_INCLUDE_DIRS} 
  ${TinyXML2_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "include/*.hpp" "include/*.h")
set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS})

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION ${DIR}
  )
endforeach()

# Prepare "Catch2" library for other executables
add_library(pluginlib INTERFACE)
target_include_directories(pluginlib INTERFACE ${ALL_LIBRARY_HDRS})