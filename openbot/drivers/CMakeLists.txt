## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

# 检查是否是ARM架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  # 移除-mfa标志
  message(STATUS "Compiling for ARM architecture. Disabling -mfma flag.")
else()
  add_compile_options(-mfma)
endif()

# library camera_component
add_library(camera_component SHARED components/camera_component.cc)
target_link_libraries(camera_component ${PROJECT_NAME})
target_link_libraries(camera_component ${avcodec_LIBRARIES})
target_link_libraries(camera_component ${swscale_LIBRARIES})

# install
install(TARGETS
  camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


