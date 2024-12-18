## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

add_compile_options(-mfma)

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


