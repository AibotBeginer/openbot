# library camera_component
add_library(camera_component SHARED components/camera_component.cc)
target_link_libraries(camera_component ${PROJECT_NAME})

# install
install(TARGETS
  camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


