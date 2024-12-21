# library camera_component
add_library(map_component SHARED components/map_component.cc)
target_link_libraries(map_component ${PROJECT_NAME})

# install
install(TARGETS
    map_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


