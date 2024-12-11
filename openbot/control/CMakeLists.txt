
# library planning_component
add_library(controller_component SHARED components/controller_component.cc)
target_link_libraries(controller_component ${PROJECT_NAME})

# install
install(TARGETS
    controller_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)