
# library planning_component
add_library(planning_component SHARED components/planning_component.cc)
target_link_libraries(planning_component ${PROJECT_NAME})


# install
install(TARGETS
    planning_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)