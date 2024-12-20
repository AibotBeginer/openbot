# library map_builder_compontent
add_library(map_builder_compontent SHARED components/map_builder_compontent.cc)
target_link_libraries(map_builder_compontent ${PROJECT_NAME})

# install
install(TARGETS
  map_builder_compontent
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)