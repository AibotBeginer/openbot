
# library grpc_component
add_library(grpc_component SHARED compontents/grpc_component.cc)
target_link_libraries(grpc_component ${PROJECT_NAME})

# install
install(TARGETS
  grpc_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)