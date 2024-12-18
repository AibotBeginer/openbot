# grpc_component
if(${BUILD_GRPC})

  # library grpc_component
  add_library(grpc_component SHARED components/grpc_component.cc)
  target_link_libraries(grpc_component ${PROJECT_NAME})

  # application.grpc_client_main
  google_binary(application.grpc_client_main
    SRCS
      common/grpc/grpc_client_main.cc
  )

  # application.grpc_server_main
  google_binary(application.grpc_server_main
    SRCS
      common/grpc/grpc_server_main.cc
  )
  
  # install
  install(TARGETS
    grpc_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

