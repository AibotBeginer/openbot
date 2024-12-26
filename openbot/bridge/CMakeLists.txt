# grpc_component
if(${BUILD_GRPC})

  # library grpc_component
  add_library(grpc_component SHARED components/grpc_component.cc)
  target_link_libraries(grpc_component ${PROJECT_NAME})

  # openbot.application.grpc_client_main
  google_binary(openbot.application.grpc_client_main
    SRCS
      common/grpc/grpc_client_main.cc
  )

  # openbot.application.grpc_server_main
  google_binary(openbot.application.grpc_server_main
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

find_package(websocketpp REQUIRED)

# library websocket_component
add_library(websocket_component SHARED components/websocket_component.cc)
target_link_libraries(websocket_component ${PROJECT_NAME})
target_include_directories(websocket_component PRIVATE ${WEBSOCKETPP_INCLUDE_DIRS})

# install
install(TARGETS
  websocket_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)