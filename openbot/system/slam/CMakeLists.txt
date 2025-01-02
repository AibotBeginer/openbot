# openbot.application.slam_main
google_binary(openbot.application.slam_main
  SRCS
  slam_main.cc
)

# install
install(TARGETS
  openbot.application.slam_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
