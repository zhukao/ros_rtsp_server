cmake_minimum_required(VERSION 3.5)
project(ros_rtsp_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(img_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/rtsp_server_node/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rtsp_server_engine/include
  ${CMAKE_SYSROOT}/usr/include/liveMedia
  ${CMAKE_SYSROOT}/usr/include/groupsock
  ${CMAKE_SYSROOT}/usr/include/UsageEnvironment
  ${CMAKE_SYSROOT}/usr/include/BasicUsageEnvironment
  ${CMAKE_SYSROOT}/usr/include/jsoncpp
)

file(GLOB PROJECT_SRC
    "src/main.cc"
    "rtsp_server_node/src/rtsp_server_node.cpp"
    "rtsp_server_engine/src/rtsp_component.cc" 
    "rtsp_server_engine/src/rtsp_server/rtsp_server.cc"
)

set(BASE_LIBRARIES pthread jsoncpp
)

set(LIVE_LIB liveMedia BasicUsageEnvironment groupsock UsageEnvironment)
set(FFMPEG_LIBRARIES avformat avutil avcodec)
add_executable(${PROJECT_NAME} ${PROJECT_SRC})
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  img_msgs
)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${LIVE_LIB} ${FFMPEG_LIBRARIES} rt)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/data/
  DESTINATION lib/${PROJECT_NAME}/data/
)

ament_package()
