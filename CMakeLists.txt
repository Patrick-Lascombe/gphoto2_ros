cmake_minimum_required(VERSION 2.8.3)
project(gphoto2_ros)

find_package(catkin REQUIRED genmsg actionlib_msgs COMPONENTS
  message_generation
  roscpp
  self_test
  sensor_msgs
  std_msgs
  std_srvs
  actionlib_msgs
  actionlib
)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(gphoto2 REQUIRED libgphoto2)

add_service_files(FILES
  Capture.srv
  GetConfig.srv
  SetConfig.srv
  DownloadPictures.srv
  GetPicturePathList.srv
  DeletePictures.srv
)

add_action_files(DIRECTORY action
  FILES
  Trigger.action
  SetFocus.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gphoto2_ros
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs self_test actionlib_msgs
  DEPENDS OpenCV gphoto2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gphoto2_INCLUDE_DIRS}
)

add_library(gphoto2_ros
   src/libphoto2/photo_reporter.cpp
   src/libphoto2/photo_camera_list.cpp
   src/libphoto2/photo_camera.cpp
   src/libphoto2/photo_image.cpp
)

add_executable(photo_node src/photo_node/photo_node.cpp)
add_executable(photo_node_sim src/photo_node/photo_node_sim.cpp)
add_executable(test_photo src/test_photo/test_photo.cpp)

add_dependencies(photo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(photo_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_photo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gphoto2_ros
  usb-1.0
  -lstdc++fs
  ${OpenCV_LIBRARIES}
)  

target_link_libraries(photo_node
  gphoto2_ros
  usb-1.0
  ${catkin_LIBRARIES}
  ${gphoto2_LIBRARIES}
)

target_link_libraries(photo_node_sim
  gphoto2_ros
  usb-1.0
  ${catkin_LIBRARIES}
  ${gphoto2_LIBRARIES}
)

target_link_libraries(test_photo
  gphoto2_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS gphoto2_ros  test_photo photo_node photo_node_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
