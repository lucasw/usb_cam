cmake_minimum_required(VERSION 3.5)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# find_package(camera_info_manager REQUIRED)
# find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(class_loader REQUIRED)
find_package(internal_pub_sub REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

# idl stuff
# msg/Format.msg

include_directories(include
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(usbcam SHARED src/usb_cam_core.cpp src/usb_cam.cpp)
target_link_libraries(usbcam
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  # TODO(lucasw) should this have been in libavcodec?
  avutil
)
ament_target_dependencies(usbcam
  "builtin_interfaces"
  "class_loader"
  "internal_pub_sub"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
)
install(TARGETS
  usbcam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(usb_cam_node src/usb_cam_node.cpp)
target_link_libraries(usb_cam_node usbcam)
ament_target_dependencies(usb_cam_node rclcpp usbcam)
install(TARGETS usb_cam_node DESTINATION lib/${PROJECT_NAME})

# Don't use this it runs very poorly
install(PROGRAMS scripts/show_image.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
