cmake_minimum_required(VERSION 2.8.3)
project(rtt_qhull_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(qhull_msgs)
ros_generate_rtt_service_proxies(qhull_msgs)

orocos_generate_package(
  DEPENDS qhull_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_geometry_msgs

)
