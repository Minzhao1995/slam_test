# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

#ros教程中的句子
include_directories(include ${catkin_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization sensor_msgs cv_bridge  image_transport pcl_conversions pcl_ros)

FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )

INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

ADD_LIBRARY( slambase SHARED slamBase.cpp tracking.cpp vo.cpp mapping.cpp pose_graph.cpp frame.cpp)
TARGET_LINK_LIBRARIES( slambase
   ${OpenCV_LIBS}
 ${PCL_LIBRARIES}
 ${OCTOMAP_LIBRARIES})

#ADD_EXECUTABLE( sender sender.cpp )
#TARGET_LINK_LIBRARIES( sender
#    ${OpenCV_LIBS}
#   ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#ADD_EXECUTABLE( listener listener.cpp )
#TARGET_LINK_LIBRARIES( listener
#   ${OpenCV_LIBS}
#  ${PCL_LIBRARIES} ${catkin_LIBRARIES})


