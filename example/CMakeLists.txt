# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

#ros教程中的句子
include_directories(include ${catkin_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization sensor_msgs cv_bridge  image_transport pcl_conversions pcl_ros)

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


ADD_EXECUTABLE( slamtest slam_test.cpp )
TARGET_LINK_LIBRARIES( slamtest
    slambase
libviso_zmz
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} ${catkin_LIBRARIES})
ADD_EXECUTABLE( stereo vo_test.cpp )
TARGET_LINK_LIBRARIES( stereo

    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} ${catkin_LIBRARIES})
