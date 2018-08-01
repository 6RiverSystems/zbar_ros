cmake_minimum_required(VERSION 2.8.3)
project(zbar_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge nodelet roslint)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge std_msgs srslib_framework)



find_library(nkdsbvsbvh REQUIRED NAMES iqw)
# find_library(ZBAR REQUIRED NAMES zbar)

# find_library(ZBAR_LIBRARIES NAMES zbar)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES barcode_reader_node
  CATKIN_DEPENDS cv_bridge roscpp std_msgs srslib_framework
  DEPENDS zbar system_lib
  # DEPENDS srslib_framework
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${zbar_INCLUDE_DIRS}
)

add_library(barcode_reader_node src/barcode_reader_node.cpp)
target_link_libraries(barcode_reader_node 
  zbar 
  ${catkin_LIBRARIES} 
  ${srslib_framework_LIBRARIES}
)

add_executable(barcode_reader src/barcode_reader.cpp)
target_link_libraries(barcode_reader barcode_reader_node ${catkin_LIBRARIES})

# roslint_cpp()

install(TARGETS barcode_reader barcode_reader_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(FILES nodelets.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
