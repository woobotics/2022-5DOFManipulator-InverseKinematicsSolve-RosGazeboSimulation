##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(manipulator_22)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
 qt_build
 roscpp
 rospy
 roslib
 std_msgs
 kubo_msgs
 message_generation)

add_message_files(
 FILES
 state.msg
)

find_package(GAZEBO REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(INCLUDE_DIRS include
LIBRARIES manipulator_22)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/manipulator_22/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(manipulator_22 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(manipulator_22 ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS manipulator_22 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

