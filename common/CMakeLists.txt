cmake_minimum_required(VERSION 3.5)

project(Common LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(WIN32)
    set(CMAKE_DEBUG_POSTFIX d)
else()
    set(CMAKE_DEBUG_POSTFIX _debug)
endif()


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport cv_bridge pcl_conversions
pcl_ros
pcl_msgs)

include_directories(${catkin_INCLUDE_DIRS})

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Sql REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets PrintSupport REQUIRED)

find_package(OpenMP REQUIRED)

catkin_package()

set(SOURCES
    LidarSlamApplication.cpp
    LidarSlamApplication.h
    LidarSlamManager.cpp
    LidarSlamManager.h
    LidarSlamMainView.cpp
    LidarSlamMainView.h
    LidarSlamModuleBase.cpp
    LidarSlamModuleBase.h

    qtcustomplot/qcustomplot.cpp
    qtcustomplot/qcustomplot.h
    # qtcustomplot/QCPLinearColorMap.cpp
    # qtcustomplot/QCPLinearColorMap.h
    qtcustomplot/QCPColorGraph.cpp
    qtcustomplot/QCPColorGraph.h

    listener.cpp
    listener.h
    qNode.cpp
    qNode.h 
)	

add_library(${PROJECT_NAME} ${SOURCES})

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /utf-8)
    if(NOT (CMAKE_BUILD_TYPE STREQUAL "Debug"))
        target_compile_options(${PROJECT_NAME} PRIVATE /guard:cf)
        target_link_options(${PROJECT_NAME} PRIVATE /GUARD:CF)
    endif()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Sql
    Qt${QT_VERSION_MAJOR}::PrintSupport
    ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>"
)
