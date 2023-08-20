cmake_minimum_required(VERSION 3.5)

project(LidarSlamMap LANGUAGES CXX)

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


find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets DataVisualization Charts REQUIRED) #DataVisualization
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets DataVisualization Charts REQUIRED) #DataVisualization

#find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Core Gui UiTools REQUIRED)

find_package(OpenMP REQUIRED)

add_definitions(-DQCUSTOMPLOT_USE_OPENGL)

set(SOURCES
    LidarSlamMapModule.cpp
    LidarSlamMapModule.h
    LidarSlamMapWidget.cpp
    LidarSlamMapWidget.h
    LidarSlamMapWidget.ui
    LidarSlamSensorWidget.cpp
    LidarSlamSensorWidget.h
    LidarSlamSensorWidget.ui
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
    Qt${QT_VERSION_MAJOR}::DataVisualization
    Qt${QT_VERSION_MAJOR}::Charts
    )

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Common)

# option(QCUSTOMPLOT_USE_OPENGL "Use OpenGL" ON)

# if(QCUSTOMPLOT_USE_OPENGL)
#     target_compile_definitions(${PROJECT_NAME} PUBLIC QCUSTOMPLOT_USE_OPENGL)
# endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>"
    )
