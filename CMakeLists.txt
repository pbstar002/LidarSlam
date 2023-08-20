cmake_minimum_required(VERSION 3.5)

project(LidarSLAM LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport cv_bridge
pcl_conversions
pcl_ros
pcl_msgs)

include_directories(${catkin_INCLUDE_DIRS})


find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets Sql REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Sql REQUIRED)
message("QT VERSION IS ")
message(${QT_VERSION_MAJOR})
#set(CMAKE_PREFIX_PATH "")
find_package(OpenMP REQUIRED)

# find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)
# Find the required packages for GPU acceleration
find_package(OpenGL REQUIRED)# Set compiler flags for hardware acceleration
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")# Add the necessary include directories
include_directories(${OPENGL_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS} )

# Link the necessary libraries
# target_link_libraries(your_target_name ${OPENGL_LIBRARIES})

add_definitions(-DQCUSTOMPLOT_USE_OPENGL)

# Load VTK Settings
include( ${VTK_USE_FILE} )
# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )


#add_subdirectory(tools/framlesshelper)
add_subdirectory(common)
#add_subdirectory(lightware)
#add_subdirectory(slam2d)
#add_subdirectory(imu)
add_subdirectory(modules/LidarSlamHome)
add_subdirectory(modules/LidarSlamUser)
add_subdirectory(modules/LidarSlamData)
add_subdirectory(modules/LidarSlamMap)
add_subdirectory(modules/LidarSlamCamera)

catkin_package()

set(PROJECT_SOURCES
        main.cpp
        res/qdarkstyle/style.qrc
        res/lidarslam.qrc
        ui/mainwindow.cpp
        ui/mainwindow.h
        ui/mainwindow.ui
#        ui/headbar.cpp
#        ui/headbar.h
#        ui/headbar.ui
#        ui/pbreadcrumb.cpp
#        ui/pbreadcrumb.h

       #  ui/configdialog.cpp
       #  ui/configdialog.h
       #  ui/configdialog.ui
         ui/firstpage.h
         ui/firstpage.cpp
         ui/firstpage.ui
       #  ui/waitwidget.h
       #  ui/waitwidget.cpp
       #  ui/waitwidget.ui


        tools/widgets/pfileexplorer.cpp
        tools/widgets/pfileexplorer.h
        .astylerc
)

set(APP_ICON_RESOURCE_WINDOWS "${CMAKE_CURRENT_SOURCE_DIR}/lidarslam.rc")

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(LidarSLAM
        ${PROJECT_SOURCES} ${APP_ICON_RESOURCE_WINDOWS}
    )
else()
    if(ANDROID)
        add_library(LidarSLAM SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(LidarSLAM
            ${PROJECT_SOURCES} ${APP_ICON_RESOURCE_WINDOWS}
        )
    endif()
endif()

target_include_directories(LidarSLAM PUBLIC
        ui
        tools/framlesshelper
        "${CMAKE_SOURCE_DIR}/modules/LidarSlamHome"
        "${CMAKE_SOURCE_DIR}/modules/LidarSlamUser"
        "${CMAKE_SOURCE_DIR}/modules/LidarSlamData"
        "${CMAKE_SOURCE_DIR}/modules/LidarSlamMap"
        "${CMAKE_SOURCE_DIR}/modules/LidarSlamCamera"
    #    "${CMAKE_SOURCE_DIR}/modules/NAME"
  
        )

target_link_libraries(LidarSLAM LINK_PUBLIC Common 
${catkin_LIBRARIES} 
${OPENGL_LIBRARIES}
${PCL_LIBRARIES}
 ${VTK_LIBRARIES}
 pcl_visualization )

target_link_libraries(LidarSLAM PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
#        FramelessHelper
        LidarSlamHome
        LidarSlamUser
        LidarSlamMap
        LidarSlamData
        LidarSlamCamera
        #NAME
        )
