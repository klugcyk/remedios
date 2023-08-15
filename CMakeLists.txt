# 文件等级：密一
# author:klug
# 献给美人儿蕾梅黛丝
# start:230521
# last:230815

cmake_minimum_required(VERSION 3.5)

project(remedios VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# pcl
#find_package(PCL REQUIRED)
# opencv
find_package(OpenCV REQUIRED)
# eigen
find_package(Eigen3 REQUIRED)

# basler camera lib
include_directories(/opt/pylon/include)
include_directories(/opt/pylon/share/pylon/Samples/C++)
include_directories(/opt/pylon/share/pylon/Samples/C++/include)

# selbst
include_directories(~/buendia)
include_directories(~/remedios)
include_directories(~/undb)

# opencv
include_directories(/usr/local/include/opencv4)

# eigen
include_directories(${EIGEN3_INCLUDE_DIRS})

set(PROJECT_SOURCES
    main.cpp
    source.hpp
    # windows
    ~/remedios/ui_control/mainwindow.cpp
    ~/remedios/ui_control/mainwindow.h
    ~/remedios/ui_control/mainwindow.ui
    #~/remedios/ui_control/einlengthmeasure.cpp
    #~/remedios/ui_control/einlengthmeasure.h
    #~/remedios/ui_control/einlengthmeasure.ui

    # algorithm
    #~/remedios/img_process/laser_length_measure.cpp
    #~/remedios/img_process/laser_length_measure.hpp
    ~/remedios/img_process/laserLengthMeasure.cpp
    ~/remedios/img_process/laserLengthMeasure.hpp
    ~/remedios/img_process/zenturmExtract.cpp
    ~/remedios/img_process/zenturmExtract.hpp
    #~/remedios/galvo/galvo.hpp
    #~/remedios/galvo/galvo.cpp
    #~/remedios/ander/laser_zenturm_extract.cpp
    #~/remedios/ander/laser_zenturm_extract.hpp

    # outside
    #~/buendia/galvo_control/galvo_control.cpp
    #~/buendia/galvo_control/galvo_control.hpp
    #~/buendia/socket/socket.cpp
    #~/buendia/socket/socket.hpp
    #~/buendia/galvo_control/Crc16Class.cpp
    #~/buendia/galvo_control/Crc16Class.h
    ~/buendia/camera/camera.cpp
    ~/buendia/camera/camera.hpp
    ~/undb/cameraGene/cameraGene.cpp
    ~/undb/cameraGene/cameraGene.hpp
    #~/undb/math/least_sqaure.cpp
    #~/undb/math/least_sqaure.hpp

)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(remedios
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET remedios APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(remedios SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(remedios
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(remedios PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    pthread

    /opt/pylon/lib/libgxapi.so
    /opt/pylon/lib/libPylonDataProcessing.so
    /opt/pylon/lib/libPylonDataProcessingGui.so
    /opt/pylon/lib/libXmlParser_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libpylon_TL_gtc.so
    /opt/pylon/lib/libpylonutility.so
    /opt/pylon/lib/libGenApi_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libpylonc.so
    /opt/pylon/lib/libLog_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libNodeMapData_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/liblog4cpp_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libPylonDataProcessingCore.so
    /opt/pylon/lib/libMathParser_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libFirmwareUpdate_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libuxapi.so
    /opt/pylon/lib/libpylon_TL_camemu.so
    /opt/pylon/lib/libGCBase_gcc_v3_1_Basler_pylon.so
    /opt/pylon/lib/libpylon_TL_gige.so
    /opt/pylon/lib/libpylonbase.so
    /opt/pylon/lib/pylon-libusb-1.0.so
    /opt/pylon/lib/libpylon_TL_usb.so
    /opt/pylon/lib/libpylonc.so.7.2
    /opt/pylon/lib/libuxapi.so.7.2
    /opt/pylon/lib/libgxapi.so.7.2
    /opt/pylon/lib/libpylonbase.so.7.2
    /opt/pylon/lib/libpylonutility.so.7.2
    /opt/pylon/lib/libgxapi.so.7.2.1
    /opt/pylon/lib/libpylonbase.so.7.2.1
    /opt/pylon/lib/libpylonc.so.7.2.1
    /opt/pylon/lib/libuxapi.so.7.2.1
    /opt/pylon/lib/libpylonutility.so.7.2.1

    ~/sakura/lib/libmath.so
    ~/sakura/lib/libcameraGene.so

    ${OpenCV_LIBS}
    )

set_target_properties(remedios PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(remedios)
endif()
