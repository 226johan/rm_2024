cmake_minimum_required(VERSION 3.27)
project(rm_2024)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)


find_library(mvsdk libMVSDK.so HINTS /usr/lib)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rm_2024 main.cpp
        include/mindvision/camera_mindvision_running.h
        include/mindvision/CameraApi.h
        include/mindvision/CameraDefine.h
        include/mindvision/CameraStatus.h
        src/mindvision/camera_mindvision_running.cpp
        include/debug.h
        src/Armor/armor.cpp
        include/Armor/armor.h
        include/Preprocess/Preprocess.h
        src/Armor/ArmorDetetion.cpp
        include/Armor/ArmorDetetion.h
        src/Tracker/Tracker.cpp
        include/Tracker/Tracker.h
        src/Resolver/Resolver.cpp
        include/Resolver/Resolver.h
        src/Resolver/GimbalControl.cpp
        include/Resolver/GimbalControl.h
        src/KalmanFilter/KalmanFilter.cpp
        include/KalmanFilter/KalmanFilter.h
        src/SerialPort/SerialPort.cpp
        include/SerialPort/SerialPort.h
)

target_link_libraries(rm_2024
        ${OpenCV_LIBS}
        ${mvsdk}
)
