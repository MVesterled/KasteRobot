cmake_minimum_required(VERSION 3.5)
project(KasteRobot LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific settings
if(APPLE)
    # macOS specific includes and linking
    message(STATUS "Configuring for macOS")

    # Find OpenCV
    find_package(OpenCV REQUIRED)

    # Find ur_rtde
    find_package(ur_rtde REQUIRED)

    # Include Pylon framework for macOS
    find_library(PYLON_FRAMEWORK pylon PATHS "/Library/Frameworks/pylon.framework/Versions/A")
    include_directories("/Library/Frameworks/pylon.framework/Versions/A/Headers")
    include_directories("/Library/Frameworks/pylon.framework/Versions/A/Headers/GenICam/Base")
    include_directories("/Library/Frameworks/pylon.framework/Versions/A/Headers/GenICam/GeniApi")

    # Link the libraries
    set(PYLON_LIBS ${PYLON_FRAMEWORK})

else()
    # Linux specific includes and linking
    message(STATUS "Configuring for Linux")

    # Set the C++ flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

    # Find OpenCV
    find_package(OpenCV REQUIRED)

    # Find Pylon for Linux
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(Pylon REQUIRED)
    include_directories(${Pylon_INCLUDE_DIRS})
    set(PYLON_LIBS ${Pylon_LIBRARIES})

    # Find ur_rtde
    find_package(ur_rtde REQUIRED)
endif()

# Add the executable
add_executable(KasteRobot main.cpp Camera.h Camera.cpp)

# Link libraries (OpenCV, Pylon, ur_rtde)
target_link_libraries(KasteRobot PRIVATE ${OpenCV_LIBS} ur_rtde::rtde ${PYLON_LIBS})

# Install setup for macOS (optional)
if(APPLE)
    include(GNUInstallDirs)
    install(TARGETS KasteRobot
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()
