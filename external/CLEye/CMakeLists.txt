cmake_minimum_required(VERSION 2.8.12)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../)
include(${ROOT_DIR}/contrib/common.cmake)

project(PSMOVE_CLEYE_SERVER)

#CL_EYE_DRIVER
find_path(CL_EYE_SDK_PATH Bin/CLEyeMulticam.dll
    HINTS "C:/Program Files/Code Laboratories/CL-Eye Platform SDK"
    "C:/Program Files (x86)/Code Laboratories/CL-Eye Platform SDK")
IF(CL_EYE_SDK_PATH)
    add_definitions(-DCAMERA_CONTROL_HAS_CL_DRIVER)
    # Only directly link against the CLEye library in 32-bit systems
    list(APPEND PSMOVEAPI_TRACKER_REQUIRED_LIBS CLEyeMulticam)
    file(COPY ${CL_EYE_SDK_PATH}/Bin/CLEyeMulticam.dll
        DESTINATION ${ROOT_DIR}/external/libs)
    # XXX: If this crashes, disable compiler optimizations
ELSE()  # Fall back to registry version
    add_definitions(-DCAMERA_CONTROL_HAS_CL_DRIVER)
ENDIF()

#include_directories(${ROOT_DIR}/external/mingw-w64-headers)
link_directories(${ROOT_DIR}/external/libs)

add_executable(psmove_cleye_server ${CMAKE_CURRENT_LIST_DIR}/camera_control_cleye_server_win32.cpp)
target_link_libraries(psmove_cleye_server CLEyeMulticam)       
list(APPEND PSMOVEAPI_INSTALL_TARGETS psmove_cleye_server)        

# Install
install(TARGETS psmove_cleye_server
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)