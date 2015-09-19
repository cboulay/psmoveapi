cmake_minimum_required(VERSION 2.8.12)
set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
include(${ROOT_DIR}/contrib/common.cmake)

# Essential C utilities
foreach(UTILITY moved psmovepair psmove_dfu_mode psmove_auth_response psmove_get_firmware_info)
    add_executable(${UTILITY} ${CMAKE_CURRENT_LIST_DIR}/${UTILITY}.c)
    target_link_libraries(${UTILITY} psmoveapi_static)
    set_target_properties(${UTILITY} PROPERTIES
        COMPILE_FLAGS -DUSING_STATIC_LIBRARY)
    list(APPEND PSMOVEAPI_INSTALL_TARGETS ${UTILITY})
endforeach()

# Essential C++ utilities
foreach(UTILITY magnetometer_calibration)
    add_executable(${UTILITY} ${CMAKE_CURRENT_LIST_DIR}/${UTILITY}.cpp)
    target_link_libraries(${UTILITY} psmoveapi_static)
    set_target_properties(${UTILITY} PROPERTIES
        COMPILE_FLAGS -DUSING_STATIC_LIBRARY)
    list(APPEND PSMOVEAPI_INSTALL_TARGETS ${UTILITY})
endforeach()

# Essential C++ utilities using tracking
foreach(UTILITY smoothing_calibration)
    add_executable(${UTILITY} ${CMAKE_CURRENT_LIST_DIR}/${UTILITY}.cpp)
    target_link_libraries(${UTILITY} psmoveapi_static)
    target_link_libraries(${UTILITY} psmoveapi_tracker_static)
    set_target_properties(${UTILITY} PROPERTIES
        COMPILE_FLAGS -DUSING_STATIC_LIBRARY)
    list(APPEND PSMOVEAPI_INSTALL_TARGETS ${UTILITY})
endforeach()

# OpenGL based utilities
set(OPENGL_UTILITIES_LIBRARIES "")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(OPENGL_FRAMEWORK OpenGL)
    find_library(GLUT_FRAMEWORK GLUT)
    FIND_PACKAGE(SDL2)
    include_directories(${SDL2_INCLUDE_DIR})
    list(APPEND OPENGL_UTILITIES_LIBRARIES
        ${SDL2_LIBRARY} ${OPENGL_FRAMEWORK} ${GLUT_FRAMEWORK})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    list(APPEND OPENGL_UTILITIES_LIBRARIES SDL GL glut)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(ENV{SDL2DIR} ${ROOT_DIR}/external/SDL2/)
    FIND_PACKAGE(SDL2)
    include_directories(${SDL2_INCLUDE_DIR})
    list(APPEND OPENGL_UTILITIES_LIBRARIES ${SDL2_LIBRARY})
    list(GET SDL2_LIBRARY 0 FIRST_SDL2_LIB)
    get_filename_component(SDL_BIN_PATH ${FIRST_SDL2_LIB} DIRECTORY)
ENDIF()

# Co-registration Utility
FIND_PACKAGE(OCULUS_SDK)
IF(OCULUS_SDK_FOUND)
    add_executable(coregister_dk2 ${CMAKE_CURRENT_LIST_DIR}/coreg_calibration/coregister_dk2.cpp)
    include_directories(${OCULUS_SDK_INCLUDE_DIRS})
    target_link_libraries(coregister_dk2 
        psmoveapi 
        psmoveapi_tracker 
        ${OCULUS_SDK_LIBRARIES}
        ${OPENGL_UTILITIES_LIBRARIES})
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        add_custom_command(TARGET coregister_dk2 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SDL_BIN_PATH}/SDL2.dll"
                $<TARGET_FILE_DIR:coregister_dk2>)
    ENDIF()                
ENDIF(OCULUS_SDK_FOUND)