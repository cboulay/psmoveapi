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