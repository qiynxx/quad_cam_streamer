cmake_minimum_required(VERSION 3.10)
project(quad_cam_streamer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MPP REQUIRED rockchip_mpp)

# cppzmq provides cmake config
find_package(cppzmq QUIET)
if(NOT cppzmq_FOUND)
    # Fallback: find zmq via pkg-config
    pkg_check_modules(ZMQ REQUIRED libzmq)
endif()

# nlohmann_json
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    # Header-only, try pkg-config
    pkg_check_modules(NLOHMANN_JSON REQUIRED nlohmann_json)
endif()

file(GLOB SOURCES src/*.cpp)
list(FILTER SOURCES EXCLUDE REGEX ".*\\.bak$")
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${MPP_INCLUDE_DIRS}
    ${CMAKE_SYSROOT}/usr/include
    ${CMAKE_SYSROOT}/usr/include/rkaiq
    ${CMAKE_SYSROOT}/usr/include/rkaiq/uAPI2
    ${CMAKE_SYSROOT}/usr/include/rkaiq/common
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${MPP_LIBRARIES}
    pthread
    rkaiq
    asound
    m
)

if(cppzmq_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE cppzmq)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE ${ZMQ_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${ZMQ_LIBRARIES})
endif()

if(nlohmann_json_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
else()
    target_include_directories(${PROJECT_NAME} PRIVATE ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES config.json DESTINATION /etc/quad_cam_streamer/)
install(DIRECTORY calib/ DESTINATION /etc/quad_cam_streamer/calib/)
install(PROGRAMS S99quad_cam_streamer DESTINATION /etc/init.d/)
