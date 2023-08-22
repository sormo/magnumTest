cmake_minimum_required(VERSION 3.4)
project(Magnum2D)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(MAGNUM_WITH_MAGNUMFONT ON)

add_subdirectory(corrade)
add_subdirectory(magnum)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(MAGNUM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/magnum/src)
set(CORRADE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/corrade/src)
set(MAGNUM2D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/magnum2d)
set(MAGNUM_WITH_IMGUI ON CACHE BOOL "" FORCE)
add_subdirectory(magnum-integration EXCLUDE_FROM_ALL)

include_directories(${IMGUI_DIR})
include_directories(${MAGNUM_DIR})
include_directories(${CORRADE_DIR})
include_directories(${MAGNUM2D_DIR})

include_directories(${CMAKE_BINARY_DIR}/magnum/src)

add_subdirectory(magnum2d)
add_subdirectory(projects)
