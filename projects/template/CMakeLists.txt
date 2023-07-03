set(CMAKE_CXX_STANDARD 20)

add_executable(template main.cpp)
target_link_libraries(template PRIVATE Magnum2D)

set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT template)
