project(template)

find_package(Magnum REQUIRED GL)

if(CORRADE_TARGET_EMSCRIPTEN)
    find_package(Magnum REQUIRED EmscriptenApplication)
else()
    find_package(Magnum REQUIRED Sdl2Application)
endif()

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

set(CMAKE_CXX_STANDARD 20)
add_compile_definitions(_SILENCE_ALL_CXX20_DEPRECATION_WARNINGS)

add_executable(template main.cpp)

target_link_libraries(template PRIVATE Magnum2D)

set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT template)