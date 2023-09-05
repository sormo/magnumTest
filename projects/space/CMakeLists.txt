project(space)

find_package(Magnum REQUIRED GL)

if(CORRADE_TARGET_EMSCRIPTEN)
    find_package(Magnum REQUIRED EmscriptenApplication)
else()
    find_package(Magnum REQUIRED Sdl2Application)
endif()

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

set(CMAKE_CXX_STANDARD 20)
add_compile_definitions(_SILENCE_ALL_CXX20_DEPRECATION_WARNINGS)

corrade_add_resource(Space_RESOURCES assets/resources.conf)

add_executable(space main.cpp
                     utils.h
                     utils.cpp
                     point.h
                     point.cpp
                     simulation.h
                     simulation.cpp
                     simulationBodies.h
                     camera.h
                     camera.cpp
                     common.h
                     common.cpp
                     trajectory.h
                     trajectory.cpp
                     ship.h
                     ship.cpp
                     burnsHandler.h
                     burnsHandler.cpp
                     vectorHandler.h
                     vectorHandler.cpp
                     celestialObject.h
                     celestialObject.cpp
                     testMassPoint.h
                     testBodies.h
                     bodies.h
                     bodies.cpp
                     conicfit/conicApproximation.h
                     conicfit/conicApproximation.cpp
                     conicfit/conicFit.h
                     ${Space_RESOURCES})

target_link_libraries(space PRIVATE Magnum2D)

set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT space)