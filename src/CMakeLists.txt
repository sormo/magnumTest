if(CORRADE_TARGET_EMSCRIPTEN)
    set(MAGNUM_WITH_EMSCRIPTENAPPLICATION ON CACHE BOOL "" FORCE)
else()
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/SDL2-2.0.10 ${CMAKE_PREFIX_PATH})
    set(MAGNUM_WITH_SDL2APPLICATION ON CACHE BOOL "" FORCE)
endif()

find_package(Magnum REQUIRED GL)

if(CORRADE_TARGET_EMSCRIPTEN)
    find_package(Magnum REQUIRED EmscriptenApplication)
else()
    find_package(Magnum REQUIRED Sdl2Application)
endif()

find_package(MagnumIntegration REQUIRED ImGui)

set (CMAKE_CXX_STANDARD 20)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(Application2D Application2D.cpp Application2D.h main.cpp utils.h utils.cpp)
target_link_libraries(Application2D PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::Primitives
    Magnum::Shaders
    MagnumIntegration::ImGui)

#if(CORRADE_TARGET_EMSCRIPTEN)
#    target_link_libraries(Application2D PRIVATE Magnum::EmscriptenApplication)
#endif()

# For Emscripten copy the boilerplate next to the executable so it can be run
# directly from the build dir; provide an install target as well
if(CORRADE_TARGET_EMSCRIPTEN)
    add_custom_command(TARGET Application2D POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${MAGNUM_EMSCRIPTENAPPLICATION_JS}
            ${MAGNUM_WEBAPPLICATION_CSS}
            ${CMAKE_CURRENT_SOURCE_DIR}/Application2D.html
            $<TARGET_FILE_DIR:Application2D>)

    install(TARGETS Application2D DESTINATION ${MAGNUM_DEPLOY_PREFIX})
    install(FILES
        Application2D.html
        ${MAGNUM_EMSCRIPTENAPPLICATION_JS}
        ${MAGNUM_WEBAPPLICATION_CSS}
        DESTINATION ${MAGNUM_DEPLOY_PREFIX})
    install(FILES
        $<TARGET_FILE_DIR:Application2D>/Application2D.js.mem
        $<TARGET_FILE_DIR:Application2D>/Application2D.wasm
        DESTINATION ${MAGNUM_DEPLOY_PREFIX} OPTIONAL)
endif()

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Application2D)
