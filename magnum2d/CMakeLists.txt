project(Magnum2D)

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
find_package(Magnum REQUIRED Text MagnumFont)

set(CMAKE_CXX_STANDARD 20)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

corrade_add_resource(Magnum2D_RESOURCES assets/resources.conf)

add_library(Magnum2D STATIC Magnum2D.h Magnum2D.cpp ${Magnum2D_RESOURCES})
target_link_libraries(Magnum2D PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::Primitives
    Magnum::Shaders
    Magnum::Text
    Magnum::MagnumFont
    MagnumIntegration::ImGui)

if(CORRADE_TARGET_EMSCRIPTEN)
    add_custom_command(TARGET Magnum2D POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${MAGNUM_EMSCRIPTENAPPLICATION_JS}
            ${MAGNUM_WEBAPPLICATION_CSS}
            ${CMAKE_CURRENT_SOURCE_DIR}/Magnum2D.html
            $<TARGET_FILE_DIR:Magnum2D>)

    install(TARGETS Magnum2D DESTINATION ${MAGNUM_DEPLOY_PREFIX})
    install(FILES
        Magnum2D.html
        ${MAGNUM_EMSCRIPTENAPPLICATION_JS}
        ${MAGNUM_WEBAPPLICATION_CSS}
        DESTINATION ${MAGNUM_DEPLOY_PREFIX})
    install(FILES
        $<TARGET_FILE_DIR:Magnum2D>/Magnum2D.js.mem
        $<TARGET_FILE_DIR:Magnum2D>/Magnum2D.wasm
        DESTINATION ${MAGNUM_DEPLOY_PREFIX} OPTIONAL)
endif()

set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Magnum2D)
