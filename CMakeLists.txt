cmake_minimum_required(VERSION 3.16)
project(carsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ---- raylib (built from source) --------------------------------------------
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
)
FetchContent_MakeAvailable(raylib)

# ---- raygui (header only: populate, don't build its examples) --------------
FetchContent_Declare(
    raygui
    GIT_REPOSITORY https://github.com/raysan5/raygui.git
    GIT_TAG        4.0
)
FetchContent_GetProperties(raygui)
if(NOT raygui_POPULATED)
    FetchContent_Populate(raygui)
endif()

add_executable(carsim main.cpp sim.cpp)
target_include_directories(carsim PRIVATE ${raygui_SOURCE_DIR}/src)
target_link_libraries(carsim PRIVATE raylib)

# copy the engine-sound asset next to the executable so it loads regardless of cwd
add_custom_command(TARGET carsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/idle.ogg $<TARGET_FILE_DIR:carsim>/idle.ogg)

# raylib needs these on macOS
if(APPLE)
    target_link_libraries(carsim PRIVATE
        "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()
