# cmake example file from godot-cpp
# godot-cpp/test/CMakeLists.txt

cmake_minimum_required(VERSION 3.17)

project(vehicle-shapecast
    VERSION 1.0
    DESCRIPTION "This repository serves as a quickstart template for GDExtension development with Godot 4.0+."
    LANGUAGES CXX
)

set(TARGET_NAME "vehicle-shapecast")

add_subdirectory(godot-cpp SYSTEM)

file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(${TARGET_NAME} SHARED)

target_sources(${TARGET_NAME} PRIVATE
    ${SRC}
)

set(OUTPUT_DIR "${PROJECT_SOURCE_DIR}/project/addons/vehicle_shapecast/bin/")

target_include_directories(${TARGET_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

# Link to godot-cpp target
target_link_libraries(${TARGET_NAME} PRIVATE godot-cpp)

# Get useful properties from godot-cpp target
get_target_property(GODOTCPP_SUFFIX godot-cpp GODOTCPP_SUFFIX)

set_target_properties(
    ${TARGET_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_VISIBILITY_PRESET ${GODOTCPP_SYMBOL_VISIBILITY}

        POSITION_INDEPENDENT_CODE ON
        BUILD_RPATH_USE_ORIGIN ON

        # Try to ensure only static libraries are selected to be linked to.
        LINK_SEARCH_START_STATIC ON
        LINK_SEARCH_END_STATIC ON

        # NOTE: Wrapping the output variables inside a generator expression
        # prevents msvc generator from adding addition Config Directories
        LIBRARY_OUTPUT_DIRECTORY "$<1:${OUTPUT_DIR}>"
        RUNTIME_OUTPUT_DIRECTORY "$<1:${OUTPUT_DIR}>"
        PDB_OUTPUT_DIRECTORY     "$<1:${OUTPUT_DIR}>" #MSVC Only, ignored on other platforms

        PREFIX ""
        OUTPUT_NAME "${TARGET_NAME}${GODOTCPP_SUFFIX}"

        # Some IDE's respect this property to logically group targets
        FOLDER "godot-cpp"
)

# CMAKE_SYSTEM_NAME refers to the target system
if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set_target_properties(
        ${TARGET_NAME}
        PROPERTIES SUFFIX "" OUTPUT_DIR "${OUTPUT_DIR}/libmytest.macos.${GODOTCPP_TARGET}.framework"
    )
endif()