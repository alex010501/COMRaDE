cmake_minimum_required(VERSION 3.21)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg.json" VCPKG_JSON)
string(JSON PROJECT_NAME GET "${VCPKG_JSON}" "name")
string(JSON PROJECT_VERSION GET "${VCPKG_JSON}" "version")

project(${PROJECT_NAME} VERSION ${PROJECT_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)

set(COMRADE_ROOT_BIN_DIR "${CMAKE_SOURCE_DIR}/bin")
set(COMRADE_ROOT_LIB_DIR "${CMAKE_SOURCE_DIR}/lib")

function(comrade_set_output_dirs target runtime_base_dir archive_base_dir)
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${runtime_base_dir}/Debug"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${runtime_base_dir}/Release"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${runtime_base_dir}/RelWithDebInfo"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${runtime_base_dir}/MinSizeRel"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${runtime_base_dir}/Debug"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${runtime_base_dir}/Release"
        LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${runtime_base_dir}/RelWithDebInfo"
        LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${runtime_base_dir}/MinSizeRel"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${archive_base_dir}/Debug"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${archive_base_dir}/Release"
        ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${archive_base_dir}/RelWithDebInfo"
        ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL "${archive_base_dir}/MinSizeRel"
    )
endfunction()

find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS numeric_odeint)
find_package(assimp CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED)
find_package(DataFrame CONFIG REQUIRED)
find_package(urdfdom CONFIG REQUIRED)
find_package(tinyxml CONFIG REQUIRED)

file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})
comrade_set_output_dirs(${PROJECT_NAME} "${COMRADE_ROOT_BIN_DIR}" "${COMRADE_ROOT_LIB_DIR}")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        Boost::numeric_odeint
        assimp::assimp
        BulletDynamics
        BulletCollision
        LinearMath
        DataFrame::DataFrame
        unofficial-tinyxml::unofficial-tinyxml
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            $<$<CONFIG:Debug>:msvcprtd>
            $<$<NOT:$<CONFIG:Debug>>:msvcprt>
    )
endif()

target_compile_definitions(${PROJECT_NAME}
    PRIVATE DLL_LIBRARY_EXPORTS
)

add_subdirectory(example)
