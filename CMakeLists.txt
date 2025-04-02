# CMake version
cmake_minimum_required(VERSION 3.19)

# vcpkg auto-download and integration
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/vcpkg")
    # Download and install vcpkg if not found
    include(FetchContent)
    
    # Set vcpkg installation location 
    set(VCPKG_INSTALL_DIR "${CMAKE_SOURCE_DIR}/vcpkg" CACHE PATH "Where to install vcpkg")
    message(STATUS "VCPKG install dir: ${VCPKG_INSTALL_DIR}")
    
    # Download vcpkg
    FetchContent_Declare(
        vcpkg
        GIT_REPOSITORY https://github.com/Microsoft/vcpkg.git
        GIT_TAG master
        SOURCE_DIR "${VCPKG_INSTALL_DIR}"
    )
    
    FetchContent_GetProperties(vcpkg)
    if(NOT vcpkg_POPULATED)
        message(STATUS "Downloading vcpkg...")
        FetchContent_Populate(vcpkg)
        
        # Bootstrap vcpkg
        if(UNIX)
            message(STATUS "Bootstrapping vcpkg...")
            execute_process(
                COMMAND ./bootstrap-vcpkg.sh
                WORKING_DIRECTORY "${VCPKG_INSTALL_DIR}"
                RESULT_VARIABLE bootstrap_result
            )
        else()
            message(STATUS "Bootstrapping vcpkg...")
            execute_process(
                COMMAND ./bootstrap-vcpkg.bat
                WORKING_DIRECTORY "${VCPKG_INSTALL_DIR}"
                RESULT_VARIABLE bootstrap_result
            )
        endif()
        
        if(NOT bootstrap_result EQUAL 0)
            message(FATAL_ERROR "Failed to bootstrap vcpkg")
        endif()
    endif()
    
    set(VCPKG_ROOT "${VCPKG_INSTALL_DIR}" CACHE PATH "vcpkg root directory" FORCE)
endif()

# Set vcpkg toolchain file
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
CACHE STRING "Vcpkg toolchain file")

file(READ "${CMAKE_SOURCE_DIR}/vcpkg.json" VCPKG_JSON)
string(JSON PROJECT_NAME GET "${VCPKG_JSON}" "name")
string(JSON PROJECT_VERSION GET "${VCPKG_JSON}" "version")

project(${PROJECT_NAME} VERSION ${PROJECT_VERSION})

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build folders
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})

# Eigen
find_package(Eigen3 CONFIG REQUIRED)
find_package(boost_numeric_odeint CONFIG REQUIRED)

# Set 'RAL_SOURCES' variable
file(GLOB_RECURSE RAL_SOURCES CONFIGURE_DEPENDS "include/*.h"
                                                "src/*.cpp")

# Build RAL library
if(${BUILD_SHARED_LIBS})
    add_library(${PROJECT_NAME} SHARED ${RAL_SOURCES})
else()
    add_library(${PROJECT_NAME} STATIC ${RAL_SOURCES})
endif()

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)