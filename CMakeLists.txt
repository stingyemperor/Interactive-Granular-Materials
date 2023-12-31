cmake_minimum_required(VERSION 3.22)
project(app CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_BUILD_TYPE Release)
include(FetchContent)

set(EIGEN_ROOT_DIR "/usr/local/include/eigen/3.4.0_1/include/eigen3/")
# set(ONEDPL_ROOT_DIR "/usr/local/include/onedpl/2022.0.0/include/")

if(APPLE)
  find_package(OpenMP REQUIRED)

  if(OPENMP_FOUND)
    if(CMAKE_VERSION VERSION_GREATER "3.8")
      link_libraries(OpenMP::OpenMP_CXX)
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
  endif(OPENMP_FOUND)
endif()

if(UNIX AND NOT APPLE)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    if(CMAKE_VERSION VERSION_GREATER "3.8")
      link_libraries(OpenMP::OpenMP_CXX)
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
  endif(OPENMP_FOUND)
endif()

if(NOT TARGET spdlog)
  find_package(spdlog REQUIRED)
endif()

# External dependencies Enable SIMD instructions
add_compile_options("-march=native")
add_compile_options("-mavx")
add_compile_options("-Wno-ignored-attributes") # AVX warnings
# ##############################################################################
# Add dependencies with FetchContent
# ##############################################################################

function(add_git_dependency libName gitURL gitTag)
  FetchContent_Declare(
    ${libName}
    GIT_REPOSITORY ${gitURL}
    GIT_TAG ${gitTag}
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE)
  FetchContent_MakeAvailable(${libName})
  target_compile_options(${libName} PRIVATE "-w")
endfunction()

if(APPLE)
  include_directories(
    ${EIGEN_ROOT_DIR}
    # ${ONEDPL_ROOT_DIR}
  )
endif()

# add glm
find_package(glm REQUIRED)

# add eigen
find_package(Eigen3 REQUIRED)

# Add Raylib
set(BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE) # don't build the supplied examples
set(BUILD_GAMES
    OFF
    CACHE BOOL "" FORCE) # don't build the supplied example games
add_git_dependency(raylib https://github.com/raysan5/raylib.git 4.5.0)

# ##############################################################################
# Project executable setup
# ##############################################################################

# Adding our source files Define PROJECT_SOURCES as a list of all source files
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_LIST_DIR}/sources/*.cpp")

# Define PROJECT_INCLUDE to be the path to the include directory of the project
set(PROJECT_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/sources/")

# Declaring our executable
add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDE})

target_link_libraries(${PROJECT_NAME} PRIVATE raylib)

# ##############################################################################
# Project build settings
# ##############################################################################

add_definitions(-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
# add_definitions(-DSOME_DEFINITION)
#
# Setting ASSETS_PATH
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -DDEBUG")

  # Set the asset path macro to the absolute path on the dev machine
  target_compile_definitions(
    ${PROJECT_NAME} PUBLIC ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/")
else()
  # Set the asset path macro in release mode to a relative path that assumes the
  # assets folder is in the same directory as the game executable
  target_compile_definitions(${PROJECT_NAME} PUBLIC ASSETS_PATH="./assets/")
endif()

# Set common compiler flags SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall
# -Wswitch")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if(${PLATFORM} STREQUAL "Web")
  # Tell Emscripten to build an .html file.
  set(CMAKE_EXECUTABLE_SUFFIX ".html")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} -s USE_GLFW=3 -s ASSERTIONS=1 -s WASM=1 -Os -Wall -s TOTAL_MEMORY=67108864 -s FORCE_FILESYSTEM=1 --preload-file assets/ --shell-file ../sources/minshell.html"
  )
  set(CMAKE_EXECUTABLE_SUFFIX ".html") # This line is used to set your
                                       # executable to build with the emscripten
                                       # html template so that you can directly
                                       # open it.
  set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".html")
  target_compile_definitions(
    ${PROJECT_NAME} PUBLIC ASSETS_PATH="/assets/") # Set the asset path macro in
                                                   # release mode to a relative
                                                   # path that assumes the
                                                   # assets folder is in the
                                                   # same directory as the game
                                                   # executable
endif()

# Ensure that hot-reload is enabled for VS
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /ZI")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /SAFESEH:NO")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SAFESEH:NO")
endif()

# openmP###################### if(APPLE) set(OpenMP_C "${CMAKE_C_COMPILER}")
# set(OpenMP_C_FLAGS "-Xclang -fopenmp
# -I/usr/local/include/libomp/17.0.1/include") set(OpenMP_C_LIB_NAMES "libomp")
# set(OpenMP_libomp_LIBRARY "omp")
#
# set(OpenMP_CXX "${CMAKE_CXX_COMPILER}") set(OpenMP_CXX_FLAGS "-Xclang -fopenmp
# -I/usr/local/include/libomp/17.0.1/include") set(OpenMP_CXX_LIB_NAMES
# "libomp") set(OpenMP_libomp_LIBRARY "omp") endif()
#
# if(APPLE) link_directories("/usr/local/include/libomp/17.0.1/lib")
# target_link_directories(${PROJECT_NAME} PRIVATE
# "/usr/local/include/libomp/17.0.1/lib") endif()
#

if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  find_package(oneDPL REQUIRED)
  target_link_libraries(${PROJECT_NAME} PUBLIC oneDPL)
endif()

if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Find TBB
  find_package(TBB REQUIRED)
  target_link_libraries(${PROJECT_NAME} PUBLIC TBB::tbb)
endif()

# ##########   tracy ###############################################
function(add_git_dependency_tracy libName gitURL gitTag)
  FetchContent_Declare(
    ${libName}
    GIT_REPOSITORY ${gitURL}
    GIT_TAG ${gitTag}
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE)
  FetchContent_MakeAvailable(${libName})
endfunction()
add_git_dependency_tracy(tracy https://github.com/wolfpld/tracy.git master)
target_link_libraries(${PROJECT_NAME} PUBLIC TracyClient)

# add benchmark library
find_package(benchmark REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC benchmark::benchmark)
