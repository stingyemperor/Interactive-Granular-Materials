# CMAKE generated file: DO NOT EDIT!
# Generated by CMake Version 3.22
cmake_policy(SET CMP0009 NEW)

# PROJECT_SOURCES at CMakeLists.txt:57 (file)
file(GLOB_RECURSE NEW_GLOB LIST_DIRECTORIES false "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/*.cpp")
set(OLD_GLOB
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/entities/ParticleData.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/entities/ParticleEmitter.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/entities/ParticleGenerator.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/entities/ParticleRenderer.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/entities/ParticleSystem.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/entities/ParticleUpdater.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/main.cpp"
  "/home/empurrer/Documents/raylib-cpp-cmake-template/sources/utils/CompactNSearch.cpp"
  )
if(NOT "${NEW_GLOB}" STREQUAL "${OLD_GLOB}")
  message("-- GLOB mismatch!")
  file(TOUCH_NOCREATE "/home/empurrer/Documents/raylib-cpp-cmake-template/Debug/CMakeFiles/cmake.verify_globs")
endif()
