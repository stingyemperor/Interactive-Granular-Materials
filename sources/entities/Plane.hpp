#pragma once
#include "raylib.h"
#include <eigen3/Eigen/Core>
#include <glm/glm.hpp>

struct Plane{
  glm::vec3 normal;
  glm::vec3 position;

  void draw(){DrawGrid(10,1.0f);}
  Plane(){
    normal = glm::vec3(0.0f,1.0f,0.0f);
    position = glm::vec3(0.0f,0.0f,0.0f);
  }
};
