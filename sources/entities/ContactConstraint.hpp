#pragma once
#include "ParticleData.hpp"
#include "utils/CompactNSearch.h"

struct ContactConstraint{

  std::array<std::vector<unsigned int>, max_count> particles;
  std::array<bool,max_count> has_collided;


  unsigned int num_particles{0};
  void generateContacts(std::array<std::vector<unsigned int>, max_count> &neighbors, std::array<glm::vec3, max_count> &positions, float radius);
  bool checkCollision(glm::vec3 &xi, glm::vec3 &xj, float radius);
  void solve(std::array<glm::vec3, max_count> &positions, std::array<glm::vec3, max_count> &delta_x, std::array<int,max_count> &num_constraints,float radius);
  ~ContactConstraint();
};
