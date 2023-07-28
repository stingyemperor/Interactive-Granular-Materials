#pragma once
#include "Plane.hpp"
#include "ParticleData.hpp"
struct BoxConstraint{
  // std::vector<unsigned int> particles;
  std::array<unsigned int ,max_count> particles;
  // keep track of the number of particles that the constraint affects
  unsigned int num_particles{0};
  //Chcek if particle is colliding and add it to the particle list if it is
  void checkCollision(std::array<glm::vec3, max_count> &p, Plane &plane);
  // Solve the constraint
  void solve(std::array<glm::vec3,max_count> &pos_estimate,std::array<glm::vec3,max_count> &delta_x, Plane &plane,std::array<int, max_count> &num_constraints);

};
