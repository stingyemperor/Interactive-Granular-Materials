#include "BoxConstraint.hpp"

void BoxConstraint::checkCollision(std::array<glm::vec3, max_count> &p, Plane &plane){
  num_particles = 0;
  for(unsigned int i  = 0; i < max_count; ++i){
    // TODO change the collision test to fit more situations
    if(p[i].y <= 0.0f){
      particles[num_particles] = i;
      num_particles++;
    }
  }
}

void BoxConstraint::solve(std::array<glm::vec3, max_count> &pos_estimate,std::array<glm::vec3, max_count> &delta_x, Plane &plane, std::array<int, max_count> &num_constraints){
  for(unsigned int i = 0; i < num_particles; ++i){
      //TODO chamge it to save the delta x in another structure
    float lambda = glm::dot(pos_estimate[particles[i]], plane.normal);
    delta_x[particles[i]] += -1.0f * lambda * plane.normal;
    num_constraints[particles[i]]++;
  }
}
