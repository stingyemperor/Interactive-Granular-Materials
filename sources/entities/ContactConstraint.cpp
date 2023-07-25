#include "ContactConstraint.hpp"
#include <array>
#include <glm/geometric.hpp>

bool ContactConstraint::checkCollision(glm::vec3 &xi, glm::vec3 &xj, float radius){
  glm::vec3 x_ij = xi - xj;
  float check = glm::length(x_ij)- (2.0f*radius); 
  if(check < 0.0f){
    return true;
  }
  return false;
}

void ContactConstraint::generateContacts(std::array<std::vector<unsigned int>, max_count> &neighbors, std::array<glm::vec3, max_count> &positions, float radius){
  // for each particle
  for(int i = 0 ; i < max_count; ++i){
    // for each neighbor for a particle
    for(unsigned int index : neighbors[i]){
      num_particles++;
      if(checkCollision(positions[i], positions[index], radius)){
        particles[num_particles].push_back(index);
      }
    } 
  }
}

void ContactConstraint::solve(std::array<glm::vec3, max_count> &positions, std::array<glm::vec3, max_count> &delta_x, float radius){
  for(int i = 0; i < num_particles; ++i){
    if(particles[i].size() > 0){
      for(int j = 0; j < particles[i].size(); ++j){
        glm::vec3 x_ij = positions[i] - positions[j];
        float x_ij_norm = glm::length(x_ij); 
        glm::vec3 delta_xi = -0.5f * (x_ij_norm - (2*radius)) * glm::normalize(x_ij);
        glm::vec3 delta_xj = -0.5f * (x_ij_norm - (2*radius)) * glm::normalize(x_ij);
        delta_x[i] += delta_xi;
        delta_x[j] += delta_xj;
      }
    }  
  }
}
