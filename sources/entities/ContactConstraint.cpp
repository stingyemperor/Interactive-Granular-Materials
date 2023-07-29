// #include "ContactConstraint.hpp"
// #include <array>
// #include <glm/gtx/fast_square_root.hpp>

// bool ContactConstraint::checkCollision(glm::vec3 &xi, glm::vec3 &xj, float radius){
//   glm::vec3 x_ij = xi - xj;
//   float check = glm::fastLength(x_ij)- (2.0f*radius);
//   if(check < 0.0f){
//     return true;
//   }
//   return false;
// }

// void ContactConstraint::generateContacts(std::array<std::vector<unsigned int>, max_count> &neighbors, std::array<glm::vec3, max_count> &positions, float radius){
//   // TODO: fix the num_particles
//   num_particles = 0;
//   // for each particle
//   for(int i = 0 ; i < max_count; ++i){
//     if(neighbors[i].size() > 0){
//       // for each neighbor for a particle
//       for(unsigned int index : neighbors[i]){
//         if(checkCollision(positions[i], positions[index], radius)){
//           particles[i].push_back(index);
//           num_particles++;
//         }
//       }
//     }
//   }
// }

// void ContactConstraint::solve(std::array<glm::vec3, max_count> &positions, std::array<glm::vec3, max_count> &delta_x,std::array<int,max_count> &num_constraints,float radius){
//   if(num_particles>0){
//     for(int i = 0; i < max_count; ++i){
//       if(particles[i].size() > 0){
//         for(int j = 0; j < particles[i].size(); ++j){
//           int neighbor_index = particles[i][j];
//           glm::vec3 x_ij = positions[i] - positions[neighbor_index];

//           float k = 1.0f - powf((1.0f - 0.5f), 0.5f);

//           float x_ij_norm = glm::fastLength(x_ij);
//           float distance = x_ij_norm - (2.0f*radius);

//           // glm::vec3 delta_xi = (distance/2.0f)*(x_ij/x_ij_norm);
//           // glm::vec3 delta_xj = (distance/2.0f)*(x_ij/x_ij_norm);
//           glm::vec3 delta_xi =  0.5f * (distance) * glm::normalize(x_ij);
//           glm::vec3 delta_xj = -0.5f * (distance) * glm::normalize(x_ij);
//           delta_x[i] += delta_xi;
//           delta_x[neighbor_index] += delta_xj;
//           num_constraints[i]+= 1;
//           num_constraints[neighbor_index]+= 1;
//         }
//       }
//     }
//   }

// }

// ContactConstraint::~ContactConstraint(){
//   //has_collided.clear();
// }
