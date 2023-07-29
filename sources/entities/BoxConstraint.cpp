// #include "BoxConstraint.hpp"

// void BoxConstraint::checkCollision(std::array<glm::vec3, max_count> &p, Plane &plane){
//   num_particles = 0;
//   for(unsigned int i  = 0; i < max_count; ++i){
//     // TODO change the collision test to fit more situations
//     if(p[i].y <= 0.0f){
//       particles[num_particles] = i;
//       num_particles++;
//     }
//   }
// }

// void BoxConstraint::solve(glm::vec3 &p1,std::array<glm::vec3, max_count> &delta_x, Plane &plane, std::array<int, max_count> &num_constraints){
//     float lambda = glm::dot(pos_estimate[particles[i]], plane.normal);
//     delta_x[particles[i]] += -1.0f * lambda * plane.normal;
//     num_constraints[particles[i]]++;

// }
