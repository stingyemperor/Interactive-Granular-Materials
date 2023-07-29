// #include "ParticleGenerator.hpp"
// #include "entities/ParticleData.hpp"
// #include <eigen3/Eigen/src/Core/Matrix.h>
// // #include <glm/gtc/random.hpp>

// void BoxPosGen::generate(ParticleData *p){

//   unsigned int index = 0;
//   // Eigen::Vector3f start_pos(-2.0f,2.0f,0.0f);
//   glm::vec3 start_pos(-2.0f,2.0f,0.0f);
//   unsigned int side_length  = std::cbrt(max_count);
//   //TODO change top 2.0 instead of 2.1
//   float diameter = p->radius * 2.1f;

//   for(unsigned int x = 0; x < side_length; ++x){
//     for(unsigned int y = 0; y < side_length; ++y){
//       for(unsigned int z = 0; z < side_length; ++z){
//         // Eigen::Vector3f offset(x*diameter, y*diameter, -1.0f* z*diameter);
//         glm::vec3 offset(x*diameter, y*diameter, -1.0f* z*diameter);
//         p->position[index] = start_pos + offset;
//         index++;
//       }
//     }
//   }
// }

// // void BasicVelGen::generate(ParticleData *p, size_t maxCount){
// //   for(size_t i = 0; i < maxCount; ++i){
// //     p->m_vel[i] = glm::linearRand(m_minStartVel, m_maxStartVel);
// //   }
// // }
