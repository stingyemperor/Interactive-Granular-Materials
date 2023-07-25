#include "ParticleSystem.hpp"
#include "raylib.h"
// #include "raylib.h"

void ParticleSystem::draw(Camera &camera,Texture2D &sphere){
  for(unsigned int i = 0; i < max_count; ++i){
    Vector3 pos = {particles.position[i].x,particles.position[i].y, particles.position[i].z};
    // DrawSphereWires(pos, particles.radius, 6,6,RAYWHITE);
    DrawBillboard(camera, sphere, pos, 0.4f, WHITE);
  }
}
// ParticleSystem::ParticleSystem(size_t maxCount){
//   m_count = maxCount;
//   m_particles.generate(maxCount);
// }

// void ParticleSystem::init(){
//    for(auto& em : m_emitters){
//     em->emit(&m_particles);
//   }
// }

// void ParticleSystem::update(double dt){
//   for(auto& up : m_updaters){
//     up->update(dt, &m_particles);
//   }
// }

// void ParticleSystem::draw(){
//   for(int i = 0; i < m_count; ++i){
//     glm::vec4 m_pos = m_particles.m_pos[i];
//     Vector3 pos = {m_pos.x, m_pos.y, m_pos.z};
//     DrawSphereWires(pos, 0.1f, 6, 6, RAYWHITE);
//   }
// }
