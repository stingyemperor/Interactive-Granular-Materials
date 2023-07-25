#pragma once
#include "ParticleGenerator.hpp"
#include "ParticleUpdater.hpp"
#include <raylib.h>
#include <vector>


struct ParticleSystem{
  ParticleData particles;
  void draw(Camera &camera, Texture2D &sphere);  
  void update();
};

// class ParticleSystem{
// public:
//   ParticleData m_particles;
//   size_t m_count;

//   std::vector<std::shared_ptr<ParticleEmitter>> m_emitters;
//   std::vector<std::shared_ptr<ParticleUpdater>> m_updaters;

// public:
//   explicit ParticleSystem(size_t maxCount);
//   virtual ~ParticleSystem(){}
//   
//   ParticleSystem(const ParticleSystem &) = delete;
//   ParticleSystem &operator = (const ParticleSystem &) = delete;
//   
//   virtual void init();
//   virtual void update(double dt);

//   void draw();
//   virtual size_t numAllParticles() const {return m_particles.m_count;}

//   void addEmitter(std::shared_ptr<ParticleEmitter> em){m_emitters.push_back(em);}
//   void addUpdater(std::shared_ptr<ParticleUpdater> up){m_updaters.push_back(up);}
//   ParticleData *finalData() {return &m_particles;}
//   
// };
