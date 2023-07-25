#pragma once

#include "ParticleData.hpp"
// #include <array>
#include <glm/gtc/random.hpp>

// // struct ParticleGenerator{
// // public:
// //   ParticleGenerator(){}
// //   virtual ~ParticleGenerator(){}

// //   virtual void generate(ParticleData *p, size_t maxCount) = 0;
// // };

struct BoxPosGen{
  void generate(ParticleData *p);
};

// struct BasicVelGen{
//   std::array<float, 3> minStartVel{0.0f,0.0f,0.0f};
//   std::array<float, 3> maxStartVel{1.0f,1.0f,1.0f};
//   void generate(ParticleData *p, size_t maxCount); 
// };
