#pragma once

#include "BoxConstraint.hpp"
#include "ParticleData.hpp"
#include "utils/CompactNSearch.h"
#include <array>
#include "Plane.hpp"
#include <glm/glm.hpp>

struct Simulation{
  unsigned point_id;
  unsigned int solver_iterations;
  std::array<glm::vec3, max_count> pos_estimate;
  std::array<glm::vec3, max_count> delta_x;
  BoxConstraint boxConstraint;
  std::array<std::vector<unsigned int>, max_count> neighbors; 


  void resetNeighbor(CompactNSearch::NeighborhoodSearch &nsearch);
  void setPointSet(CompactNSearch::NeighborhoodSearch &nsearch, std::array<glm::vec3, max_count> &positions);
  void getNeighbors(CompactNSearch::NeighborhoodSearch &nsearch,std::array<std::vector<unsigned int>,max_count> &neighbors);
  void simulate(ParticleData *p, float dt, CompactNSearch::NeighborhoodSearch &nsearch, Plane &plane);
  void printVector(glm::vec3 &pos);
};
// class EulerUpdater : public ParticleUpdater{
// public:
//   glm::vec4 m_globalAcceleration{0.0f,-0.1f,0.0f,0.0f};
// public:
//   virtual void update(double dt, ParticleData *p) override;
// };

// class FloorUpdater : public ParticleUpdater{
// public:
//   float m_floorY{0.0f};
//   float m_bounceFactor{0.5f};
// public:
//   virtual void update(double dt, ParticleData *p) override;
// };

// class WindUpdater : public ParticleUpdater{
// public:
//   glm::vec4 wind = {0.01f, 0.0f,0.0f,0.0f};

// public:
//   virtual void update(double dt, ParticleData *p) override;
// };

// class Constraints : public ParticleUpdater{
// public:
//   unsigned int pointId;
//   // CompactNSearch::NeighborhoodSearch nsearch;
//   // void init(ParticleData *p);
//   void resetNeighbors(CompactNSearch::NeighborhoodSearch &nsearch);
//   void setPointSet(CompactNSearch::NeighborhoodSearch &nsearch, ParticleData *p);
//   std::vector<unsigned int> getNeighbors(CompactNSearch::NeighborhoodSearch &nsearch, ParticleData *p, int i);
//   void frictionConstraint(glm::vec4 &x_i, glm::vec4 &x_j);
//   void simulate(ParticleData *p, float dt, CompactNSearch::NeighborhoodSearch &nsearch);
//   virtual void update(double dt, ParticleData *p) override;
// };



