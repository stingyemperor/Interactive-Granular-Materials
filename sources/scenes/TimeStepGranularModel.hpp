#pragma once
#include "GranularModel.hpp"

namespace PBD {
struct TimeStepGranularModel {
  void clearAccelerations(GranularModel &granularModel);
  void constraintProjection(GranularModel &granularModel);

  TimeStepGranularModel(){};
  ~TimeStepGranularModel(){};

  void step(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch,std::uniform_real_distribution<double> &distribution, std::mt19937 &generator);
  void boundaryConstraint(GranularModel &model, const unsigned int x1,
                          const unsigned int x2);
  void floorConstraint(GranularModel &model, const unsigned int x1);
  void floorFrictionConstraint(GranularModel &model, const unsigned int x1);
  void contactConstraint(GranularModel &model, const unsigned int x1,
                         const unsigned int x2);
  void contactConstraintFriction(GranularModel &model, const unsigned int x1,
                                 const unsigned int x2);
  void upsampledParticlesUpdate(GranularModel &model, const Real h);
  void checkBoundary(GranularModel &model);
  void mergeParticles(GranularModel &model);
  void deleteParticles(GranularModel &model);
  void merge2Particles(GranularModel &model,std::uniform_real_distribution<double> &distribution, std::mt19937 &generator);
  void reset();
  void applyForce(GranularModel &model);
  void calculateAverageEnergy(GranularModel &model, std::ofstream &file);
};
} // namespace PBD
