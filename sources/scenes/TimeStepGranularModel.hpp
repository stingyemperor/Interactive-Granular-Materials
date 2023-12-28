#pragma once
#include "GranularModel.hpp"

namespace PBD {
struct TimeStepGranularModel {
  TimeStepGranularModel() {}
  ~TimeStepGranularModel() {}

  void clearAccelerations(GranularModel &granularModel);
  void constraintProjection(GranularModel &granularModel);

  void step(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch,
            std::uniform_real_distribution<double> &distribution,
            std::mt19937 &generator);
  void linearSolver(GranularModel &model);
  void parallelSolver(GranularModel &model);
  void boundaryConstraint(GranularModel &model, const unsigned int x1,
                          const unsigned int x2);
  void floorConstraint(GranularModel &model, const unsigned int x1);
  void floorFrictionConstraint(GranularModel &model, const unsigned int x1);
  void contactConstraint(GranularModel &model, const unsigned int x1,
                         const unsigned int x2);
  /**
   * @brief Solve contact constraints for two particles and add positional
   * friction force for the contacts.
   *
   * @param model
   * @param x1 index of the first particle
   * @param x2 index of the second particle
   */
  void contactConstraintFriction(GranularModel &model, const unsigned int x1,
                                 const unsigned int x2);
  void upsampledParticlesUpdate(GranularModel &model, const Real h);
  /**
   * @brief Update the boundary value for each each particle
   *
   * @param model
   */
  void checkBoundary(GranularModel &model);
  void deleteParticles(GranularModel &model);
  void merge2Particles(GranularModel &model,
                       std::uniform_real_distribution<double> &distribution,
                       std::mt19937 &generator,
                       CompactNSearch::NeighborhoodSearch &nsearch);
  /**
   * @brief Add flags for particles to be merged or split based on provided
   * conditions.
   *
   * @param model
   * @param nsearch
   */
  void setAdaptiveFlags(GranularModel &model,
                        CompactNSearch::NeighborhoodSearch &nsearch);
  /**
   * @brief Merge particles that are marked to be merged and set the merged
   * particle as inactive.
   *
   * @param model
   * @param nsearch
   */
  void merge(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch);
  /**
   * @brief Split particles that are marked to be split
   *
   * @param model
   * @param nsearch
   */
  void split(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch);
  /**
   * @brief Sets the given particle inactive and resizes the point set for
   * neighbor search.
   *
   * @param index
   * @param model
   * @param nsearch
   */
  void setInactive(const unsigned int index, GranularModel &model,
                   CompactNSearch::NeighborhoodSearch &nsearch);
  /**
   * @brief Sets the given particle active and resizes the point set for
   * neighbor search.
   *
   * @param index
   * @param model
   * @param nsearch
   */
  void setActive(GranularModel &model,
                 CompactNSearch::NeighborhoodSearch &nsearch);
  void reset();
  void applyForce(GranularModel &model);
  /**
   * @brief Calculate the average kinetic energy for the particles and write it
   * to a file.
   *
   * @param model
   * @param file
   */
  void calculateAverageEnergy(GranularModel &model, std::ofstream &file);
};
} // namespace PBD
