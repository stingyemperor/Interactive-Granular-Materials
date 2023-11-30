#pragma once
#include "../utils/Common.hpp"
#include "ParticleData.hpp"
#include <array>
#include <random>

namespace PBD {
/**
 * @class Configuration
 * @brief Defines the starting cofiguration for the particles
 *
 */
struct Configuration {

  Configuration() = default;
  ~Configuration() = default;

  /**
   * @brief Samples particles in a cuboid region defined by the parameters
   *
   * @param particleRadius
   * @param startPos
   * @param width
   * @param height
   * @param depth
   */
  void cuboid(unsigned int particleRadius, Vector3r &startPos,
              unsigned int width, unsigned int height, unsigned int depth);

  /**
   * @brief Samples particles in a spherical region
   *
   * @param particleRadius
   * @param startPos
   * @param radius
   */
  void sphere(unsigned int particleRadius, Vector3r &startPos,
              unsigned int radius);

  void surfaceSampler();
};

} // namespace PBD
