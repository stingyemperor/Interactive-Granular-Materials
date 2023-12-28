#pragma once
#include "Common.hpp"
#include "Discregrid/cubic_lagrange_discrete_grid.hpp"
#include "utils/Common.hpp"
#include <array>
#include <eigen3/Eigen/src/Geometry/AlignedBox.h>
#include <vector>

namespace Common {
struct PossiblePoint;
}

namespace Discregrid {
class CubicLagrangeDiscreteGrid;
}

struct VolumeSampler {
  static std::vector<Vector3r> sampleMeshDense(
      const Matrix3rx &vertices, const Matrix3ix &indices,
      const Real &partRadius, const Real &cellSize, const int &maxSamples = -1,
      const bool &invert = false,
      const std::array<unsigned int, 3> &sdfResolution = {
          static_cast<unsigned int>(20), static_cast<unsigned int>(20),
          static_cast<unsigned int>(20)});

  static std::vector<Vector3r> sampleMeshRandom(
      const Matrix3rx &vertices, const Matrix3ix &indices,
      const Real &partRadius, const unsigned int &numTrials = 10,
      const Real &initialPointsDensity = 40, const bool &invert = false,
      const std::array<unsigned int, 3> &sdfResolution = {
          static_cast<unsigned int>(20), static_cast<unsigned int>(20),
          static_cast<unsigned int>(20)});

  static Discregrid::CubicLagrangeDiscreteGrid *
  generateSDF(const Matrix3rx &vertices, const Matrix3ix &indices,
              Eigen::AlignedBox<Real, 3> bbox,
              const std::array<unsigned int, 3> &resolution,
              const bool &invert);

  static double distanceToSDF(Discregrid::CubicLagrangeDiscreteGrid *sdf,
                              const Vector3r &x, const Real &thickness = 0.0f);

  static void
  generateInitialSetP(std::vector<Common::PossiblePoint> &possiblePoints,
                     const Eigen::AlignedBox<Real, 3> &bbox,
                     Discregrid::CubicLagrangeDiscreteGrid *sdf,
                     const unsigned int &numInitialPoints,
                     const Real &partRadius);

  static void parallelUniformVolumeSampling(
      std::vector<Vector3r> &samples,
      const std::vector<Common::PossiblePoint> &possiblePoints,
      const Real &minRadius, const unsigned int &numTrials,
      std::vector<std::vector<Vector3i>> &phaseGroups);
};
