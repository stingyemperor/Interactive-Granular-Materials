#pragma once
#include "Common.hpp"
#include <random>
#include <vector>

namespace Common {
struct PossiblePoint;
}

struct SurfaceSampler {
  static std::vector<Vector3r>
  sampleMesh(const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
             const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices,
             const Real &minRadius, const unsigned int &numTrials = 10,
             const Real &initialPointsDensity = 40,
             const unsigned int &distanceNorm = 1);

  static void computeFaceNormals(
      std::vector<Vector3r> &faceNormals,
      const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
      const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices);

  static void calculateTriangleAreas(
      std::vector<Real> &areas, Real &maxArea, Real &totalArea,
      const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
      const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices);

  static void generateInitalSetP(
      std::vector<Common::PossiblePoint> possiblePoints,
      std::vector<Real> &areas, Real &maxArea,
      const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
      const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices);

  static unsigned int generateRandomTriangleIndex(
      const std::vector<Real> &areas, const Real &maxArea, std::mt19937 &mt,
      std::uniform_real_distribution<Real> &uniformDist);

  static void parallelUniformSurfaceSampling(
      std::vector<Vector3r> &samples,
      const std::vector<Common::PossiblePoint> &possiblePoints,
      const unsigned int &numTrials, const Real &minRadius,
      const unsigned int &distanceNorm,
      const std::vector<Vector3r> &faceNormals);
};
