#include "SurfaceSampler.hpp"
#include "Common.hpp"
#include "utils/Common.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <math.h>
#include <random>
#include <unordered_map>
#include <vector>

using namespace Common;

std::vector<Vector3r> SurfaceSampler::sampleMesh(
    const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
    const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices,
    const Real &minRadius, const unsigned int &numTrials,
    const Real &initialPointsDensity, const unsigned int &distanceNorm) {

  std::vector<Vector3r> samples;
  Real cellSize = minRadius / sqrt(3.0);

  auto bbox = Common::computeBoundingBox(vertices);

  std::vector<Real> areas;

  Real maxArea = std::numeric_limits<Real>::min();

  auto totalArea = static_cast<Real>(0.0);
  calculateTriangleAreas(areas, maxArea, totalArea, vertices, indices);

  const Real circleArea = M_PI * minRadius * minRadius;
  const auto numInitialPoints = static_cast<unsigned int>(
      initialPointsDensity * (totalArea / circleArea));

  std::vector<PossiblePoint> possiblePoints;
  possiblePoints.resize(numInitialPoints);

  std::vector<Vector3r> faceNormals;
  computeFaceNormals(faceNormals, vertices, indices);

  generateInitalSetP(possiblePoints, areas, maxArea, vertices, indices);

  const Real factor = 1.0 / cellSize;

#pragma omp parallel for schedule(static)
  for (int i = 0; i < static_cast<int>(possiblePoints.size()); ++i) {
    const Vector3r &v = possiblePoints[i].pos;
    const int cellPos1 = Common::floor((v.x() - bbox.min()[0]) * factor) + 1;
    const int cellPos2 = Common::floor((v.y() - bbox.min()[1]) * factor) + 1;
    const int cellPos3 = Common::floor((v.z() - bbox.min()[2]) * factor) + 1;
    possiblePoints[i].cP = Vector3i(cellPos1, cellPos2, cellPos3);
  }

  sort(possiblePoints, 0, static_cast<int>(possiblePoints.size()) - 1);

  parallelUniformSurfaceSampling(samples, possiblePoints, numTrials, minRadius,
                                 distanceNorm, faceNormals);

  return samples;
}

void SurfaceSampler::computeFaceNormals(
    std::vector<Vector3r> &faceNormals,
    const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
    const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices) {

  const unsigned int numFaces = indices.cols();
  faceNormals.resize(numFaces);

#pragma omp parallel default(shared)
  {
#pragma omp for schedule(static)
    for (unsigned int i = 0; i < numFaces; ++i) {
      const Vector3r &a = vertices.col(indices.col(i)[0]);
      const Vector3r &b = vertices.col(indices.col(i)[1]);
      const Vector3r &c = vertices.col(indices.col(i)[2]);

      Vector3r v1 = b - a;
      Vector3r v2 = c - a;

      faceNormals[i] = v1.cross(v2);
      faceNormals[i].normalize();
    }
  }
}

void SurfaceSampler::calculateTriangleAreas(
    std::vector<Real> &areas, Real &maxArea, Real &totalArea,
    const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
    const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices) {

  const unsigned int numFaces = indices.cols();
  areas.resize(numFaces);
  Real tmpTotalArea = static_cast<Real>(0.0);
  Real tmpMaxArea = std::numeric_limits<Real>::min();

#pragma omp parallel default(shared)
  {
#pragma omp for reduction(+ : totalArea) schedule(static)
    for (int i = 0; i < static_cast<int>(numFaces); ++i) {
      const Vector3r &a = vertices.col(indices.col(i)[0]);
      const Vector3r &b = vertices.col(indices.col(i)[1]);
      const Vector3r &c = vertices.col(indices.col(i)[2]);

      const Vector3r d1 = b - a;
      const Vector3r d2 = c - a;

      const Real area = (d1.cross(d2)).norm() / 2.0;
      areas[i] = area;
      tmpTotalArea += area;

      if (area > tmpMaxArea) {
#pragma omp critical
        { tmpMaxArea = std::max(area, tmpMaxArea); }
      }
    }
  }
  maxArea = std::max(tmpMaxArea, maxArea);
  totalArea = tmpTotalArea;
}

void SurfaceSampler::generateInitalSetP(
    std::vector<Common::PossiblePoint> possiblePoints, std::vector<Real> &areas,
    Real &maxArea, const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices,
    const Eigen::Matrix<unsigned int, 3, Eigen::Dynamic> &indices) {

#pragma omp parallel default(shared)
  {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<Real> uniformDist(0.0, 1.0);

#pragma omp for schedule(static)
    for (int i = 0; i < static_cast<int>(possiblePoints.size()); ++i) {
      Real rand1 = sqrt(uniformDist(mt));
      Real u = 1.0 - rand1;
      Real v = uniformDist(mt) * rand1;
      Real w = 1.0 - u - v;

      const unsigned int randTriangleIndex =
          generateRandomTriangleIndex(areas, maxArea, mt, uniformDist);

      const Vector3r &a = vertices.col(indices.col(randTriangleIndex)[0]);
      const Vector3r &b = vertices.col(indices.col(randTriangleIndex)[1]);
      const Vector3r &c = vertices.col(indices.col(randTriangleIndex)[2]);

      possiblePoints[i].pos = u * a + v * b + w * c;
      possiblePoints[i].ID = randTriangleIndex;
    }
  }
}

unsigned int SurfaceSampler::generateRandomTriangleIndex(
    const std::vector<Real> &areas, const Real &maxArea, std::mt19937 &mt,
    std::uniform_real_distribution<Real> &uniformDist) {

  bool notValid = true;
  unsigned int index;
  while (notValid) {
    index = static_cast<unsigned int>(static_cast<Real>(areas.size()) *
                                      uniformDist(mt));
    if (uniformDist(mt) < areas[index] / maxArea) {
      notValid = false;
    }
  }
  return index;
}

void SurfaceSampler::parallelUniformSurfaceSampling(
    std::vector<Vector3r> &samples,
    const std::vector<Common::PossiblePoint> &possiblePoints,
    const unsigned int &numTrials, const Real &minRadius,
    const unsigned int &distanceNorm,
    const std::vector<Vector3r> &faceNormals) {

  std::vector<std::vector<Vector3i>> phaseGroups;
  phaseGroups.resize(27);

  std::unordered_map<Vector3i, HashEntry, HashFunc> hMap(2 *
                                                         possiblePoints.size());

  samples.clear();
  samples.reserve(possiblePoints.size());

  const unsigned int maxSamplesPerCell = 1u;
  {
    const Vector3i &cell = possiblePoints[0].cP;
    HashEntry &entry = hMap[cell];
    entry.startIndex = 0;
    entry.samples.reserve(maxSamplesPerCell);
    int index = cell[0] % 3 + 3 * (cell[1] % 3) + 9 * (cell[2] % 3);
    phaseGroups[index].push_back(cell);
  }

  for (int i = 1; i < static_cast<int>(possiblePoints.size()); ++i) {
    const Vector3i &cell = possiblePoints[i].cP;
    HashEntry &entry = hMap[cell];
    if (cell != possiblePoints[i - 1].cP) {
      HashEntry &entry = hMap[cell];
      entry.startIndex = i;
      entry.samples.reserve(maxSamplesPerCell);
      int index = cell[0] % 3 + 3 * (cell[1] % 3) + 9 * (cell[2] % 3);
      phaseGroups[index].push_back(cell);
    }
  }

  for (int t = 0; t < static_cast<int>(numTrials); ++t) {
    for (const auto &cells : phaseGroups) {
#pragma omp parallel for schedule(static)
      for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        const auto entryIt = hMap.find(cells[i]);
        if (entryIt != hMap.end()) {
          HashEntry &entry = entryIt->second;
          if (entry.samples.size() > 0) {
            continue;
          }
          if (entry.startIndex + t < possiblePoints.size()) {
            if (possiblePoints[entry.startIndex].cP ==
                possiblePoints[entry.startIndex + t].cP) {
              const PossiblePoint &test = possiblePoints[entry.startIndex + t];
              if (!checkNeighbors(hMap, test, possiblePoints, minRadius,
                                  distanceNorm, faceNormals)) {
                const int index = entry.startIndex + t;
#pragma omp critical
                {
                  entry.samples.push_back(index);
                  samples.push_back(possiblePoints[index].pos);
                }
              }
            }
          }
        }
      }
    }
  }
}
