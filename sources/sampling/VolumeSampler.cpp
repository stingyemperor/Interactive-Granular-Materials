#include "sampling/VolumeSampler.hpp"
#include "Discregrid/All"
#include "sampling/Discregrid/cubic_lagrange_discrete_grid.hpp"
#include "sampling/Discregrid/triangle_mesh.hpp"
#include <array>
#include <cmath>
#include <random>
#include <vector>

std::vector<Vector3r> VolumeSampler::sampleMeshDense(
    const Matrix3rx &vertices, const Matrix3ix &indices, const Real &partRadius,
    const Real &cellSize, const int &maxSamples, const bool &invert,
    const std::array<unsigned int, 3> &sdfResolution) {

  auto bbox = Common::computeBoundingBox(vertices);

  Discregrid::CubicLagrangeDiscreteGrid *sdf =
      generateSDF(vertices, indices, bbox, sdfResolution, invert);

  const Real halfCellSize = cellSize / static_cast<Real>(2.0);

  unsigned int sampleCounter = 0;
  std::vector<Vector3r> samples;
  std::vector<unsigned int> freeCells;

  for (Real z = bbox.min()[2]; z <= bbox.max()[2]; z += cellSize) {
    for (Real y = bbox.min()[1]; y <= bbox.max()[1]; y += cellSize) {
      for (Real x = bbox.min()[0]; x <= bbox.max()[0]; x += cellSize) {
        auto offsetX = static_cast<Real>(0.0);
        auto offsetY = static_cast<Real>(0.0);
        auto offsetZ = static_cast<Real>(0.0);
        Vector3r particlePosition = {x + halfCellSize + offsetX,
                                     y + halfCellSize + offsetY,
                                     z + halfCellSize + offsetZ};

        if (distanceToSDF(sdf, particlePosition, -partRadius) < 0.0) {
          samples.push_back(particlePosition);
          freeCells.push_back(sampleCounter);
          sampleCounter++;
        }
      }
    }
  }
  return samples;
}

std::vector<Vector3r> VolumeSampler::sampleMeshRandom(
    const Matrix3rx &vertices, const Matrix3ix &indices, const Real &partRadius,
    const unsigned int &numTrials, const Real &initalPointsDensity,
    const bool &invert, const std::array<unsigned int, 3> &sdfResolution) {

  auto bbox = Common::computeBoundingBox(vertices);

  Discregrid::CubicLagrangeDiscreteGrid *sdf =
      generateSDF(vertices, indices, bbox, sdfResolution, invert);

  std::vector<Vector3r> samples;
  Real minRadius = static_cast<Real>(2.0) * partRadius;
  Real cellSize = minRadius / sqrt(3.0);

  const auto numInitialPoints = static_cast<unsigned int>(
      initalPointsDensity * (bbox.volume() / (cellSize * cellSize * cellSize)));

  std::vector<Common::PossiblePoint> possiblePoints;
  possiblePoints.reserve(numInitialPoints);

  std::vector<std::vector<Vector3i>> phaseGroups;
  phaseGroups.resize(27);

  generateInitialSetP(possiblePoints, bbox, sdf, numInitialPoints, partRadius);

  const Real factor = 1.0 / cellSize;

#pragma omp parallel for schedule(static)                                      \
    shared(possiblePoints, bbox, factor) default(none)
  for (auto &possiblePoint : possiblePoints) {
    const Vector3r &v = possiblePoint.pos;
    const int cellPos1 = Common::floor((v.x() - bbox.min()[0]) * factor) + 1;
    const int cellPos2 = Common::floor((v.y() - bbox.min()[1]) * factor) + 1;
    const int cellPos3 = Common::floor((v.z() - bbox.min()[2]) * factor) + 1;
    possiblePoint.cP = Vector3i(cellPos1, cellPos2, cellPos3);
  }

  Common::sort(possiblePoints, 0, static_cast<int>(possiblePoints.size() - 1));

  parallelUniformVolumeSampling(samples, possiblePoints, minRadius, numTrials,
                                phaseGroups);
  return samples;
}

Discregrid::CubicLagrangeDiscreteGrid *
VolumeSampler::generateSDF(const Matrix3rx &vertices, const Matrix3ix &indices,
                           Eigen::AlignedBox<Real, 3> bbox,
                           const std::array<unsigned int, 3> &resolution,
                           const bool &invert) {
  std::vector<Real> doubleVec;
  doubleVec.resize(3 * vertices.cols());
  for (unsigned int i = 0; i < vertices.cols(); ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      doubleVec[3 * i + j] = static_cast<Real>(vertices.col(i)[j]);
    }
  }
  Discregrid::TriangleMesh sdfMesh(&doubleVec[0], indices.data(),
                                   vertices.cols(), indices.cols());

  Discregrid::MeshDistance md(sdfMesh);
  Eigen::AlignedBox3d domain;
  domain.extend(bbox.min().cast<double>());
  domain.extend(bbox.max().cast<double>());
  domain.max() += 1.0e-3 * domain.diagonal().norm() * Vector3r::Ones();
  domain.min() -= 1.0e-3 * domain.diagonal().norm() * Vector3r::Ones();

  auto *distanceField =
      new Discregrid::CubicLagrangeDiscreteGrid(domain, resolution);
  auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
  auto factor = static_cast<Real>(1.0);
  if (invert)
    factor = static_cast<Real>(-1.0);
  func = [&md, &factor](Vector3r const &xi) {
    return factor * md.signedDistanceCached(xi);
  };

  distanceField->addFunction(func, false);

  return distanceField;
}

double VolumeSampler::distanceToSDF(Discregrid::CubicLagrangeDiscreteGrid *sdf,
                                    const Vector3r &x, const Real &thickness) {
  Vector3r xd = {static_cast<Real>(x.x()), static_cast<Real>(x.y()),
                 static_cast<Real>(x.z())};
  const double dist = sdf->interpolate(0, xd);
  if (dist == std::numeric_limits<Real>::max())
    return dist;
  return dist - thickness;
}

void VolumeSampler::generateInitialSetP(
    std::vector<Common::PossiblePoint> &possiblePoints,
    const Eigen::AlignedBox<Real, 3> &bbox,
    Discregrid::CubicLagrangeDiscreteGrid *sdf,
    const unsigned int &numInitialPoints, const Real &partRadius) {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<Real> uniformDist(0.0, 1.0);

#pragma omp parallel for schedule(static)
  for (int i = 0; i < numInitialPoints; i++) {
    // Random coordinates
    Real x =
        bbox.min().x() + uniformDist(mt) * (bbox.max().x() - bbox.min().x());
    Real y =
        bbox.min().y() + uniformDist(mt) * (bbox.max().y() - bbox.min().y());
    Real z =
        bbox.min().z() + uniformDist(mt) * (bbox.max().z() - bbox.min().z());

    Vector3r pos = Vector3r(x, y, z);
    if (distanceToSDF(sdf, pos, -partRadius) < 0.0) {
      Common::PossiblePoint p;
      p.pos = pos;
#pragma omp critical
      { possiblePoints.push_back(p); }
    }
  }
}

void VolumeSampler::parallelUniformVolumeSampling(
    std::vector<Vector3r> &samples,
    const std::vector<Common::PossiblePoint> &possiblePoints,
    const Real &minRadius, const unsigned int &numTrials,
    std::vector<std::vector<Vector3i>> &phaseGroups) {
  // Insert possible points into the HashMap
  std::unordered_map<Vector3i, Common::HashEntry, Common::HashFunc> hMap(
      2 * possiblePoints.size());
  samples.clear();
  samples.reserve(possiblePoints.size());

  const uint maxSamplesPerCell = 1u;

  // Insert first possible point
  {
    const Vector3i &cell = possiblePoints[0].cP;
    Common::HashEntry &entry = hMap[cell];
    entry.startIndex = 0;
    entry.samples.reserve(maxSamplesPerCell);
    int index = cell[0] % 3 + 3 * (cell[1] % 3) + 9 * (cell[2] % 3);
    phaseGroups[index].push_back(cell);
  }

  for (int i = 1; i < (int)possiblePoints.size(); i++) {
    const Vector3i &cell = possiblePoints[i].cP;
    if (cell != possiblePoints[i - 1].cP) {
      Common::HashEntry &entry = hMap[cell];
      entry.startIndex = i;
      entry.samples.reserve(maxSamplesPerCell);
      int index = cell[0] % 3 + 3 * (cell[1] % 3) + 9 * (cell[2] % 3);
      phaseGroups[index].push_back(cell);
    }
  }
  // Loop over number of tries to find a sample in a cell
  for (int t = 0; t < (int)numTrials; t++) {
    // Loop over the 27 cell groups
    for (const auto &cells : phaseGroups) {
      // Loop over the cells in each cell group
#pragma omp parallel for schedule(static)
      for (int i = 0; i < (int)cells.size(); i++) {
        const auto entryIt = hMap.find(cells[i]);
        // Check if cell exists
        if (entryIt != hMap.end()) {
          Common::HashEntry &entry = entryIt->second;
          // Check if a sample is already found for this cell
          if (entry.samples.size() > 0) {
            continue;
          }
          // Check if lower than max index
          if (entry.startIndex + t < possiblePoints.size()) {
            if (possiblePoints[entry.startIndex].cP ==
                possiblePoints[entry.startIndex + t].cP) {
              // Choose position corresponding to t-th trail from cell
              const Common::PossiblePoint &test =
                  possiblePoints[entry.startIndex + t];
              // Assign sample
              std::vector<Vector3r> tmp;
              if (!checkNeighbors(hMap, test, possiblePoints, minRadius, 0,
                                  tmp)) {
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
