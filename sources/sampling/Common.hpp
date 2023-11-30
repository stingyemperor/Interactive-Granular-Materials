#pragma once
#include "../utils/Common.hpp"
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace Common {
/**
 * @class HashFunc
 * @brief Ddescribes the hash function used for surface and volume sampling
 *
 */
struct HashFunc {

  /**
   * @brief returns the hash fucntion for a cell coordinate
   *
   * @param k
   * @return
   */
  std::size_t operator()(const Vector3i &k) const {
    const int p1 = 73856093 * k[0];
    const int p2 = 19349663 * k[1];
    const int p3 = 83492791 * k[2];
    return static_cast<size_t>(p1 + p2 + p3);
  }
};

/**
 * @class PossiblePoint
 * @brief Set of possible points
 *
 */
struct PossiblePoint {
  /**
   * @brief Cell Position
   */
  Vector3i cP;
  /**
   * @brief Spatial Position
   */
  Vector3r pos;
  /**
   * @brief Traingle ID
   */
  unsigned int ID;
};

/**
 * @class HashEntry
 * @brief Entry in the spatial Hash
 *
 */
struct HashEntry {
  HashEntry(){};
  std::vector<unsigned int> samples;
  unsigned int startIndex;
};

/**
 * @brief  return the floor for a value
 *
 * @param v
 * @return
 */
static int floor(const Real v) { return static_cast<int>(v + 32768.f) - 32768; }

static Eigen::AlignedBox<Real, 3>
computeBoundingBox(const Eigen::Matrix<Real, 3, Eigen::Dynamic> &vertices) {
  Eigen::AlignedBox<Real, 3> box;
  box.min() = vertices.col(0);
  box.max() = box.min();
  box.setEmpty();

  // extend the box to include the vertices
  for (unsigned int i = 1; i < vertices.cols(); i++) {
    box.extend(vertices.col(i));
  }
  return box;
}

/**
 * @brief compare two given cells
 *
 * @param a
 * @param b
 * @return
 */
static bool compareCellID(Vector3i &a, Vector3i &b) {
  for (unsigned int i = 0; i < 3; ++i) {
    if (a[i] < b[i])
      return true;
    if (a[i] > b[i])
      return false;
  }
  return false;
}

/**
 * @brief Quick sort helper function for sorting the possible points
 *
 * @param possiblePoints
 * @param left
 * @param right
 * @return
 */
static int partition(std::vector<PossiblePoint> &possiblePoints,
                     const int &left, const int &right) {
  int i = left;
  int j = right;
  Vector3r tempPos;
  Vector3i temCell;
  PossiblePoint tmpPoint;
  Vector3i pivot = possiblePoints[left + (right - left) / 2].cP;

  while (i <= j) {
    while (compareCellID(possiblePoints[i].cP, pivot)) {
      i++;
    }

    while (compareCellID(pivot, possiblePoints[j].cP)) {
      j--;
    }

    if (i <= j) {
      tmpPoint = possiblePoints[i];
      possiblePoints[i] = possiblePoints[j];
      possiblePoints[j] = tmpPoint;
      i++;
      j--;
    }
  }
  return i;
}

static void sort(std::vector<PossiblePoint> &possiblePoints, const int &left,
                 const int right) {
  if (left < right) {
    int index = partition(possiblePoints, left, right);
    sort(possiblePoints, left, index - 1);
    sort(possiblePoints, index, right);
  }
}

/**
 * @brief Check if a cell is valid cell
 *
 * @param hMap
 * @param cell
 * @param point
 * @param possiblePoints
 * @param minRadius
 * @param distanceNorm
 * @param faceNormals
 * @return
 */
static bool
checkCell(const std::unordered_map<Vector3i, HashEntry, HashFunc> &hMap,
          const Vector3i cell, const PossiblePoint &point,
          const std::vector<PossiblePoint> &possiblePoints,
          const Real &minRadius, const unsigned int &distanceNorm,
          const std::vector<Vector3r> &faceNormals) {
  const auto nbEntryIt = hMap.find(cell);
  if (nbEntryIt != hMap.end()) {
    const HashEntry &nbEntry = nbEntryIt->second;
    for (unsigned int sample : nbEntry.samples) {
      const PossiblePoint &point2 = possiblePoints[sample];
      Real dist;
      if (distanceNorm == 0 || point.ID == point2.ID) {
        dist = (point.pos - point2.pos).norm();
      } else if (distanceNorm == 1) {
        Vector3r v = (point2.pos - point.pos).normalized();
        Real c1 = faceNormals[point.ID].dot(v);
        Real c2 = faceNormals[point2.ID].dot(v);

        dist = (point.pos - point2.pos).norm();
        if (std::abs(c1 - c2) > static_cast<Real>(0.00001)) {
          dist *= (asin(c1) - asin(c2)) / (c1 - c2);
        } else {
          dist /= (sqrt(1.0 - c1 * c2));
        }
      } else {
        return true;
      }
      if (dist < minRadius) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief check if a neighbor is too close
 *
 * @param hMap
 * @param point
 * @param possiblePoints
 * @param minRadius
 * @param distanceNorm
 * @param faceNormals
 * @return
 */
static bool
checkNeighbors(const std::unordered_map<Vector3i, HashEntry, HashFunc> &hMap,
               const PossiblePoint &point,
               const std::vector<PossiblePoint> &possiblePoints,
               const Real &minRadius, const unsigned int &distanceNorm,
               const std::vector<Vector3r> &faceNormals) {

  Vector3i nbPos = point.cP;
  if (checkCell(hMap, nbPos, point, possiblePoints, minRadius, distanceNorm,
                faceNormals))
    return true;

  for (int l = 1; l < 3; ++l) {
    for (int k = -l; k < l + 1; k += 2 * l) {
      for (int i = -l + 1; i < l; ++i) {
        for (int j = -l + 1; j < l; ++j) {
          nbPos = Vector3i(i, j, k) + point.cP;
          if (checkCell(hMap, nbPos, point, possiblePoints, minRadius,
                        distanceNorm, faceNormals))
            return true;
        }
      }

      for (int i = -l; i < l + 1; ++i) {
        for (int j = -l + 1; j < l; ++j) {
          nbPos = Vector3i(j, i, k) + point.cP;
          if (checkCell(hMap, nbPos, point, possiblePoints, minRadius,
                        distanceNorm, faceNormals)) {
            return true;
          }
        }

        for (int j = -l; j < l + 1; ++j) {
          nbPos = Vector3i(k, i, j) + point.cP;
          if (checkCell(hMap, nbPos, point, possiblePoints, minRadius,
                        distanceNorm, faceNormals)) {
            return true;
          }
        }
      }
    }
  }
  return false;
}
} // namespace Common
