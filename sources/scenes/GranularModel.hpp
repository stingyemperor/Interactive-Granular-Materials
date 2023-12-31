#pragma once
#include "../simulation/NeighborhoodSearchSpatialHashing.h"
#include "../simulation/ParticleData.hpp"
#include "../utils/Common.hpp"
#include "../utils/CompactNSearch.h"
#include "random"
#include <queue>
#include <utility>
#include <vector>

namespace PBD {

struct GranularModel {
  GranularModel();
  ~GranularModel();

  Real m_particleRadius;
  unsigned int m_numActiveParticles;
  unsigned int m_numMerging;
  unsigned int m_numSplitting;
  Real m_ParticleRadiusUpsampled;
  ParticleData m_particles;
  Vector3r samplingRadius;
  std::random_device rd;
  std::mt19937 gen;

  std::vector<Vector3r> m_boundaryX;
  std::vector<Vector3r> m_deltaX;
  std::vector<Vector3r> m_upsampledParticlesX;
  std::vector<Vector3r> m_upsampledParticlesV;
  std::vector<bool> m_deleteFlag;
  std::vector<bool> m_mergeFlag;
  std::vector<unsigned int> m_bigger;
  std::vector<unsigned int> m_smaller;
  std::vector<short int> m_mergeCount;
  std::vector<unsigned int> m_inactive;
  std::vector<bool> m_isActiveUpsampled;
  std::vector<unsigned int> m_inactiveUpsampled;
  std::vector<unsigned int> m_floorCollision;
  // std::vector<Vector3r> m_neighbors;
  std::vector<std::vector<unsigned int>> m_neighbors;
  std::vector<std::vector<unsigned int>> m_boundaryNeighbors;
  std::vector<std::vector<unsigned int>> m_upsampledNeighbors;
  std::vector<std::vector<unsigned int>> m_upsampledBoundaryNeighbors;
  std::vector<std::vector<unsigned int>> m_neighborsUpsampled;

  std::vector<std::pair<unsigned int, unsigned int>> m_mergeParticles;
  /**
   * @brief Keep track of particles that have to merge. The priority queue
   * makes sure the largest index is at the front to avoid overwrites.
   */
  std::priority_queue<unsigned int> m_merge;

  /**
   * @brief Keep track of particles that have to split
   */
  std::vector<unsigned int> m_splitParticles;

  std::vector<bool> m_isBoundary;
  std::vector<unsigned int> m_numConstraints;
  unsigned int m_pointIdLR;
  unsigned int m_pointIdBoundary;
  unsigned int m_pointIdUpsampled;
  // CompactNSearch::PointSet m_psLR;

  Real maxRadius;
  Real maxMass;
  Real minMass;

  // CompactNSearch::NeighborhoodSearch *m_compactNSearch;
  int numActiveParticles();
  void initMasses();
  void initRadius();
  void releaseParticles();
  void cleanupModel();
  void reset();
  void resizeGranularParticles(const unsigned int newSize);
  void generateNeighbors(CompactNSearch::NeighborhoodSearch &nsearch,
                         unsigned int point_set_id, unsigned int point_set_id_2,
                         unsigned int point_set_id_3);

  void resizeNeighbors(CompactNSearch::NeighborhoodSearch &nsearch);
  void clearNeighbors();

  ParticleData &getParticles();
  void initModel(const unsigned int nGranularParticles,
                 Vector3r *granularParticles,
                 const unsigned int nBoundaryParticles,
                 Vector3r *boundaryParticles,
                 const unsigned int nUpsampledParticles,
                 Vector3r *upsampledParticles);

  const unsigned int numBoundaryParticles() const {
    return (unsigned int)m_boundaryX.size();
  }
  Real getParticleRadius() const { return m_particleRadius; }
  void setParticleRadius(Real val) { m_particleRadius = val; }
  Real getParticleRadiusUpsampled() const { return m_ParticleRadiusUpsampled; }
  void setParticleRadiusUpsampled(Real val) { m_ParticleRadiusUpsampled = val; }
  // CompactNSearch::NeighborhoodSearch* getCompactNSearch(){return
  // m_compactNSearch;}

  Vector3r &getBoundaryX(const unsigned int i) { return m_boundaryX[i]; }

  void setBoundaryX(const unsigned int i, const Vector3r &val) {
    m_boundaryX[i] = val;
  }

  Vector3r &getDeltaX(const unsigned int i) { return m_deltaX[i]; }

  void setDeltaX(const unsigned int i, const Vector3r &val) {
    m_deltaX[i] = val;
  }

  Vector3r getUpsampledX(const unsigned int i) {
    return m_upsampledParticlesX[i];
  }

  void setUpsampledX(const unsigned int i, const Vector3r &val) {
    m_upsampledParticlesX[i] = val;
  }

  Vector3r getUpsampledV(const unsigned int i) {
    return m_upsampledParticlesV[i];
  }

  void setUpsampledV(const unsigned int i, const Vector3r &val) {
    m_upsampledParticlesV[i] = val;
  }

  void setNumConstraints(const unsigned int i, unsigned int n) {
    m_numConstraints[i] = n;
  }

  bool getDeleteFlag(const unsigned int i) { return m_deleteFlag[i]; }

  void setDeleteFlag(const unsigned int i, const bool &flag) {
    m_deleteFlag[i] = flag;
  }

  bool getMergeFlag(const unsigned int i) { return m_mergeFlag[i]; }

  void setMergeFlag(const unsigned int i, const bool &flag) {
    m_mergeFlag[i] = flag;
  }

  void setIsBoundary(const unsigned int i, const bool &b) {
    m_isBoundary[i] = b;
  }

  bool getIsBoundary(const unsigned int i) { return m_isBoundary[i]; }

  unsigned int getNumConstraints(const unsigned int i) {
    return m_numConstraints[i];
  }

  void updateMergingParticles(const unsigned int index);

  int16_t getNumNeighbors(const unsigned int index,
                          const CompactNSearch::NeighborhoodSearch &nsearch);
};
} // namespace PBD
