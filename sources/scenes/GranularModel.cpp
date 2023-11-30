#include "GranularModel.hpp"
#include "simulation/NeighborhoodSearchSpatialHashing.h"
#include "utils/CompactNSearch.h"
#include "utils/PointSet.h"

using namespace PBD;

GranularModel::GranularModel() : m_particles() {
  m_particleRadius = static_cast<Real>(0.025);
  // maxRadius = std::cbrt(3 * pow(0.025,3));
  maxMass = 2;
  minMass = 1;
  samplingRadius = Vector3r(0.05, 0.05, 0.05);
  std::mt19937 gen(rd());
}

GranularModel::~GranularModel(void) { cleanupModel(); }

void GranularModel::cleanupModel() {
  m_particles.release();
  m_boundaryX.clear();
  m_deltaX.clear();
  m_upsampledParticlesX.clear();
  m_upsampledParticlesV.clear();
  m_deleteFlag.clear();
  m_mergeFlag.clear();
  m_isBoundary.clear();
  m_bigger.clear();
  m_smaller.clear();
  m_mergeCount.clear();
  m_inactive.clear();
  m_isActiveUpsampled.clear();
  m_inactiveUpsampled.clear();
  m_floorCollision.clear();
}

ParticleData &GranularModel::getParticles() { return m_particles; }

void GranularModel::reset() {
  const unsigned int nPoints = m_particles.size();
  for (unsigned int i = 0; i < nPoints; ++i) {
    const Vector3r &x0 = m_particles.getPosition0(i);
    m_particles.setPosition(i, x0);
    m_particles.setOldPosition(i, x0);
    m_particles.getVelocity(i).setZero();
    m_particles.getAcceleration(i).setZero();
    m_deltaX[i].setZero();
    m_deleteFlag[i] = false;
    m_mergeFlag[i] = false;
    m_isBoundary[i] = false;
  }
}

void GranularModel::initMasses() {
  const int nParticles = (int)m_particles.size();
#pragma omp parallel default(shared)
  {
#pragma omp for schedule(static)
    for (int i = 0; i < nParticles; i++) {
      m_particles.setMass(i, 1.0);
    }
  }
}

void GranularModel::initRadius() {
  const int nParticles = (int)m_particles.size();
#pragma omp parallel default(shared)
  {
#pragma omp for schedule(static)
    for (int i = 0; i < nParticles; ++i) {
      m_particles.setRadius(i, m_particleRadius);
    }
  }
}

void GranularModel::resizeGranularParticles(const unsigned int newSize) {
  m_particles.resize(newSize);
  m_deltaX.resize(newSize);
  m_numConstraints.resize(newSize);
  m_deleteFlag.resize(newSize);
  m_mergeFlag.resize(newSize);
  m_isBoundary.resize(newSize);
  m_bigger.reserve(newSize);
  m_smaller.reserve(newSize);
  m_mergeCount.reserve(newSize);
  m_inactive.reserve(newSize);
  m_floorCollision.reserve(newSize);
}

void GranularModel::releaseParticles() {
  m_particles.release();
  m_deltaX.clear();
  m_numConstraints.clear();
  m_deleteFlag.clear();
  m_mergeFlag.clear();
  m_isBoundary.clear();
  m_bigger.clear();
  m_smaller.clear();
  m_mergeCount.clear();
  m_inactive.clear();
  m_isActiveUpsampled.clear();
  m_floorCollision.clear();
}

void GranularModel::initModel(const unsigned int nGranularParticles,
                              Vector3r *granularParticles,
                              const unsigned int nBoundaryParticles,
                              Vector3r *boundaryParticles,
                              const unsigned int nUpsampledParticles,
                              Vector3r *upsampledParticles) {

  releaseParticles();
  resizeGranularParticles(nGranularParticles);

  for (int i = 0; i < (int)nGranularParticles; ++i) {
    m_particles.getPosition0(i) = granularParticles[i];
    m_deleteFlag[i] = false;
    m_mergeFlag[i] = false;
    m_isBoundary[i] = false;
    m_particles.m_isActive[i] = true;
    m_floorCollision[i] = false;
  }

  // TODO boundary psi stuff????
  m_boundaryX.resize(nBoundaryParticles);
  for (int i = 0; i < (int)nBoundaryParticles; ++i) {
    m_boundaryX[i] = boundaryParticles[i];
  }

  m_upsampledParticlesX.resize(nUpsampledParticles);
  m_upsampledParticlesV.resize(nUpsampledParticles);
  m_isActiveUpsampled.resize(nUpsampledParticles);
  m_inactiveUpsampled.reserve(nUpsampledParticles);

  for (int i = 0; i < (int)nUpsampledParticles; ++i) {
    m_upsampledParticlesX[i] = upsampledParticles[i];
    m_upsampledParticlesV[i].setZero();
    m_isActiveUpsampled[i] = true;
  }

  // m_neighbors.reserve(nGranularParticles);
  // m_boundaryNeighbors.reserve(nGranularParticles);
  // m_neighborsUpsampled.reserve(nGranularParticles);
  // m_upsampledNeighbors.reserve(nUpsampledParticles);
  // m_upsampledBoundaryNeighbors.reserve(nUpsampledParticles);

  initMasses();
  initRadius();

  reset();
}

// neighbors not including boundary particles
void GranularModel::generateNeighbors(
    CompactNSearch::NeighborhoodSearch &nsearch, unsigned int point_set_id_1,
    unsigned int point_set_id_2, unsigned int point_set_id_3) {
  // std::cout << "Particles : "<< m_particles.size() << "\n";
  nsearch.find_neighbors();
  // nsearch.resize_point_set(point_set_id_1,m_particles.getPosition(0).data(),
  // m_particles.size());

  CompactNSearch::PointSet const &ps = nsearch.point_set(point_set_id_1);

  for (unsigned int i = 0; i < ps.n_points(); ++i) {
    std::vector<unsigned int> neighborParticle;
    // std::vector<unsigned int> neighborBoundary;
    std::vector<unsigned int> neighborUpsampled;

    for (unsigned int j = 0; j < ps.n_neighbors(point_set_id_1, i); ++j) {
      const unsigned int pid = ps.neighbor(point_set_id_1, i, j);
      if (m_particles.getIsActive(pid)) {
        neighborParticle.push_back(pid);
      }
    }
    // for(unsigned int k = 0; k < ps.n_neighbors(point_set_id_2, i); ++k){
    //   const unsigned int pid = ps.neighbor(point_set_id_2, i, k);
    //   neighborBoundary.push_back(pid);
    // }

    for (unsigned int k = 0; k < ps.n_neighbors(point_set_id_3, i); ++k) {
      const unsigned int pid = ps.neighbor(point_set_id_3, i, k);
      neighborUpsampled.push_back(pid);
    }
    m_neighbors.push_back(neighborParticle);
    // m_boundaryNeighbors.push_back(neighborBoundary);
    m_neighborsUpsampled.push_back(neighborUpsampled);
  }

  CompactNSearch::PointSet const &psUpsampled =
      nsearch.point_set(point_set_id_3);
  for (unsigned int i = 0; i < psUpsampled.n_points(); ++i) {
    std::vector<unsigned int> neighborUpsampled;
    for (unsigned int j = 0; j < psUpsampled.n_neighbors(point_set_id_1, i);
         ++j) {
      const unsigned int pid = psUpsampled.neighbor(point_set_id_1, i, j);
      if (m_particles.getIsActive(pid)) {
        neighborUpsampled.push_back(pid);
      }
    }

    // std::vector<unsigned int> neighborBoundaryUpsampled;
    // for(unsigned int j = 0; j < psUpsampled.n_neighbors(point_set_id_2, i);
    // ++j){
    //   const unsigned int pid = psUpsampled.neighbor(point_set_id_2, i, j);
    //   neighborBoundaryUpsampled.push_back(pid);
    // }
    m_upsampledNeighbors.push_back(neighborUpsampled);
    // m_upsampledBoundaryNeighbors.push_back(neighborBoundaryUpsampled);
  }
}

void GranularModel::clearNeighbors() {
  m_neighbors.clear();
  m_boundaryNeighbors.clear();
  m_neighborsUpsampled.clear();
  m_upsampledNeighbors
      .clear(); // model.m_upsampledParticlesV[i] = (1.0 - alpha_i) * v_i_avg +
                // alpha_i*(model.m_upsampledParticlesV[i] +
                // delta_t*Vector3r(0.0,-9.81,0.0));
  m_upsampledBoundaryNeighbors.clear();

  // model.m_upsampledParticlesX[i] +=  delta_t *
  // model.m_upsampledParticlesV[i];
}
