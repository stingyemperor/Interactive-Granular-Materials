#include "GranularModel.hpp"
#include "entities/NeighborhoodSearchSpatialHashing.h"
#include "utils/CompactNSearch.h"
#include "utils/PointSet.h"

using namespace PBD;


GranularModel::GranularModel() : m_particles(){
  m_particleRadius = static_cast<Real>(0.025);
}

GranularModel::~GranularModel(void){
  cleanupModel();
}

void GranularModel::cleanupModel(){
  m_particles.release();
  m_boundaryX.clear();
  m_deltaX.clear();
  m_upsampledParticlesX.clear();
  m_upsampledParticlesV.clear();
  m_deleteFlag.clear();
  // delete m_compactNSearch;
}

ParticleData &GranularModel::getParticles(){
  return m_particles;
}

void GranularModel::reset(){
  const unsigned int nPoints = m_particles.size();
  for(unsigned int i = 0; i < nPoints; ++i){
    const Vector3r& x0 = m_particles.getPosition0(i);
    m_particles.setPosition(i, x0);
    m_particles.setOldPosition(i, x0);
    m_particles.getVelocity(i).setZero();
    m_particles.getAcceleration(i).setZero();
    m_deltaX[i].setZero();
    m_deleteFlag[i] = false; 
  }
}


void GranularModel::initMasses(){
  const int nParticles = (int)m_particles.size();
  #pragma omp parallel default(shared)
  {
    #pragma omp for schedule(static)
    for(int i = 0; i < nParticles; i++){
      m_particles.setMass(i, 1.0);
    }
  }
}

void GranularModel::initRadius(){
  const int nParticles = (int)m_particles.size();
  #pragma omp parallel default(shared)
  {
    #pragma omp for schedule(static)
    for(int i = 0; i < nParticles ;++i){
      m_particles.setRadius(i, m_particleRadius);
    }
  }
}  

void GranularModel::resizeGranularParticles(const unsigned int newSize){
  m_particles.resize(newSize);
  m_deltaX.resize(newSize);
  m_numConstraints.resize(newSize);
  m_deleteFlag.resize(newSize);
}

void GranularModel::releaseParticles(){
  m_particles.release();
  m_deltaX.clear();
  m_numConstraints.clear();
  m_deleteFlag.clear();
}

void GranularModel::initModel(const unsigned int nGranularParticles, Vector3r* granularParticles,
                              const unsigned int nBoundaryParticles, Vector3r* boundaryParticles,
                              const unsigned int nUpsampledParticles, Vector3r* upsampledParticles){


  releaseParticles();
  resizeGranularParticles(nGranularParticles);

  for(int i = 0; i < (int)nGranularParticles; ++i){
    m_particles.getPosition0(i) = granularParticles[i];
    m_deleteFlag[i] = false;
  }


  // TODO boundary psi stuff????
  m_boundaryX.resize(nBoundaryParticles);
  for(int i = 0; i < (int)nBoundaryParticles; ++i){
    m_boundaryX[i] = boundaryParticles[i];
  }

  m_upsampledParticlesX.resize(nUpsampledParticles);
  m_upsampledParticlesV.resize(nUpsampledParticles);
  for(int i = 0; i < (int)nUpsampledParticles; ++i){
    m_upsampledParticlesX[i] = upsampledParticles[i];
    m_upsampledParticlesV[i].setZero();
  }

  initMasses();
  initRadius();

  reset();
}

// neoghbors not including boundary particles
void GranularModel::generateNeighbors(CompactNSearch::NeighborhoodSearch &nsearch, unsigned int point_set_id_1, unsigned int point_set_id_2, unsigned int point_set_id_3){
  nsearch.find_neighbors();
  CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id_1);
  for(unsigned int i = 0 ; i < ps.n_points() ; ++i){
    std::vector<unsigned int> neighborParticle;
    std::vector<unsigned int> neighborBoundary;
    for(unsigned int j = 0; j < ps.n_neighbors(point_set_id_1,i); ++j){
      const unsigned int pid = ps.neighbor(point_set_id_1, i, j);
      neighborParticle.push_back(pid);
    }
    for(unsigned int k = 0; k < ps.n_neighbors(point_set_id_2, i); ++k){
      const unsigned int pid = ps.neighbor(point_set_id_2, i, k);
      neighborBoundary.push_back(pid);
    }
    m_neighbors.push_back(neighborParticle);
    m_boundaryNeighbors.push_back(neighborBoundary);
  }

  CompactNSearch::PointSet const& psUpsampled = nsearch.point_set(point_set_id_3);
  for(unsigned int i = 0; i < psUpsampled.n_points(); ++i){
    std::vector<unsigned int> neighborUpsampled;
    for(unsigned int j = 0; j < psUpsampled.n_neighbors(point_set_id_1, i);++j){
      const unsigned int pid = psUpsampled.neighbor(point_set_id_1, i, j);
      neighborUpsampled.push_back(pid);
    }
    m_upsampledNeighbors.push_back(neighborUpsampled);
  }
}

void GranularModel::clearNeighbors(){
  m_neighbors.clear();
  m_boundaryNeighbors.clear();
  m_upsampledNeighbors.clear();
}
