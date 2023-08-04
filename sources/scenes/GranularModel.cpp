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
  }
}


void GranularModel::initMasses(){
  const int nParticles = (int)m_particles.size();
  const Real diam = static_cast<Real>(2.0) * m_particleRadius;

  #pragma omp parallel default(shared)
  {
    #pragma omp for schedule(static)
    for(int i = 0; i < nParticles; i++){
      m_particles.setMass(i, 1.0f);
    }
  }
}

void GranularModel::resizeGranularParticles(const unsigned int newSize){
  m_particles.resize(newSize);
  m_deltaX.resize(newSize);
}

void GranularModel::releaseParticles(){
  m_particles.release();
  m_deltaX.clear();
}

void GranularModel::initModel(const unsigned int nGranularParticles, Vector3r* granularParticles,
                              const unsigned int nBoundaryParticles, Vector3r* boundaryParticles){

  releaseParticles();
  resizeGranularParticles(nGranularParticles);

  for(int i = 0; i < (int)nGranularParticles; ++i){
    m_particles.getPosition0(i) = granularParticles[i];
  }


  // TODO boundary psi stuff????
  m_boundaryX.resize(nBoundaryParticles);
  for(int i = 0; i < (int)nBoundaryParticles; ++i){
    m_boundaryX[i] = boundaryParticles[i];
  }

  initMasses();

  reset();
}

// neoghbors not including boundary particles
void GranularModel::generateNeighbors(CompactNSearch::NeighborhoodSearch &nsearch, unsigned int point_set_id_1, unsigned int point_set_id_2){
  nsearch.find_neighbors();
  CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id_1);
  for(unsigned int i = 0 ; i < ps.n_points() ; ++i){
    std::vector<unsigned int> neighbors;
    for(unsigned int j = 0; j < ps.n_neighbors(point_set_id_1,i); ++j){
      //std::cout <<  j << "\n";

      const unsigned int pid = ps.neighbor(point_set_id_1, i, j);
      neighbors.push_back(pid);
      //m_neighbors[i].push_back(pid);
      //std::cout << "num_neighbors" + m_neighbors.size() << "\n";
    }
    m_neighbors.push_back(neighbors);
  }

  CompactNSearch::PointSet const& ps2 = nsearch.point_set(point_set_id_2);
  for(unsigned int i = 0 ; i < ps2.n_points() ; ++i){
    std::vector<unsigned int> neighbors;
    for(unsigned int j = 0; j < ps2.n_neighbors(point_set_id_2,i); ++j){
      //std::cout <<  j << "\n";

      const unsigned int pid = ps2.neighbor(point_set_id_2, i, j);
      neighbors.push_back(pid);
      //m_neighbors[i].push_back(pid);
      //std::cout << "num_neighbors" + m_neighbors.size() << "\n";
    }
    m_boundaryNeighbors.push_back(neighbors);
  }
}

void GranularModel::clearNeighbors(){
  m_neighbors.clear();
  m_boundaryNeighbors.clear();
}
