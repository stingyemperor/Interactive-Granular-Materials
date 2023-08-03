#include "GranularModel.hpp"
#include "entities/NeighborhoodSearchSpatialHashing.h"
#include "utils/CompactNSearch.h"
#include "utils/PointSet.h"

using namespace PBD;


GranularModel::GranularModel() : m_particles(){
  m_particleRadius = static_cast<Real>(0.2);
  m_neighborhoodSearch = NULL;
  // m_compactNSearch = NULL;
}

GranularModel::~GranularModel(void){
  cleanupModel();
}

void GranularModel::cleanupModel(){
  m_particles.release();
  m_boundaryX.clear();
  m_deltaX.clear();
  delete m_neighborhoodSearch;
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

  // initalize neighbourhood search
  if(m_neighborhoodSearch == NULL){
    m_neighborhoodSearch = new NeighborhoodSearchSpatialHashing(m_particles.size(),1.0f);
  }

  // m_neighborhoodSearch->setRadius(1.0f);

  // Initialize compactNsearchNeighborhood
  // m_compactNSearch->set_radius(0.1);
  reset();
}

// neighbors not including boundary particles
void GranularModel::generateNeighbors(CompactNSearch::NeighborhoodSearch &nsearch, unsigned int point_set_id, CompactNSearch::PointSet &ps){
  for(unsigned int i = 0 ; i < ps.n_points() ; ++i){
    for(unsigned int j = 0; j < ps.n_neighbors(point_set_id,i); ++j){
      m_neighbors[i].push_back(ps.neighbor(point_set_id, i, j)); 
    }  
  }
}
