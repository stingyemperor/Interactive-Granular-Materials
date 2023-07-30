#include "GranularModel.hpp"
#include "utils/CompactNSearch.h"

using namespace PBD;


GranularModel::GranularModel() : m_particles(){
  m_particleRadius = static_cast<Real>(0.2);
  m_neighborhoodSearch = NULL;
}

GranularModel::~GranularModel(void){
  cleanupModel();
}

void GranularModel::cleanupModel(){
  m_particles.release();
  m_boundaryX.clear();
  m_deltaX.clear();
  delete m_neighborhoodSearch;
}

ParticleData &GranularModel::getParticles(){
  return m_particles;
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
    m_neighborhoodSearch = new CompactNSearch::NeighborhoodSearch(1.0f);
  }
}
