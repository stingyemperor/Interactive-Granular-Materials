#include "GranularModel.hpp"

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

void GranularModel::releaseParticles(){
  m_particles.release();
  m_deltaX.clear();
}

