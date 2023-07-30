#pragma once 
#include "../utils/Common.hpp"
#include "../utils/CompactNSearch.h"
#include "../entities/ParticleData.hpp"

namespace PBD{

  struct GranularModel{
 
    GranularModel();
    ~GranularModel();
    
    Real m_particleRadius;
    ParticleData m_particles;
    std::vector<Vector3r> m_boundaryX;
    std::vector<Vector3r> m_deltaX;
    CompactNSearch::NeighborhoodSearch *m_neighborhoodSearch;
   
    void initMasses();
    void releaseParticles();
    void cleanupModel();
    void reset();
    void resizeGranularParticles(const unsigned int newSize);

    ParticleData &getParticles();
    void initModel(const unsigned int nGranularParticles, Vector3r* granularParticles, 
                   const unsigned int nBoundaryParticles, Vector3r* boundaryParticles); 
    const unsigned int numBoundaryParticles() const { return (unsigned int)m_boundaryX.size();} 
    Real getParticleRadius() const {return m_particleRadius;}
    void setParticleRadius(Real val) {m_particleRadius = val;}
    
    Vector3r& getBoundaryX(const unsigned int i){
      return m_boundaryX[i];
    }
    
    void setBoundaryX(const unsigned int i, const Vector3r &val){
      m_boundaryX[i];    
    }
    
    Vector3r& getDeltaX(const unsigned int i){
      return m_deltaX[i];
    }
    
    void setDeltaX(const unsigned int i, const Vector3r &val){
      m_deltaX[i] = val;
    }

  }; 
}


