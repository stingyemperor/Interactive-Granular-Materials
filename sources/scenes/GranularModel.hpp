#pragma once 
#include "../utils/Common.hpp"
#include "../utils/CompactNSearch.h"
#include "../entities/ParticleData.hpp"
#include "../entities/NeighborhoodSearchSpatialHashing.h"

namespace PBD{

  struct GranularModel{
 
    GranularModel();
    ~GranularModel();
    
    Real m_particleRadius;
    ParticleData m_particles;
    std::vector<Vector3r> m_boundaryX;
    std::vector<Vector3r> m_deltaX;
    // std::vector<Vector3r> m_neighbors;
    std::vector<std::vector<unsigned int>> m_neighbors;
    std::vector<std::vector<unsigned int>> m_boundaryNeighbors;
    unsigned int m_pointId1;
    unsigned int m_pointId2;
  

    // CompactNSearch::NeighborhoodSearch *m_compactNSearch;
    void initMasses();
    void releaseParticles();
    void cleanupModel();
    void reset();
    void resizeGranularParticles(const unsigned int newSize);
    void generateNeighbors(CompactNSearch::NeighborhoodSearch &nsearch,unsigned int point_set_id, unsigned int point_set_id_2);

    void clearNeighbors();

    ParticleData &getParticles();
    void initModel(const unsigned int nGranularParticles, Vector3r* granularParticles, 
                   const unsigned int nBoundaryParticles, Vector3r* boundaryParticles); 
    const unsigned int numBoundaryParticles() const { return (unsigned int)m_boundaryX.size();} 
    Real getParticleRadius() const {return m_particleRadius;}
    void setParticleRadius(Real val) {m_particleRadius = val;}
    // CompactNSearch::NeighborhoodSearch* getCompactNSearch(){return m_compactNSearch;}
    
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


