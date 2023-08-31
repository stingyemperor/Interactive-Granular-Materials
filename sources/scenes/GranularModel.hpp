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
    Real m_ParticleRadiusUpsampled;
    ParticleData m_particles;
    std::vector<Vector3r> m_boundaryX;
    std::vector<Vector3r> m_deltaX;
    std::vector<Vector3r> m_upsampledParticlesX;
    std::vector<Vector3r> m_upsampledParticlesV;
    std::vector<bool> m_deleteFlag;
    std::vector<bool> m_mergeFlag;
    // std::vector<Vector3r> m_neighbors;
    std::vector<std::vector<unsigned int>> m_neighbors;
    std::vector<std::vector<unsigned int>> m_boundaryNeighbors;
    std::vector<std::vector<unsigned int>> m_upsampledNeighbors;
    std::vector<std::vector<unsigned int>> m_upsampledBoundaryNeighbors;
    std::vector<unsigned int> m_numConstraints;
    unsigned int m_pointId1;
    unsigned int m_pointId2;
    unsigned int m_pointId3;


    // CompactNSearch::NeighborhoodSearch *m_compactNSearch;
    void initMasses();
    void initRadius();
    void releaseParticles();
    void cleanupModel();
    void reset();
    void resizeGranularParticles(const unsigned int newSize);
    void generateNeighbors(CompactNSearch::NeighborhoodSearch &nsearch,unsigned int point_set_id, unsigned int point_set_id_2, unsigned int point_set_id_3);

    void clearNeighbors();

    ParticleData &getParticles();
    void initModel(const unsigned int nGranularParticles, Vector3r* granularParticles,
                   const unsigned int nBoundaryParticles, Vector3r* boundaryParticles,
                   const unsigned int nUpsampledParticles, Vector3r* upsampledParticles);

    const unsigned int numBoundaryParticles() const { return (unsigned int)m_boundaryX.size();}
    Real getParticleRadius() const {return m_particleRadius;}
    void setParticleRadius(Real val) {m_particleRadius = val;}
    Real getParticleRadiusUpsampled() const {return m_ParticleRadiusUpsampled;}
    void setParticleRadiusUpsampled(Real val) {m_ParticleRadiusUpsampled = val;}
    // CompactNSearch::NeighborhoodSearch* getCompactNSearch(){return m_compactNSearch;}

    Vector3r& getBoundaryX(const unsigned int i){
      return m_boundaryX[i];
    }

    void setBoundaryX(const unsigned int i, const Vector3r &val){
      m_boundaryX[i] = val;
    }

    Vector3r& getDeltaX(const unsigned int i){
      return m_deltaX[i];
    }

    void setDeltaX(const unsigned int i, const Vector3r &val){
      m_deltaX[i] = val;
    }

    Vector3r getUpsampledX(const unsigned int i){
      return m_upsampledParticlesX[i];
    }

    void setUpsampledX(const unsigned int i, const Vector3r &val){
      m_upsampledParticlesX[i] = val;
    }

    Vector3r getUpsampledV(const unsigned int i){
      return m_upsampledParticlesV[i];
    }

    void setUpsampledV(const unsigned int i, const Vector3r &val){
      m_upsampledParticlesV[i] = val;
    }

    void setNumConstraints(const unsigned int i, unsigned int n){
      m_numConstraints[i] = n;
    }

    bool getDeleteFlag(const unsigned int i){
      return m_deleteFlag[i];
    }

    void setDeleteFlag(const unsigned int i, const bool &flag){
      m_deleteFlag[i] = flag;
    }

    bool getMergeFlag(const unsigned int i){
      return m_mergeFlag[i];
    }

    void setMergeFlag(const unsigned int i, const bool &flag){
      m_mergeFlag[i] = flag;
    }

    unsigned int getNumConstraints(const unsigned int i){
      return m_numConstraints[i];
    }
  };
}


