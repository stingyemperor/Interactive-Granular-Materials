#pragma once
#include "../utils/Common.hpp"
#include "ParticleData.hpp"
#include <vector>

namespace PBD{ 
  struct Constraint;
  
  /** This class takes care of the 
  */
  struct SimulationModel{
    SimulationModel();
    SimulationModel(const SimulationModel&) = delete;
    SimulationModel& operator=(const SimulationModel&) = delete;
    ~SimulationModel();
    
    void init();
    void initParameters();
    
    typedef std::vector<Constraint*> ConstraintVector;

    ParticleData m_particles;
    ConstraintVector m_constraints;
    
    void cleanUp(); 
    void reset();
    
    ParticleData &getParticles();
    ConstraintVector &getConstraints();
    void resetContacts();
    void updatedConstraints();

    // Constraints
    bool addContactConstraint(const unsigned int &paticle1, const unsigned int &particle2);
    bool addPlaneConstraint(const unsigned int &paticle);
  };

}
