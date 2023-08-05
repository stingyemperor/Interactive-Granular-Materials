#include "TimeStepGranularModel.hpp"
#include "../entities/TimeManager.hpp"
#include "../entities/Simulation.hpp"
#include "entities/ParticleData.hpp"
#include "entities/TimeIntegration.hpp"
#include "utils/CompactNSearch.h"

using namespace PBD;

void TimeStepGranularModel::step(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch){
  
  TimeManager *tm = TimeManager::getCurrent();
  const Real h = tm->getTimeStepSize();
  ParticleData &pd = model.getParticles();

  clearAccelerations(model);

  // TIme intergration
  for(unsigned int i = 0; i < pd.size(); ++i){
    model.getDeltaX(i).setZero();
    pd.getOldPosition(i) = pd.getPosition(i);
    TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
  }

  // Neighborhood search
  model.generateNeighbors(nsearch, model.m_pointId1,model.m_pointId2);  
  // Constraint Projection
  constraintProjection(model);
    

  // Update velocity
  for(unsigned int i = 0; i < pd.size(); ++i){
    TimeIntegration::VelocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
  }

  // Clear Neighbors
  model.clearNeighbors();
}

void TimeStepGranularModel::clearAccelerations(GranularModel &granularModel){
  ParticleData &pd = granularModel.getParticles();
  const unsigned int count = pd.size();
  Simulation* sim = Simulation::getCurrent();
  const Vector3r grav(0.0,-9.81,0.0);
  for(unsigned int i = 0; i < count; ++i){
    if(pd.getMass(i) != 0.0){
      Vector3r &a = pd.getAcceleration(i);
      a = grav;
    }
  } 
}

void TimeStepGranularModel::reset(){}

void TimeStepGranularModel::constraintProjection(GranularModel &model){
  const unsigned int maxIter = 2;
  unsigned int iter = 0;

  ParticleData &pd = model.getParticles();

  for(unsigned int i = 0; i < pd.size() ; ++i){
    model.getDeltaX(i).setZero();
    model.setNumConstraints(i, 0);
  }

  while(iter < maxIter){
    for(unsigned int i = 0; i < pd.size(); ++i){

      // boundary collision handling

      for(unsigned int boundaryIndex : model.m_boundaryNeighbors[i]){
        Vector3r p1 = model.getParticles().getPosition(i);
        Vector3r p2 = model.getBoundaryX(boundaryIndex);

        Vector3r p_12 = p1 - p2;
        Real dist = p_12.norm();
        Real mag = dist - model.getParticleRadius() * static_cast<Real>(2.0);

        if(mag < static_cast<Real>(0.0)){
          model.m_deltaX[i] -= (mag/dist) * p_12;
          model.m_numConstraints[i] += 1; 
        }
      }
      
      // inter particle collision handling
      for(unsigned int particleIndex : model.m_neighbors[i]){
        Vector3r p1 = model.getParticles().getPosition(i);
        Vector3r p2 = model.getParticles().getPosition(particleIndex);

        Vector3r p_12 = p1 - p2;
        Real dist = p_12.norm();
        Real mag = dist - model.getParticleRadius() * static_cast<Real>(2.0);

        if(mag < static_cast<Real>(0.0)){
          model.m_deltaX[i] -= static_cast<Real>(0.5) * (mag/dist) * p_12;
          model.m_deltaX[particleIndex] += static_cast<Real>(0.5) * (mag/dist) * p_12;
          model.m_numConstraints[i] += 1;
          model.m_numConstraints[particleIndex] += 1;
        }
      }

    }
    
    for(unsigned int i = 0; i < pd.size(); ++i){
      if(model.m_numConstraints[i] == 0){
        model.m_particles.m_x[i] += model.m_deltaX[i]; 
      }else{
        model.m_particles.m_x[i] += model.m_deltaX[i]/model.m_numConstraints[i];
      }
    }

    iter++;
  }
  

  
  
  // ----- Boundary collisions-----------------
  // for each particle check if it is colliding with a boundary particle
  // if it is colliding, solve constraint and update num constraints for particle
  
  // unsigned int **neighbors = model.getNeighborhoodSearch().g
}
