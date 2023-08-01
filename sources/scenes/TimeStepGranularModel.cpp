#include "TimeStepGranularModel.hpp"
#include "../entities/TimeManager.hpp"
#include "../entities/Simulation.hpp"
#include "entities/ParticleData.hpp"
#include "entities/TimeIntegration.hpp"

using namespace PBD;

void TimeStepGranularModel::step(GranularModel &model){
  
  TimeManager *tm = TimeManager::getCurrent();
  const Real h = tm->getTimeStepSize();
  ParticleData &pd = model.getParticles();

  clearAccelerations(model);

  // TIme intergration
  for(unsigned int i = 0; i < pd.size(); ++i){
    model.getDeltaX(i).setZero();
    // TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
  }
  // Neighborhood search
  // model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0),model.numBoundaryParticles(), &model.getBoundaryX(0));

  // model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0));
  // model.getCompactNSearch()->find_neighbors();
  // Constraint Projection


  // Update velocity


  // compute new time
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
  const unsigned int maxIter = 5;
  unsigned int iter = 0;

  ParticleData &pd = model.getParticles();
  // unsigned int **neighbors = model.getNeighborhoodSearch().g
}
