#include "TimeStep.hpp"
#include "entities/Simulation.hpp"

using namespace PBD;

TimeStep::TimeStep(){}

TimeStep::~TimeStep(){}

void TimeStep::clearAcceleration(SimulationModel &model){
  ParticleData &pd = model.getParticles(); 
  Simulation *sim = Simulation::getCurrent();
  const Vector3r grav(0.0,-9.81,0.0);
  const unsigned int count = pd.size();
  for(unsigned int i = 0; i < count; ++i){
    if(pd.getMass(i) != 0.0){
      Vector3r &a = pd.getAcceleration(i);
      a = grav;
    }
  }
}


