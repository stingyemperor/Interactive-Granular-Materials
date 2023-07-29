#include "Constraints.hpp"

using namespace PBD;

bool ContactConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2, const Real stiffness){
  m_stiffness = stiffness;
  m_bodies[0] = particle1;
  m_bodies[1] = particle2;
  ParticleData &pd = model.getParticles();
  
  const Vector3r &x1_0 = pd.getPosition(particle1); 
  const Vector3r &x2_0 = pd.getPosition(particle2); 
  m_restLength = (x2_0 - x1_0).norm();
  
  return true;
}

bool ContactConstraint::solvePositionConstraint(SimulationModel &model, const unsigned int iter){
  ParticleData &pd = model.getParticles();

  const unsigned int i1 = m_bodies[0];
  const unsigned int i2 = m_bodies[1];
  
  Vector3r &x1 = pd.getPosition(i1);
  Vector3r &x2 = pd.getPosition(i2);

  const Real invMass1 = pd.getInvMass(i1);
  const Real invMass2 = pd.getInvMass(i2);

  Vector3r corr1, corr2;
  return true;
}
