#include "SimulationModel.hpp"

using namespace PBD;

SimulationModel::SimulationModel(){

}

ParticleData &SimulationModel::getParticles(){
  return m_particles;
}

void SimulationModel::cleanUp(){
  resetContacts();
  m_constraints.clear();
  m_particles.release();
}

void SimulationModel::reset(){
  for(int i = 0; i < m_particles.size(); ++i){
    const Vector3r& x0 = m_particles.getPosition0(i);
    m_particles.setPosition(i, x0);
    m_particles.getVelocity(i).setZero();
    m_particles.getAcceleration(i).setZero();
    updatedConstraints();
  }
}

SimulationModel::ConstraintVector & SimulationModel::getConstraints(){
  return m_constraints;
}

void SimulationModel::updatedConstraints(){
  for(unsigned int i = 0; i < m_constraints.size(); ++i){
    // TODO fix 
    // m_constraints[i]->updatedConstraints(*this);
  }
}

void SimulationModel::resetContacts(){

}
