#include "Simulation.hpp"
#include "TimeManager.hpp"

using namespace PBD;

Simulation *Simulation::current = nullptr;
int Simulation::GRAVITAION = -1;

Simulation::Simulation() {
  m_gravitation = Vector3r(0.0, -9.81, 0.0);
  m_timeStep = nullptr;
  m_model = nullptr;
}

Simulation::~Simulation() {
  delete m_timeStep;
  delete TimeManager::getCurrent();
  current = nullptr;
}

Simulation *Simulation::getCurrent() {
  if (current == nullptr) {
    current = new Simulation();
  }
  return current;
}

void Simulation::setCurrent(Simulation *tm) { current = tm; }

bool Simulation::hasCurrent() { return (current != nullptr); }

// TODO init()
void Simulation::reset() {
  m_model->reset();
  if (m_timeStep) {
    m_timeStep->reset();
  }
}
