#pragma once
#include <vector>
#include "../utils/Common.hpp"
#include <array>
#include "entities/SimulationModel.hpp"
#include "math.h"

#define _USE_MATH_DEFINES

namespace PBD{
  struct SimulationModel;

  struct Constraint{
    std::vector<unsigned int> m_bodies;

    Constraint(const unsigned int numberOfBodies){
      m_bodies.resize(numberOfBodies);
    }

    unsigned int numberOfBodies() const {return static_cast<unsigned int>(m_bodies.size());}
    virtual ~Constraint(){};
    virtual bool initConstraintsBeforeProjection(SimulationModel &model){return true;}
    virtual bool updateConstraints(SimulationModel &model){return true;}
    virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter){return true;}
  };

  struct ContactConstraint : public Constraint{
    Real m_stiffness;
    Real m_restLength;

    ContactConstraint() : Constraint(2) {};
    virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2, const Real stiffness);
    virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
  };
}
