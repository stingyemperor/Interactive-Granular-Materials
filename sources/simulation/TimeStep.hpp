#pragma once
#include "SimulationModel.hpp"

namespace PBD {
  struct TimeStep{
    void clearAcceleration(SimulationModel &model);
    TimeStep();
    virtual ~TimeStep();
    virtual void step(SimulationModel &model) = 0;
    virtual void reset(){};

    virtual void init(){};
  };
}
