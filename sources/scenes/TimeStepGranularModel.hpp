#pragma once
#include "GranularModel.hpp"

namespace PBD {
  struct TimeStepGranularModel{
    void clearAccelerations(GranularModel &granularModel); 
    void constraintProjection(GranularModel &granularModel);

    TimeStepGranularModel(){};
    ~TimeStepGranularModel(){};

    void step(GranularModel &model,CompactNSearch::NeighborhoodSearch &nsearch);
    void boundaryConstraint(GranularModel &model, const unsigned int x1,  const unsigned int x2);
    void contactConstraint(GranularModel &model, const unsigned int x1, const unsigned int x2);
    void reset();
    
  };
}
