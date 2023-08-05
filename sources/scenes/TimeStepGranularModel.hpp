#pragma once
#include "GranularModel.hpp"

namespace PBD {
  struct TimeStepGranularModel{
    void clearAccelerations(GranularModel &granularModel); 
    void constraintProjection(GranularModel &granularModel);

    TimeStepGranularModel(){};
    ~TimeStepGranularModel(){};

    void step(GranularModel &model,CompactNSearch::NeighborhoodSearch &nsearch);
    void boundaryConstarint(GranularModel &model, const unsigned int p1, const unsigned int p2);
    void reset();
    
  };
}
