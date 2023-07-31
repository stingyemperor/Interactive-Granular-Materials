#pragma once
#include "../utils/Common.hpp"
#include "SimulationModel.hpp"
#include "TimeStep.hpp"

namespace PBD{
  struct Simulation{
    static int GRAVITAION;
    SimulationModel *m_model;
    TimeStep *m_timeStep;
    Vector3r m_gravitation;  

    static Simulation *current;
    Simulation();
    ~Simulation();
    void reset();
    
    static Simulation* getCurrent();
    static void setCurrent(Simulation* tm);
    static bool hasCurrent();
    
    SimulationModel *getModel() {return m_model;} 
    void setModel(SimulationModel *model) {m_model = model;}
    TimeStep *getTimeStep() {return m_timeStep;}
    void setTimeStep(TimeStep *ts) {m_timeStep = ts;}
  };
}

