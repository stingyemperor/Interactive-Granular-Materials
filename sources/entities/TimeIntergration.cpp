#include "TimeIntegration.hpp"

using namespace PBD;


void TimeIntegration::semiImplicitEuler(const Real h, const Real mass, Vector3r &position, 
                                        Vector3r &velocity, const Vector3r &acceleration){

  if(mass != 0.0){
    velocity += acceleration * h;
    position += velocity * h;
  }
}

void TimeIntegration::VelocityUpdateFirstOrder(const Real h, const Real mass, const Vector3r &position, const Vector3r &oldPosition, Vector3r &velocity){
  if(mass != 0.0){
    velocity  = (1.0/h) * (position - oldPosition);
  }
}
