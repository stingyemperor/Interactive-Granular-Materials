#include "TimeIntegration.hpp"

using namespace PBD;


void TimeIntegration::semiImplicitEuler(const Real h, const Real mass, Vector3r &position, 
                                        Vector3r &velocity, const Vector3r &acceleration){

  if(mass != 0.0){
    velocity += acceleration * h;
    position += velocity * h;
  }
}
