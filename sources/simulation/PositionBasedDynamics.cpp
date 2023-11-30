#include "PositionBasedDynamics.hpp"
#include "utils/Common.hpp"

using namespace PBD;


bool PositionBasedDynamics::solveContactConstraint(const Vector3r &p0, 
  Real invMass0, const Vector3r &p1,Real invMass1, const Real restLength, 
  const Real stiffness, Vector3r &corr0, Vector3r &corr1){
    
  Real wSum = invMass0 + invMass1;
  if(wSum == 0.0){
    return false;
  }

  Vector3r n = p1 - p0;
  Real d = n.norm();
  n.normalize();

  Vector3r corr; 
  corr = stiffness * n * (d - restLength)/wSum; 

  corr0 =  invMass0 *  corr;
  corr1 = -invMass1 *  corr;
  return true;
}
