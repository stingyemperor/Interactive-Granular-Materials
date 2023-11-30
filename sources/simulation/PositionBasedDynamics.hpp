#pragma once
#include "../utils/Common.hpp"

namespace PBD{

  struct PositionBasedDynamics{
    
    static bool solveContactConstraint(
      const Vector3r &p0, Real invMass0, const Vector3r &p1, Real invMass1, const Real restLength, 
      const Real stiffness, Vector3r &corr0, Vector3r &corr1);
  };
}
