#pragma once
#include "../utils/Common.hpp"

namespace PBD {
  struct TimeIntegration{
    static void semiImplicitEuler(const Real h, const Real mass, Vector3r &position, Vector3r &velocity, const Vector3r &acceleration);
  };
}
