#pragma once
#include "ParticleData.hpp"
#include "utils/CompactNSearch.h"

struct ContactConstraint{
  
  std::array<unsigned int, max_count> particles;
  unsigned int num_particles{0};
  void generateContacts(std::array<std::vector<unsigned int>, max_count> neighbors);
  void solve();

};
