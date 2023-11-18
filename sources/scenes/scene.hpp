#include "utils/Common.hpp"
#include "utils/CompactNSearch.h"

struct Scene {
  Scene();
  ~Scene();

  void AddWall(const Vector3r &minX, const Vector3r &maxX,
               std::vector<Vector3r> &boundaryParticles);
  void CreateDam();
};
