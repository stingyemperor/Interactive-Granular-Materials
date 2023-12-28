#include "scenes/Scene.hpp"
#include "sampling/OBJLoader.hpp"
#include "sampling/VolumeSampler.hpp"
#include "scenes/GranularModel.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>

void Scene::SampleFromMesh(const std::string &name,
                           std::vector<Vector3r> &granularParticles,
                           const Vector3r &translation, const Real radius) {
  Matrix3rx vertices;
  Matrix3ix indices;
  Matrix3rx normals;

  OBJLoader::loadObj(name, vertices, indices, normals);
  std::vector<Vector3r> particles =
      VolumeSampler::sampleMeshRandom(vertices, indices, radius);

  granularParticles.resize(particles.size());
  for (int i = 0; i < particles.size(); ++i) {
    granularParticles[i] = particles[i] + translation;
  }
}

void Scene::BreakingDam(std::vector<Vector3r> &granularParticles,
                        const unsigned int width, const unsigned int height,
                        const unsigned int depth, const Real particleRadius) {
  Real diam = 2.0 * particleRadius;
  const Real containerWidth =
      (width + 1) * particleRadius * static_cast<Real>(2.0 * 3.0);
  const Real containerDepth =
      (depth + 1) * particleRadius * static_cast<Real>(2.0 * 3.0);

  const Real startX = -static_cast<Real>(0.5) * containerWidth + diam;
  const Real startY = diam * 5.0;
  const Real startZ = -static_cast<Real>(0.5) * containerDepth + diam;
  Vector3r startPos(startX, startY, startZ);

  granularParticles.resize(width * height * depth);
  for (unsigned int i = 0; i < static_cast<int>(width); ++i) {
    for (unsigned int j = 0; j < static_cast<int>(height); ++j) {
      for (unsigned int k = 0; k < static_cast<int>(depth); ++k) {
        granularParticles[i * height * depth + j * depth + k] =
            diam * Vector3r(i, j, k) + startPos;
      }
    }
  }
}
