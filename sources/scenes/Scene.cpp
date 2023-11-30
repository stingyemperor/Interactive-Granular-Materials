#include "scenes/Scene.hpp"
#include "sampling/OBJLoader.hpp"
#include "sampling/VolumeSampler.hpp"
#include "scenes/GranularModel.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>

void Scene::SampleFromMesh(const std::string &name,
                           std::vector<Vector3r> &granularParticles,
                           const Vector3r &translation,
                           const PBD::GranularModel &model) {
  Matrix3rx vertices;
  Matrix3ix indices;
  Matrix3rx normals;

  OBJLoader::loadObj(name, vertices, indices, normals);
  std::vector<Vector3r> particles = VolumeSampler::sampleMeshRandom(
      vertices, indices, model.getParticleRadius());

  granularParticles.resize(particles.size());
  for (int i = 0; i < particles.size(); ++i) {
    granularParticles[i] = particles[i] + translation;
  }
}
