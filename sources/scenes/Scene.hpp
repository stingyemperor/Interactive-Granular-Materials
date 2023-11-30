#include "GranularModel.hpp"
#include "raylib.h"
#include "sampling/OBJLoader.hpp"
#include "sampling/VolumeSampler.hpp"
#include "utils/Common.hpp"
#include <string>
#include <vector>

struct Scene {
  Scene() {}
  ~Scene() {}

  /**
   * @brief Samples a rectangular grid of particles
   *
   * @param minX
   * @param maxX
   * @param boundaryParticles
   */
  void AddWall(const Vector3r &minX, const Vector3r &maxX,
               std::vector<Vector3r> &boundaryParticles);
  /**
   * @brief Generate the starting configuration for the particles from a mesh
   * and translate it from origin
   *
   * @param name
   * @param granularParticles
   */
  void SampleFromMesh(const std::string &name,
                      std::vector<Vector3r> &granularParticles,
                      const Vector3r &translation,
                      const PBD::GranularModel &model);
};

/**
 * @class Window
 * @brief Sets the properties of the display window as well as opens and closes
 * the window
 *
 */
struct Window {
  unsigned int m_width;
  unsigned int m_height;

  Window(unsigned int width, unsigned int height) {
    this->m_width = width;
    this->m_height = height;
    InitWindow(width, height, "Granular");
  }

  void close() { CloseWindow(); }
  ~Window() = default;
};
