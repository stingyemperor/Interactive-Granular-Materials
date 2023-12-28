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
   * @brief Samples particle from an input mesh
   *
   * @param name Path to the obj file
   * @param granularParticles
   * @param translation
   * @param radius
   */
  void SampleFromMesh(const std::string &name,
                      std::vector<Vector3r> &granularParticles,
                      const Vector3r &translation, const Real radius);

  void BreakingDam(std::vector<Vector3r> &granularParticles,
                   const unsigned int width, const unsigned int height,
                   const unsigned int depth, const Real particleRadius);
};

/**
 * @class Window
 * @brief Sets the properties of the display window as well as opens and closes
 * the window
 *
 */
struct Window {
  int m_width;
  int m_height;

  Window() {
    this->m_width = GetScreenWidth();
    this->m_height = GetScreenHeight();
    InitWindow(this->m_width, this->m_height, "Granular");
  }
  /**
   * @brief Initilaizes the window
   *
   * @param width
   * @param height
   */
  Window(unsigned int width, unsigned int height) {
    this->m_width = width;
    this->m_height = height;
    InitWindow(width, height, "Granular");
  }

  void close() { CloseWindow(); }
  ~Window() = default;
};
