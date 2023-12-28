#include "iostream"
#include "random"
#include "raylib.h"
#include "sampling/OBJLoader.hpp"
#include "sampling/SurfaceSampler.hpp"
#include "sampling/VolumeSampler.hpp"
#include "scenes/GranularModel.hpp"
#include "scenes/Scene.hpp"
#include "scenes/TimeStepGranularModel.hpp"
#include "simulation/Simulation.hpp"
#include "spdlog/spdlog.h"
#include "utils/Common.hpp"
#include "utils/CompactNSearch.h"
#include <benchmark/benchmark.h>
#include <chrono>
#include <eigen3/Eigen/src/Core/GlobalFunctions.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <fstream>
#include <vector>

using namespace PBD;
using namespace Utilities;
using namespace CompactNSearch;

void timeStep();
void buildModel();
void createBreakingDam(NeighborhoodSearch &nsearch);
void addWall(const Vector3r &minX, const Vector3r &maxX,
             std::vector<Vector3r> &boundaryParticles);
void addWallHCP(const unsigned int heigth, unsigned int width,
                unsigned int depth, std::vector<Vector3r> &boundaryParticles);
void initBoundaryData(std::vector<Vector3r> &boundaryParticles);
void createPointSet(NeighborhoodSearch &nsearch);
void render();
void cleanUp();
void reset();

GranularModel model;
TimeStepGranularModel simulation;
NeighborhoodSearch nsearch(0.075, true);
Scene scene;

const Real particleRadius = static_cast<Real>(0.025);
const Real particleRadiusUpsampled = 0.5 * particleRadius;
const unsigned int width = 15;
const unsigned int depth = 20;
const unsigned int height = 20;
const Real containerWidth =
    (width + 1) * particleRadius * static_cast<Real>(2.0 * 3.0);
// const Real containerWidth =
//     (width + 1) * particleRadius * static_cast<Real>(2.0 * 5.0);
const Real containerDepth =
    (depth + 1) * particleRadius * static_cast<Real>(2.0 * 3.0);
const Real containerHeight = 4.0;

double currentTime = 0.0;
double previousTime = GetTime();
float deltaTime = 0.0;
bool isPause = true;
float timer = 0;
int direction = -1;
Vector3r translateBoundary(0.1, 0.0, 0.0);
Vector3r rotationAxis(0, 1, 0);
//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
  // Initialization
  //--------------------------------------------------------------------------------------
  // directory is relative to build folder
  Window window;
  Texture2D sphere = LoadTexture("../assets/sphere.png");
  Shader alpha = LoadShader(NULL, "../assets/depth.fs");
  // Define the camera to look into our 3d world
  Camera3D camera = {0};
  camera.position = (Vector3){0.0f, 4.0f, 4.0f}; // Camera position
  camera.target = (Vector3){0.0f, 0.0f, 0.0f};   // Camera looking at point
  camera.up =
      (Vector3){0.0f, 1.0f, 0.0f}; // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;             // Camera field-of-view Y
  camera.projection = CAMERA_PERSPECTIVE; // Camera projection type
  // std::ofstream file("energy.csv");
  // std::ofstream file("timeMerge.csv");
  std::ofstream file("timeBaseParallel.csv");

  // ---------------------Random for sphere sampling ----------------------
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  DisableCursor(); // Limit cursor to relative movement inside the window

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  // ------------ Particle Stuff --------------------------------------
  createBreakingDam(nsearch);
  // ------------ Particle Stuff --------------------------------------
  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    // UpdateCamera(&camera, CAMERA_FREE);
    if (IsKeyPressed(KEY_SPACE)) {
      isPause = !isPause;
    }
    if (IsKeyDown('Z'))
      camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    //----------------------------------------------------------------------------------
    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(DARKGRAY);

    DrawText(TextFormat("CURRENT FPS: %i", static_cast<int>(1.0f / deltaTime)),
             GetScreenWidth() - 220, 40, 20, GREEN);
    DrawText(TextFormat("Particles: %i", model.m_numActiveParticles),
             GetScreenWidth() - 220, 60, 20, GREEN);
    DrawText(TextFormat("Merging: %i", model.m_numMerging),
             GetScreenWidth() - 220, 80, 20, GREEN);
    DrawText(TextFormat("Splitting: %i", model.m_numSplitting),
             GetScreenWidth() - 220, 100, 20, GREEN);

    BeginMode3D(camera);
    BeginShaderMode(alpha);

    DrawGrid(16, 0.5);

    for (unsigned int i = 0; i < model.m_numActiveParticles; ++i) {
      Vector3r particle_pos = model.getParticles().getPosition(i);
      Vector3 pos = {static_cast<float>(particle_pos.x()),
                     static_cast<float>(particle_pos.y()),
                     static_cast<float>(particle_pos.z())};

      // if(model.m_isBoundary[i] == true){
      //   DrawBillboard(camera, sphere, pos,
      //   model.m_particles.getRadius(i)*2.0, BLACK);
      // }else{
      //   DrawBillboard(camera, sphere, pos,
      //   model.m_particles.getRadius(i)*2.0, WHITE);
      // }
      DrawBillboard(camera, sphere, pos, model.m_particles.getRadius(i) * 2.0,
                    WHITE);
    }
    // for (unsigned int i = 0; i < model.m_upsampledParticlesX.size(); ++i) {
    //   if (model.m_isActiveUpsampled[i]) {
    //     Vector3r particle_pos = model.getUpsampledX(i);
    //     Vector3 pos = {(float)particle_pos.x(), (float)particle_pos.y(),
    //                    (float)particle_pos.z()};
    //     DrawBillboard(camera, sphere, pos,
    //                   2.0 * model.getParticleRadiusUpsampled(), WHITE);
    //   }
    // }
    //
    // std::cout << GetFPS()  << "\n";
    // std::cout << GetFrameTime() << "\n";
    if (!isPause) {
      auto begin = std::chrono::high_resolution_clock::now();
      simulation.step(model, nsearch, uniform, generator);
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
      // file << elapsed.count() << "\n";

      // if(IsKeyDown('P')){
      // }

      // move boundary
      timer += deltaTime;

      for (unsigned int i = 0; i < model.m_boundaryX.size(); ++i) {
        // model.m_boundaryX[i] 0+= direction * translateBoundary * deltaTime;

        Eigen::AngleAxisd rotation(M_PI / 16 * deltaTime, rotationAxis);
        model.m_boundaryX[i] = rotation * model.m_boundaryX[i];
      }
      if (timer > 8) {
        timer = 0;
        direction = -1 * direction;
      }
      if (IsKeyDown('B')) {
        for (unsigned int i = 0; i < model.m_boundaryX.size(); ++i) {
          Vector3r particle_pos = model.m_boundaryX[i];
          Vector3 pos = {static_cast<float>(particle_pos.x()),
                         static_cast<float>(particle_pos.y()),
                         static_cast<float>(particle_pos.z())};
          // DrawSphereWires(pos, 0.015, 6, 6, WHITE);
          DrawBillboard(camera, sphere, pos, 0.015 * 2.0, BLACK);
        }
      }

      // for (unsigned int i = 0; i < model.m_upsampledParticlesX.size(); ++i) {
      //   if (model.m_isActiveUpsampled[i]) {
      //     Vector3r particle_pos = model.getUpsampledX(i);
      //     Vector3 pos = {(float)particle_pos.x(), (float)particle_pos.y(),
      //                    (float)particle_pos.z()};
      //     DrawBillboard(camera, sphere, pos,
      //                   2.0 * model.getParticleRadiusUpsampled(), WHITE);
      //   }
      // }
      // std::cout << model.m_particles.size() << "\n";
      // std::cout << model.m_inactiveUpsampled.size() << "\n";

      currentTime = GetTime();
      deltaTime = static_cast<float>(currentTime - previousTime);
      previousTime = currentTime;
      if (IsKeyDown('F')) {
        simulation.applyForce(model);
      }
    }
    // simulation.calculateAverageEnergy(model, file);

    // DrawGrid(6, 6);
    EndShaderMode();
    EndMode3D();

    EndDrawing();
    //----------------------------------------------------------------------------------
  }
  window.close();
  file.close();
  return 0;
}

void createBreakingDam(NeighborhoodSearch &nsearch) {
  std::vector<Vector3r> granularParticles;
  scene.BreakingDam(granularParticles, 15, 20, 20, 0.025);

  // boudary particle stuff
  std::vector<Vector3r> boundaryParticles;
  Vector3r translationBoundary(0.0, -0.2, 0.0);
  scene.SampleFromMesh("../assets/box.obj", boundaryParticles,
                       translationBoundary, 0.020);

  initBoundaryData(boundaryParticles);
  // updsampled particle init
  std::random_device rd;  // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::vector<Vector3r> upsampledParticles;
  Vector3r radius(0.05, 0.05, 0.05);

  for (Vector3r center : granularParticles) {
    Vector3r boundsMin = center - radius;
    Vector3r boundsMax = center + radius;
    std::uniform_real_distribution<> distrX(boundsMin.x(), boundsMax.x());
    std::uniform_real_distribution<> distrY(boundsMin.y(), boundsMax.y());
    std::uniform_real_distribution<> distrZ(boundsMin.z(), boundsMax.z());

    for (int i = 0; i < 10; ++i) {
      Vector3r vec(distrX(gen), distrY(gen), distrZ(gen));
      upsampledParticles.push_back(vec);
    }
  }

  // init model stuff
  model.initModel(
      (unsigned int)granularParticles.size(), granularParticles.data(),
      (unsigned int)boundaryParticles.size(), boundaryParticles.data(),
      (unsigned int)upsampledParticles.size(), upsampledParticles.data());

  createPointSet(nsearch);
}

void addWall(const Vector3r &minX, const Vector3r &maxX,
             std::vector<Vector3r> &boundaryParticles) {
  const Real particleDistance =
      static_cast<Real>(2.0) * model.getParticleRadius();
  const Vector3r diff = maxX - minX;
  const unsigned int stepsX = (unsigned int)(diff[0] / particleDistance) + 1u;
  const unsigned int stepsY = (unsigned int)(diff[1] / particleDistance) + 1u;
  const unsigned int stepsZ = (unsigned int)(diff[2] / particleDistance) + 1u;

  const unsigned int startIndex = (unsigned int)boundaryParticles.size();
  boundaryParticles.resize(startIndex + stepsX * stepsY * stepsZ);

  for (int x = 0; x < static_cast<int>(stepsX); ++x) {
    for (int y = 0; y < static_cast<int>(stepsY); ++y) {
      for (int z = 0; z < static_cast<int>(stepsZ); ++z) {
        const Vector3r currPos =
            minX + Vector3r(x * particleDistance, y * particleDistance,
                            z * particleDistance);
        boundaryParticles[startIndex + x * stepsY * stepsZ + y * stepsZ + z] =
            currPos;
      }
    }
  }
}

void addWallHCP(const unsigned int width, const unsigned int height,
                const unsigned int depth,
                std::vector<Vector3r> &boundaryParticles) {

  double diam = 2.0 * 0.025;
  double offset = diam * sqrt(2.0 / 3.0);
  for (unsigned int i = 0; i < width; ++i) {
    for (unsigned int j = 0; j < height; ++j) {
      for (unsigned int k = 0; k < depth; ++k) {
        boundaryParticles[i + j + k] =
            Vector3r(i * diam, j * (i % 2) * (offset / 2.0), k * diam);
      }
    }
  }
}

void addFloor(const int size, std::vector<Vector3r> &boundaryParticles) {
  Real diam = 0.025 * 2.0;
  Real halfSize = size / 2.0;

  for (unsigned int i = -halfSize; i < halfSize; ++i) {
    for (unsigned int j = -halfSize; j < halfSize; ++j) {
      boundaryParticles.push_back(Vector3r(i * diam, j * diam, 0));
    }
  }
}
void initBoundaryData(std::vector<Vector3r> &boundaryParticles) {
  const Real x1 = -containerWidth / 2.0;
  const Real x2 = containerWidth / 2.0;
  const Real y1 = 0.0;
  const Real y2 = containerHeight;
  const Real z1 = -containerDepth / 2.0;
  const Real z2 = containerDepth / 2.0;

  const Real diam = 2.0 * particleRadius;

  // Floor
  // addWall(Vector3r(x1, y1, z1), Vector3r(x2, y1, z2), boundaryParticles);
  // addWall(Vector3r(x1, y1, z1), Vector3r(x2, y1, z2), boundaryParticles);
  // Top
  // addWall(Vector3r(x1, y2, z1), Vector3r(x2, y2, z2), boundaryParticles);
  // Left
  // addWall(Vector3r(x1, y1, z1), Vector3r(x1, y2, z2), boundaryParticles);
  // Right
  // addWall(Vector3r(x2, y1, z1), Vector3r(x2, y2, z2), boundaryParticles);
  // Back
  // addWall(Vector3r(x1, y1, z1), Vector3r(x2, y2, z1), boundaryParticles);
  // Front
  // addWall(Vector3r(x1, y1, z2), Vector3r(x2, y2, z2), boundaryParticles);
}

void createPointSet(NeighborhoodSearch &nsearch) {
  model.m_pointIdLR = nsearch.add_point_set(
      model.m_particles.getPosition(0).data(), model.m_particles.size());
  model.m_pointIdBoundary = nsearch.add_point_set(
      model.m_boundaryX.front().data(), model.m_boundaryX.size());
  model.m_pointIdUpsampled =
      nsearch.add_point_set(model.m_upsampledParticlesX.front().data(),
                            model.m_upsampledParticlesX.size());
}
