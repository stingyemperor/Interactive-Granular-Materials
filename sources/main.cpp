#include "raylib.h"
#include "iostream"
#include <array>
#include <memory>
#include <random>
#include <vector>
#include "utils/Common.hpp"
#include "scenes/GranularModel.hpp"
#include "scenes/TimeStepGranularModel.hpp"
#include "entities/Simulation.hpp"
#include "utils/CompactNSearch.h"
#include "random"

using namespace PBD;
using namespace Utilities;
using namespace CompactNSearch;

void timeStep();
void buildModel();
void createBreakingDam(NeighborhoodSearch &nsearch);
void addWall(const Vector3r &minX, const Vector3r &maxX, std::vector<Vector3r>&boundaryParticles);
void initBoundaryData(std::vector<Vector3r> &boundaryParticles);
void createPointSet(NeighborhoodSearch &nsearch);
void render();
void cleanUp();
void reset();



GranularModel model;
TimeStepGranularModel simulation;
NeighborhoodSearch nsearch(0.075);

const Real particleRadius = static_cast<Real>(0.025);
const Real particleRadiusUpsampled = 0.5 * particleRadius;
const unsigned int width = 15;
const unsigned int depth = 15;
const unsigned int height = 20;
const Real containerWidth = (width + 1)*particleRadius*static_cast<Real>(2.0*5.0);
const Real containerDepth = (depth + 1)*particleRadius*static_cast<Real>(2.0);
const Real containerHeight = 4.0;


//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 1920;
  const int screenHeight = 1080;
  InitWindow(screenWidth, screenHeight, "Granular");
  // directory is relative to build folder
  Texture2D sphere = LoadTexture("../assets/sphere.png");
  Shader alpha = LoadShader(NULL,"../assets/depth.fs");
  // Define the camera to look into our 3d world
  Camera3D camera = { 0 };
  camera.position = (Vector3){ 0.0f, 4.0f, 4.0f }; // Camera position
  camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;                                // Camera field-of-view Y
  camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type


  DisableCursor();                    // Limit cursor to relative movement inside the window

  SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------


  // ------------ Particle Stuff --------------------------------------

  // ParticleSystem particle_system;
  // BoxPosGen box_pos_gen;
  // Simulation simulation;
  // box_pos_gen.generate(&particle_system.particles);
  // // std::cout<< GetWorkingDirectory() << std::endl;
  // // TODO change the radius for search
  // CompactNSearch::NeighborhoodSearch nsearch(0.5f);
  // Plane plane;

  createBreakingDam(nsearch); 
  // ------------ Particle Stuff --------------------------------------
  // Main game loop
  while (!WindowShouldClose())        // Detect window close button or ESC key
  {

    // Update
    //----------------------------------------------------------------------------------
    // UpdateCamera(&camera, CAMERA_FREE);

    if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(DARKGRAY);

    BeginMode3D(camera);
    BeginShaderMode(alpha);
  
    // std::cout << GetFPS()  << "\n"; 
    // std::cout << GetFrameTime() << "\n";
    simulation.step(model, nsearch);

    if(IsKeyDown('P')){
      for(unsigned int i = 0; i < model.m_particles.size(); ++i){
      Vector3r particle_pos = model.getParticles().getPosition(i); 
      Vector3 pos = {(float)particle_pos.x(),(float)particle_pos.y(),(float)particle_pos.z()}; 
      DrawBillboard(camera, sphere, pos, model.m_particles.getRadius(i)*2.0, WHITE);
      }
    }
   
    // for(unsigned int i = 0; i < model.m_upsampledParticlesX.size(); ++i){
    //   Vector3r particle_pos = model.getUpsampledX(i); 
    //   Vector3 pos = {(float)particle_pos.x(),(float)particle_pos.y(),(float)particle_pos.z()}; 
    //   DrawBillboard(camera, sphere, pos, 2.0*model.getParticleRadiusUpsampled(), WHITE);
    // }


    if(IsKeyDown('F')){
      simulation.applyForce(model);
    }
    // DrawSphere(Vector3{0,1,0}, 1, GREEN);

    // std::cout << model.m_upsampledParticlesX.size() << "\n";
    // for(unsigned int i = 0; i < model.m_boundaryX.size(); ++i){
    //   Vector3r particle_pos = model.getBoundaryX(i);
    //   Vector3 pos = {particle_pos.x(), particle_pos.y(), particle_pos.z()};
    //   DrawSphereWires(pos, 0.025, 3, 3, WHITE);
    // }
    // particle_system.draw(camera,sphere);
    // // std::cout << GetFrameTime() << std::endl;
    // simulation.simulate(&particle_system.particles, 0.01f, nsearch,plane);
    // plane.draw();
    DrawGrid(6,6);
    EndShaderMode();
    EndMode3D();

    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow();        // Close window and OpenGL context
  //--------------------------------------------------------------------------------------

  return 0;
}


void createBreakingDam(NeighborhoodSearch &nsearch){
  const Real diam = 2.0*particleRadius;
  const Real startX = -static_cast<Real>(0.5)*containerWidth + diam;
  const Real startY = diam * 5.0;
  const Real startZ = -static_cast<Real>(0.5)*containerDepth + diam;
  const Real yshift = sqrt(static_cast<Real>(3.0))*particleRadius;

  std::vector<Vector3r> granularParticles;
  granularParticles.resize(width*height*depth);

  for(unsigned int i = 0; i < (int)width; ++i){
    for(unsigned int j = 0; j < (int)height; ++j){
      for(unsigned int k = 0; k < (int)depth; ++k){
        granularParticles[i*height*depth + j*depth + k] = diam*Vector3r((Real)i, (Real)j, (Real)k) + Vector3r(startX, startY, startZ);
      }
    }
  }
   
  model.setParticleRadius(particleRadius);
  model.setParticleRadiusUpsampled(particleRadiusUpsampled);
  

  // boudary particle stuff
  std::vector<Vector3r> boundaryParticles;
  initBoundaryData(boundaryParticles);

  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::vector<Vector3r> upsampledParticles;

  for(Vector3r center : granularParticles){
    Vector3r radius(0.025,0.025,0.025);
    Vector3r boundsMin = center - radius;
    Vector3r boundsMax = center + radius;
    std::uniform_real_distribution<> distrX(boundsMin.x(),boundsMax.x());
    std::uniform_real_distribution<> distrY(boundsMin.y(),boundsMax.y());
    std::uniform_real_distribution<> distrZ(boundsMin.z(),boundsMax.z());

    for(int i = 0; i < 10 ; ++i){
      Vector3r vec(distrX(gen),distrY(gen),distrZ(gen)); 
      upsampledParticles.push_back(vec);
    }
    
  }
  // upsampled particles stuff
  // int width_upsampled = width * 2.0;
  // int height_upsampled = height * 2.0;
  // int depth_upsampled = depth* 2.0;
  // const Real diamUpsampled = 2.0*particleRadiusUpsampled;
  // const Real startXupsampled = -static_cast<Real>(0.5)*containerWidth + diamUpsampled;
  // const Real startYupsampled = diam * 5.0;
  // const Real startZupsampled = -static_cast<Real>(0.5)*containerDepth + diamUpsampled;

  // upsampledParticles.resize(width_upsampled*height_upsampled*depth_upsampled);

  // for(unsigned int i = 0; i < (int)width; ++i){
  //   for(unsigned int j = 0; j < (int)height; ++j){
  //     for(unsigned int k = 0; k < (int)depth; ++k){
  //       upsampledParticles[i*height*depth + j*depth + k] = diam*Vector3r((Real)i, (Real)j, (Real)k) + Vector3r(startX, startY, startZ);
  //     }
  //   }
  // }

  // for(unsigned int i = 0; i < (int)width_upsampled; ++i){
  //   for(unsigned int j = 0; j < (int)height_upsampled; ++j){
  //     for(unsigned int k = 0; k < (int)depth_upsampled; ++k){
  //       upsampledParticles[i*height_upsampled*depth_upsampled + j*depth_upsampled + k] = diamUpsampled*Vector3r((Real)i, (Real)j, 
  //                                                                                 (Real)k) + Vector3r(startXupsampled, 
  //                                                                                                     startYupsampled, 
  //                                                                                                     startZupsampled);
  //     }
  //   }
  // }
  //   
  // init model stuff
  model.initModel((unsigned int)granularParticles.size(), granularParticles.data(), 
                  (unsigned int)boundaryParticles.size(), boundaryParticles.data(),
                  (unsigned int)upsampledParticles.size(), upsampledParticles.data());

  createPointSet(nsearch);
}

void addWall(const Vector3r &minX, const Vector3r &maxX, std::vector<Vector3r> &boundaryParticles ){
  const Real particleDistance = static_cast<Real>(2.0)*model.getParticleRadius();
  const Vector3r diff = maxX - minX;
  const unsigned int stepsX = (unsigned int)(diff[0] / particleDistance) + 1u;
  const unsigned int stepsY = (unsigned int)(diff[1] / particleDistance) + 1u;
  const unsigned int stepsZ = (unsigned int)(diff[2] / particleDistance) + 1u;
  
  const unsigned int startIndex = (unsigned int)boundaryParticles.size();
  boundaryParticles.resize(startIndex + stepsX * stepsY * stepsZ);

  for(int x = 0; x < (int)stepsX; ++x){
    for(int y = 0; y < (int)stepsY; ++y){
      for(int z = 0; z < (int)stepsZ; ++z){
        const Vector3r currPos = minX + Vector3r(x*particleDistance, y*particleDistance, z*particleDistance);    
        boundaryParticles[startIndex + x*stepsY*stepsZ + y*stepsZ + z] = currPos;
      }
    }
  }
}

void initBoundaryData(std::vector<Vector3r> &boundaryParticles){
  const Real x1 = -containerWidth / 2.0;
  const Real x2 =  containerWidth / 2.0;
  const Real y1 = 0.0;
  const Real y2 = containerHeight;
  const Real z1 = -containerDepth / 2.0;
  const Real z2 =  containerDepth / 2.0;

  const Real diam = 2.0*particleRadius;

  // Floor
  addWall(Vector3r(x1,y1,z1),Vector3r(x2,y1,z2),boundaryParticles);
  // Top
  addWall(Vector3r(x1,y2,z1),Vector3r(x2,y2,z2),boundaryParticles);
  // Left
  addWall(Vector3r(x1,y1,z1),Vector3r(x1,y2,z2),boundaryParticles);
  // Right
  addWall(Vector3r(x2,y1,z1),Vector3r(x2,y2,z2),boundaryParticles);
  // Back
  addWall(Vector3r(x1,y1,z1),Vector3r(x2,y2,z1),boundaryParticles);
  // Front
  addWall(Vector3r(x1,y1,z2),Vector3r(x2,y2,z2),boundaryParticles);
}

void createPointSet(NeighborhoodSearch &nsearch){
  model.m_pointId1 = nsearch.add_point_set(model.m_particles.getPosition(0).data(), model.m_particles.size());
  model.m_pointId2 = nsearch.add_point_set(model.m_boundaryX.front().data(), model.m_boundaryX.size());
  model.m_pointId3 = nsearch.add_point_set(model.m_upsampledParticlesX.front().data(), model.m_upsampledParticlesX.size());
}



