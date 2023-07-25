#include "entities/ParticleGenerator.hpp"
#include "entities/ParticleUpdater.hpp"
#include "raylib.h"
#include "iostream"
#include "entities/ParticleSystem.hpp"
#include "utils/CompactNSearch.h"
#include "utils/PointSet.h"
#include <array>
#include <memory>
#include <entities/Plane.hpp>
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
  camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
  camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;                                // Camera field-of-view Y
  camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type


  DisableCursor();                    // Limit cursor to relative movement inside the window

  SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------


  // ------------ Particle Stuff --------------------------------------

  ParticleSystem particle_system;
  BoxPosGen box_pos_gen;
  Simulation simulation;
  box_pos_gen.generate(&particle_system.particles);
  // std::cout<< GetWorkingDirectory() << std::endl;
  // TODO change the radius for search
  CompactNSearch::NeighborhoodSearch nsearch(1.0f);
  Plane plane;


  // std::vector<std::array<float , 3>> pl;
  // for (int i = 0; i < 100; ++i){
  //   pl.push_back(std::array<float, 3>{0.0f,0.0f,0.0f});
  // }

  // int pointID = nsearch.add_point_set(pl.front().data(), pl.size());
  // nsearch.find_neighbors();

  


  // ParticleSystem particleSystem(1000);
  //   ParticleEmitter particleEmitter;
  // EulerUpdater eulerUpdater;
  // FloorUpdater floorUpdater;
  // WindUpdater windUpdater;
  // CompactNSearch::NeighborhoodSearch nsearch(1.0f);
  // Constraints constraints;
  //
  // // make shared pointers for the generators
  // std::shared_ptr<BoxPosGen> boxGenPtr = std::make_shared<BoxPosGen>();
  // std::shared_ptr<BasicVelGen> basicVelGenPtr = std::make_shared<BasicVelGen>();
  // std::shared_ptr<ParticleEmitter> particleEmitterPtr= std::make_shared<ParticleEmitter>();
  // std::shared_ptr<EulerUpdater> eulerUpdaterPtr= std::make_shared<EulerUpdater>();
  // std::shared_ptr<FloorUpdater> floorUpdaterPtr= std::make_shared<FloorUpdater>();
  // std::shared_ptr<WindUpdater> windUpdaterPtr= std::make_shared<WindUpdater>();
  // std::shared_ptr<Constraints>constraintsPtr= std::make_shared<Constraints>();
  // // Decalre the generators
  // BoxPosGen boxGen;
  // BasicVelGen basicVelGen;
  // *boxGenPtr = boxGen;
  // *basicVelGenPtr = basicVelGen;
  // //Add generators to the emitter
  // particleEmitter.addGenerator(boxGenPtr);
  // // particleEmitter.addGenerator(basicVelGenPtr);
  // // add emitter to system
  // *particleEmitterPtr = particleEmitter;
  // particleSystem.addEmitter(particleEmitterPtr);
  // // add updater to system
  // *eulerUpdaterPtr = eulerUpdater;
  // *floorUpdaterPtr = floorUpdater;
  // particleSystem.addUpdater(eulerUpdaterPtr);
  // particleSystem.addUpdater(floorUpdaterPtr);
  // // particleSystem.addUpdater(windUpdaterPtr);
  // particleSystem.init();
  // // constraints.init(&particleSystem.m_particles);
  // constraints.setPointSet(nsearch,&particleSystem.m_particles);

  // ------------ Particle Stuff --------------------------------------
  // Main game loop
  while (!WindowShouldClose())        // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    UpdateCamera(&camera, CAMERA_FREE);

    if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(DARKGRAY);

    BeginMode3D(camera);
    BeginShaderMode(alpha);

    particle_system.draw(camera,sphere);
    // std::cout << GetFrameTime() << std::endl;
    simulation.simulate(&particle_system.particles, 0.01f, nsearch,plane);
    plane.draw();
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
