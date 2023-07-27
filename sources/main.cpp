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
  CompactNSearch::NeighborhoodSearch nsearch(0.6f);
  Plane plane;


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
