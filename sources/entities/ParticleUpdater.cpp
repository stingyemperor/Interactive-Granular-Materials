#include "ParticleUpdater.hpp"
#include "entities/ParticleData.hpp"
#include "utils/CompactNSearch.h"
#include "utils/PointSet.h"
// #include "entities/ParticleData.hpp"
// #include "utils/CompactNSearch.h"
// #include "utils/PointSet.h"
// #include <glm/common.hpp>
// #include <glm/exponential.hpp>
#include <array>
#include <vector>


void Simulation::resetNeighbor(CompactNSearch::NeighborhoodSearch &nsearch){
  nsearch.find_neighbors();
}

void Simulation::setPointSetTest(CompactNSearch::NeighborhoodSearch &nsearch, std::vector<std::array<float,3>> &pos){
  point_id = nsearch.add_point_set(pos.front().data(), pos.size());
}

void Simulation::setPointSet(CompactNSearch::NeighborhoodSearch &nsearch, std::array<glm::vec3, max_count> &positions){
  // float* flat_array = static_cast<float*>(glm::value_ptr(positions.data());
  float *flat_array = &positions[0].x;
  point_id = nsearch.add_point_set(flat_array, positions.size());
}


void Simulation::getNeighbors(CompactNSearch::NeighborhoodSearch &nsearch,std::array<std::vector<unsigned int>,max_count> &neighbors){
  resetNeighbor(nsearch);
  CompactNSearch::PointSet const& ps_1 = nsearch.point_set(point_id);

  for(unsigned int i = 0; i < ps_1.n_points(); ++i){
    for(unsigned int j = 0; j < ps_1.n_neighbors(point_id, i); ++j){
      neighbors[i].push_back(ps_1.neighbor(point_id, i, j));
    }
  }
}


void Simulation::printVector(glm::vec3 &pos){
  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  std::cout << "pos x: " << x << "\n" << "pos y: " << y << "\n" << "pos z: " << z << "\n";
}

void Simulation::simulate(ParticleData *p, float dt, CompactNSearch::NeighborhoodSearch &nsearch, Plane &plane){

  float radius = p->radius;
  // The maxmimum number of particles
  unsigned int num_particles = max_count;
  // gravity force
  glm::vec3 force_ext(0.0f,-5.0f,0.0f);
  // Number of solver iterations
  // TODO make this a member of the class
  unsigned int solver_i = 3;

  std::vector<std::array<float , 3>> pl;
  // initial velocity update and position estimates
  for(unsigned int i = 0; i < num_particles; ++i){
    p->velocity[i] += force_ext * dt;
    pos_estimate[i] = p->position[i] + p->velocity[i] * dt;
    // test_pos.push_back(std::array<float, 3>{pos_estimate[i].x,pos_estimate[i].y,pos_estimate[i].z});
    // since we do not have constraint groups, we can initalize the delta_x in this loop
  }

  // generate neighbors
  if(makePointSet){
    // setPointSetTest(nsearch,test_pos);
    setPointSet(nsearch, pos_estimate);
    makePointSet = false;
  }
  // setPointSet(nsearch, pos_estimate);
  getNeighbors(nsearch,neighbors);

  // Test neighbor array
  // for(int k = 0; k < neighbors[2].size(); ++k){
  //   Vector3 pos_test{pos_estimate[neighbors[2][k]].x,pos_estimate[neighbors[2][k]].y,pos_estimate[neighbors[2][k]].z};
  //   DrawSphereWires(pos_test, 0.2f, 6,6, RED);
  // }

  // check for collision with box
  boxConstraint.checkCollision(pos_estimate, plane);
  //check inter particle collisions
  contactConstraint.generateContacts(neighbors, pos_estimate, radius);

  // solve constraints
  for(unsigned int i = 0; i < solver_i; ++i){
    // reset delta values
    for(unsigned int i = 0; i < num_particles; ++i){
      delta_x[i] = glm::vec3(0.0f);
    }
    // Solve constraints
    contactConstraint.solve(pos_estimate,delta_x,radius);
    boxConstraint.solve(pos_estimate,delta_x, plane);

    // add delta_x to position estimates
    for(unsigned int i = 0; i < num_particles; ++i){
      pos_estimate[i] += delta_x[i]/2.0f;
    }
  }

  //

  // // // update veocity and position
   for(unsigned int i =  0; i < num_particles; ++i){
    float fps= 1/dt;
    p->velocity[i] = (pos_estimate[i] - p->position[i])*fps;
    p->position[i] = pos_estimate[i];

    //TODO clean up the neighbors array
    neighbors[i].clear();
   }

  // printVector(pos_estimate[0]);
}

// void EulerUpdater::update(double dt, ParticleData *p){
//   const glm:: vec4 globalA{dt * m_globalAcceleration.x , dt * m_globalAcceleration.y,
//     dt * m_globalAcceleration.z, 0.0f};

//   const float localDT = (float)dt;
//   const unsigned int endId = p->m_count;
//   int n = 4;
//   float sdt = localDT/n;

//   for(size_t i = 0; i < endId; ++i){
//     for(int j = 0 ;j < n; ++j){
//       p->m_acc[i] += globalA;
//       p->m_vel[i] += sdt* p->m_acc[i];
//       p->m_pos[i] += sdt* p->m_vel[i];
//     }

//     std::array<float,3> temp = {p->m_pos[i].x,p->m_pos[i].y, p->m_pos[i].z};
//     p->positions[i] = temp;

//   }
// }

// void FloorUpdater::update(double dt, ParticleData *p){
//   const size_t endId = p->m_count;

//   for(size_t i = 0; i < endId; ++i){
//     if(p->m_pos[i].y < m_floorY){
//       glm::vec4 force = p->m_acc[i];
//       float normalFactor = glm::dot(force, glm::vec4(0.0f, 1.0f, 0.0f,0.0f));
//       if(normalFactor < 0.0f){
//         force -= glm::vec4(0.0f, 1.0f,0.0f, 0.0f) * normalFactor;
//       }
//       float velFactor = glm::dot(p->m_vel[i], glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));
//       p->m_vel[i] -= glm::vec4(0.0f, 1.0f, 0.0f, 0.0f) * (1.0f + m_bounceFactor) * velFactor;
//       p->m_acc[i] = force;
//     }
//   }
// }

// void WindUpdater::update(double dt, ParticleData *p){
//   for (size_t i = 0; i < p->m_count; ++i){
//     p->m_vel[i] += wind;
//   }
// }

// float vectorMag(glm::vec4 &v){
//
//   float result = 0;
//   result = v.x*v.x + v.y*v.y + v.z*v.z;
//   result = glm::sqrt(result);
//   return result;
// }

// void Constraints::resetNeighbors(CompactNSearch::NeighborhoodSearch &nsearch){
//   nsearch.find_neighbors();
// }

// void Constraints::setPointSet(CompactNSearch::NeighborhoodSearch &nsearch, ParticleData *p){
//   pointId = nsearch.add_point_set(p->positions.front().data(), p->positions.size());
// }

// std::vector<unsigned int> Constraints::getNeighbors(CompactNSearch::NeighborhoodSearch &nsearch, ParticleData *p ,int n){
//   resetNeighbors(nsearch);
//   CompactNSearch::PointSet const& ps_1 = nsearch.point_set(pointId);
//   std::vector<unsigned int> result;
//   for(size_t i =0; i < ps_1.n_neighbors(pointId, n); ++i){
//     result.push_back(ps_1.neighbor(pointId, n, i));
//   }
//   return result;
// }

// void Constraints::frictionConstraint(glm::vec4 &x_i, glm::vec4 &x_j){
//   glm::vec4 x_ij = x_j - x_i;
//   float x_ij_mag = vectorMag(x_ij);
//   glm::vec4 delta_x_i = -0.5f * (2 * 0.1f -x_ij_mag) * (x_ij/x_ij_mag);
//   glm::vec4 delta_x_j = -0.5f * (2 * 0.1f -x_ij_mag) * (x_ij/x_ij_mag);
//
//   glm::vec4 x_i_temp = x_i + delta_x_i;
//   glm::vec4 x_j_temp = x_j + delta_x_j;

//   x_i = x_i_temp;
//   x_j = x_j_temp;

//   // glm::vec4 delta_x_ij = delta_x_i - delta_x_j;
//   // glm::vec4 delta_x_ij_tan = delta_x_ij - (glm::dot(delta_x_ij, x_ij)) * (x_ij/(x_ij_mag * x_ij_mag));
//   //
//   // float temp = (2 * 0.1f * 0.3)/vectorMag(delta_x_ij_tan);
//   // float min_fric = glm::min(temp,1.0f);
// }

// void Constraints::simulate(ParticleData *p, float dt, CompactNSearch::NeighborhoodSearch &nsearch){

//   for(size_t i = 0; i < p->m_count; ++i){
//     p->m_vel[i] +=  1.0 * dt;
//     glm::vec4 x_i_temp = p->m_vel[i] * dt;
//     std::vector<unsigned int> neighbors = getNeighbors(nsearch, p, i);
//
//   }
// }

// void Constraints::update(double dt, ParticleData *p){

// }

