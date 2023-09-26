#include "TimeStepGranularModel.hpp"
#include "../entities/TimeManager.hpp"
#include "../entities/Simulation.hpp"
#include "entities/ParticleData.hpp"
#include "entities/TimeIntegration.hpp"
#include "utils/Common.hpp"
#include "utils/CompactNSearch.h"
#include <cmath>
#include <omp.h>
#include <algorithm>

using namespace PBD;
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
std::string sep = "\n-----------------------------------------\n";
  
void TimeStepGranularModel::step(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch){

  TimeManager *tm = TimeManager::getCurrent();
  const Real h = tm->getTimeStepSize();
  ParticleData &pd = model.getParticles();

  clearAccelerations(model);

  // TIme intergration
  for(unsigned int i = 0; i < pd.size(); ++i){
    if(pd.getIsActive(i)){
      model.getDeltaX(i).setZero();
      pd.getOldPosition(i) = pd.getPosition(i);
      TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
      if(pd.getMass(i) <= 0){
        std::cout << pd.getMass(i) << "\n";
      }
    }
  }

  // Neighborhood search
  model.generateNeighbors(nsearch, model.m_pointId1,model.m_pointId2, model.m_pointId3);

  // Constraint Projection
  constraintProjection(model);

  // Update velocity
  for(unsigned int i = 0; i < pd.size(); ++i){
    if(pd.getIsActive(i)){
      TimeIntegration::VelocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
      // particle sleeping
      Real difference = (pd.getPosition(i) - pd.getOldPosition(i)).norm();
      if(difference > static_cast<Real>(0.01)){
        pd.m_oldX[i] = pd.m_x[i];
      }
    }
  }
  

  checkBoundary(model);
  // mergeParticles(model);
  merge2Particles(model);
  // deleteParticles(model);
  // std::cout << pd.getNumberOfParticles() << "\n";
  // Updated upsampled particles
  // upsampledParticlesUpdate(model, h);
  // Clear Neighbors
  //
  model.clearNeighbors();
}

void TimeStepGranularModel::clearAccelerations(GranularModel &granularModel){
  ParticleData &pd = granularModel.getParticles();
  const unsigned int count = pd.size();
  // Simulation* sim = Simulation::getCurrent();
  const Vector3r grav(0.0,-9.81,0.0);
  for(unsigned int i = 0; i < count; ++i){
    if(pd.getMass(i) != 0.0){
      Vector3r &a = pd.getAcceleration(i);
      a = grav;
    }
  }
}

void TimeStepGranularModel::reset(){}

void TimeStepGranularModel::constraintProjection(GranularModel &model){
  const unsigned int maxIter = 8;
  const unsigned int maxStabalizationIter = 2;
  unsigned int iter = 0;
  unsigned int stabalizationIter = 0;

  ParticleData &pd = model.getParticles();

  // for(unsigned int i = 0; i < pd.size() ; ++i){
  //   model.getDeltaX(i).setZero();
  //   model.setNumConstraints(i, 0);
  // }

  // solve stabalization constraints
  while(stabalizationIter < maxStabalizationIter){

    #pragma omp parallel default(shared)
    {
      #pragma omp for schedule(static)
      for(unsigned int i = 0; i < pd.size(); ++i){
        model.getDeltaX(i).setZero();
        model.m_numConstraints[i] = 0;
      }
    }

    for(unsigned int i = 0; i < pd.size(); ++i){
      if(pd.getIsActive(i)){
        // boundary collision handling
        for(unsigned int boundaryIndex : model.m_boundaryNeighbors[i]){
          boundaryConstraint(model, i, boundaryIndex);
        }
        // inter particle collision handling
        for(unsigned int particleIndex : model.m_neighbors[i]){
          contactConstraint(model, i, particleIndex);
          // contactConstraintFriction(model, i, particleIndex);
        }
      }
    }

    // Update estimated position
    for(unsigned int i = 0; i < pd.size(); ++i){

      if(pd.getIsActive(i)){

        if(model.m_numConstraints[i] == 0){
        // vvvvvvvvvv stabalization for collision vvvvvvvvvvvvvvvvvvvvvv
          model.m_particles.m_oldX[i] += model.m_deltaX[i];
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          model.m_particles.m_x[i] += model.m_deltaX[i];
        }else{
         // vvvvvvvvvv stabalization for collision vvvvvvvvvvvvvvvvvvvvvv
          model.m_particles.m_oldX[i] += model.m_deltaX[i]/model.m_numConstraints[i];
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          model.m_particles.m_x[i] += model.m_deltaX[i]/model.m_numConstraints[i];
        }
      }
      
    }
    stabalizationIter++;
  }

  // solver iterations
  while(iter < maxIter){
    for(unsigned int i = 0; i < pd.size(); ++i){
      model.getDeltaX(i).setZero();
      model.m_numConstraints[i] = 0;
    }


    for(unsigned int i = 0; i < pd.size(); ++i){
      
      if(pd.getIsActive(i)){
        // boundary collision handling
        for(unsigned int boundaryIndex : model.m_boundaryNeighbors[i]){
          boundaryConstraint(model, i, boundaryIndex);
        }
        // inter particle collision handling
        for(unsigned int particleIndex : model.m_neighbors[i]){
          // contactConstraint(model, i, particleIndex);
          contactConstraintFriction(model, i, particleIndex);
        }
      }
    }

    // Update estimated position
    for(unsigned int i = 0; i < pd.size(); ++i){
      if(pd.getIsActive(i)){
        if(model.m_numConstraints[i] == 0){
          model.m_particles.m_x[i] += model.m_deltaX[i];
        }else{
          model.m_particles.m_x[i] += model.m_deltaX[i] * 1.5/model.m_numConstraints[i];
        }
      }
    }
    iter++;
  }
  // Update merge flag for particles
  // set up particles to be deleted and add the new particles
}

void TimeStepGranularModel::boundaryConstraint(GranularModel &model, const unsigned int x1, const unsigned int x2){
  Vector3r &p1 = model.getParticles().getPosition(x1);
  Vector3r &p2 = model.getBoundaryX(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real mag = dist - (model.getParticles().getRadius(x1) + model.getParticleRadius());

  if(mag < static_cast<Real>(0.0)){
    model.m_deltaX[x1] -= (mag/dist) * p_12;
    model.m_numConstraints[x1] += 1;
  }
}

void TimeStepGranularModel::contactConstraint(GranularModel &model, const unsigned int x1, const unsigned int x2){
  ParticleData &pd = model.getParticles();

  Vector3r p1 = pd.getPosition(x1);
  Vector3r p2 = pd.getPosition(x2);

  // Real massScaled1 = pd.getMass(x1) * exp(-pd.getPosition(x1).y());
  // Real massScaled2 = pd.getMass(x2) * exp(-pd.getPosition(x2).y());
  Real massScaled1 = 1.0 / pd.getMass(x1);
  Real massScaled2 = 1.0 / pd.getMass(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real mag = dist - (pd.getRadius(x1) + pd.getRadius(x2));

  if(mag < static_cast<Real>(0.0)){
    Real invMassSum = 1.0/(massScaled1 + massScaled2);
    model.m_deltaX[x1] -= massScaled1 * invMassSum * (mag/dist) * p_12;
    model.m_deltaX[x2] += massScaled2 * invMassSum * (mag/dist) * p_12;
    model.m_numConstraints[x1] += 1;
    model.m_numConstraints[x2] += 1;
  }
}

void TimeStepGranularModel::contactConstraintFriction(GranularModel &model, const unsigned int x1, const unsigned int x2){

  ParticleData &pd = model.getParticles();
  Vector3r p1 = pd.getPosition(x1);
  Vector3r p2 = pd.getPosition(x2);

  // Real massScaled1 = pd.getMass(x1) * exp(-pd.getPosition(x1).y());
  // Real massScaled2 = pd.getMass(x2) * exp(-pd.getPosition(x2).y());
  Real massScaled1 = 1.0 / pd.getMass(x1);
  Real massScaled2 = 1.0 / pd.getMass(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real diameter = (pd.getRadius(x1) + pd.getRadius(x2));
  Real mag = dist - diameter;

  if(mag < static_cast<Real>(0.0)){
    Real invMassSum = 1.0/(massScaled1 + massScaled2);
    Vector3r delta_p1 = massScaled1 * invMassSum * (mag/dist) * p_12;
    Vector3r delta_p2 = massScaled2 * invMassSum * (mag/dist) * p_12;
    model.m_deltaX[x1] -= delta_p1;
    model.m_deltaX[x2] += delta_p2;

    // Vector3r delta_p12 = model.m_deltaX[x1] - model.m_deltaX[x2];
    Vector3r delta_p12 = delta_p1 - delta_p2;
    Vector3r delta_p12_tang = delta_p12 - (delta_p12.dot(p_12))*p_12/(dist*dist);
    Real delta_p12_tang_norm = delta_p12_tang.norm();
    Real mu_k = static_cast<Real>(0.35);
    Real mu_s = static_cast<Real>(0.30);
    Real fric_tangent = diameter * mu_k/delta_p12_tang_norm;
    Real min_fric = std::min(fric_tangent,static_cast<Real>(1.0));

    bool check_fric = delta_p12_tang_norm < diameter * mu_s;

    if(check_fric){
      model.m_deltaX[x1] -= massScaled1 * invMassSum * delta_p12_tang;
      model.m_deltaX[x2] += massScaled2 * invMassSum * delta_p12_tang;
      model.m_numConstraints[x1] += 1;
      model.m_numConstraints[x2] += 1;
    }else{
      model.m_deltaX[x1] -= massScaled1 * invMassSum * delta_p12_tang * min_fric;
      model.m_deltaX[x2] += massScaled2 * invMassSum * delta_p12_tang * min_fric;
      model.m_numConstraints[x1] += 1;
      model.m_numConstraints[x2] += 1;
    }
  }
}

void TimeStepGranularModel::upsampledParticlesUpdate(GranularModel &model, const Real h){

  // Vector3r g(0.0,-9.81,0.0);
  Real delta_t = static_cast<Real>(0.005);
  Real radiusLR = 0.025;
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
  Real h_HR_2 = 9.0 * radiusLR * radiusLR;
  // #pragma omp parallel
  {
    // #pragma omp parallel for
    for(unsigned int i = 0; i < model.m_upsampledParticlesX.size(); ++i){
      if(model.m_isActiveUpsampled[i]){
        Real w_ij_sum = 0;
        Vector3r w_ij_v_j_sum(0.0,0.0,0.0);
        std::vector<Real> weights;
        Real w_ij_max = 0;

        for(unsigned int j = 0; j < model.m_upsampledNeighbors[i].size(); ++j){
          unsigned int particleIndex = model.m_upsampledNeighbors[i][j];
          if(model.m_particles.getIsActive(particleIndex)){
            Vector3r x_ij = model.m_upsampledParticlesX[i] - model.getParticles().getPosition(particleIndex);
            Real x_ij_norm_2 = x_ij.norm() * x_ij.norm();
            Real temp = (static_cast<Real>(1.0)  - (x_ij_norm_2/(h_HR_2)));
            Real w_ij = std::max(static_cast<Real>(0.0), temp*temp*temp);
            weights.push_back(w_ij);
            w_ij_sum += w_ij;
            w_ij_v_j_sum += w_ij * model.getParticles().getVelocity(particleIndex);
          }
        }

        for(unsigned int particleIndex : model.m_upsampledBoundaryNeighbors[i]){
          Vector3r x_ij = model.m_upsampledParticlesX[i] - model.getBoundaryX(particleIndex);
          Real x_ij_norm_2 = x_ij.norm() * x_ij.norm();
          Real temp = (static_cast<Real>(1.0)  - (x_ij_norm_2/(h_HR_2)));
          Real w_ij = std::max(static_cast<Real>(0.0), temp*temp*temp);
          weights.push_back(w_ij);
          w_ij_sum += w_ij;
        }

        if(weights.empty()){
          w_ij_max = 0;
        }else{
          w_ij_max = *max_element(weights.begin(),weights.end());
        }

        Vector3r v_i_avg;

        if(w_ij_sum <= static_cast<Real>(0.0)){
          v_i_avg.setZero();
        }
        else{
          v_i_avg = w_ij_v_j_sum/(w_ij_sum);
        }

        Real alpha_i = static_cast<Real>(0.0);
        // Real c1 = static_cast<Real>(512.0)/static_cast<Real>(729.0);
        Real c1 = 0.7023319616;
        Real c2 = static_cast<Real>(0.6);

        Real c2Check;
        c2Check = w_ij_max/w_ij_sum;
        bool alphaCheck = (w_ij_max <= c1) || (c2Check >= c2);

        if(alphaCheck){
          alpha_i = static_cast<Real>(1.0) - w_ij_max;
        }
        // std::cout << w_ij_max<< "\n";
        // alpha_i = 0.1;
        Vector3r gravitation = model.getParticles().getAcceleration(i);
        model.m_upsampledParticlesV[i] = (1.0 - alpha_i) * v_i_avg + alpha_i*(model.m_upsampledParticlesV[i] + delta_t*Vector3r(0.0,-9.81,0.0));
        model.m_upsampledParticlesX[i] +=  delta_t * model.m_upsampledParticlesV[i];
        if(model.m_upsampledParticlesX[i].y() < 0.0){
          model.m_upsampledParticlesX[i] = Vector3r(model.m_upsampledParticlesX[i].x(), 0.0 ,model.m_upsampledParticlesX[i].z());
        }
      }
    }
  }
}

void TimeStepGranularModel::checkBoundary(GranularModel &model){
  ParticleData &pd = model.getParticles();
  for(int i = 0; i < pd.size(); ++i){
    if(pd.getIsActive(i)){

      if(model.m_neighbors[i].size() == 0){
        model.m_isBoundary[i] = true;
        continue;
      }

      // for(unsigned int particleIndex : model.m_neighbors[i]){
      //   com += pd.getPosition(particleIndex); 
      //   count++;
      // }
      // com /= count;

      Vector3r com(0.0,0.0,0.0);
      unsigned int count = 0;       

      unsigned int n = model.m_neighbors[i][0];
      Vector3r a = pd.getPosition(i);
      Vector3r b = a.cross(pd.getPosition(n));
      Real b_norm2 =  b.dot(b);
      Vector3r proj = a.dot(b) * b /b_norm2;
      Vector3r projSum(0.0,0.0,0.0);

      for(unsigned int particleIndex : model.m_neighbors[i]){
        Vector3r p = pd.getPosition(particleIndex);
        com += p;  
        count++;
        projSum += (proj) - (p.dot(b) * b/(b_norm2)); 
      }

      com /= count;
      Real dist = (pd.getPosition(i) - com).norm();
      Real projSumNorm = projSum.norm(); 
      
      // std::cout << projSum << "\n";

      bool projCheck = (projSumNorm < 0.00001) && (projSumNorm > -0.00001);
      
      if(dist > 0.001 || projCheck){
        model.m_isBoundary[i] = true;
      }else{
        model.m_isBoundary[i] = false;
      }
    }
    // std::cout << dist << "\n";
  }
}

template <typename T> void quickDelete(unsigned int index, std::vector<T> &v){
  v[index] = v.back();
  v.pop_back();
}  

void TimeStepGranularModel::merge2Particles(GranularModel &model){
  ParticleData &pd = model.getParticles();
  unsigned int n = pd.size();
  
  for(unsigned int i = 0; i < n ; ++i){
 
    if(!model.m_isBoundary[i] && pd.getIsActive(i) && !model.getMergeFlag(i) && (pd.getMass(i) < model.maxMass)){
    // if there are no neighbors then skip
      if(model.m_neighbors[i].size() == 0){
        continue;
      }
      // get closest neighbor
      unsigned int closestParticle;
      Real distance = 10000.0;
      bool validNeighborPresent = false;
      // model.m_deleteFlag[i] = true;

      for(unsigned int particleIndex : model.m_neighbors[i]){
        Real dist = (pd.getPosition(i) - pd.getPosition(particleIndex)).norm();
        // check if particle is marked for deletion
        // bool isDelete = map.find(particleIndex) != map.end();
        if( (dist < distance) && (!model.m_mergeFlag[particleIndex]) && pd.getIsActive(particleIndex)){
          distance  = dist;
          closestParticle = particleIndex;
          validNeighborPresent = true;
        }
      }
      
      // check to see if any neighbors are valid
      if(!validNeighborPresent){
        continue;
      }

      if(pd.getRadius(i) <= pd.getRadius(closestParticle)){
        model.m_bigger.push_back(i);
        model.m_smaller.push_back(closestParticle);
      }else{
        model.m_bigger.push_back(closestParticle);
        model.m_smaller.push_back(i);
      }

      model.m_mergeCount.push_back(0);
      model.m_mergeFlag[i] = true;
      model.m_mergeFlag[closestParticle] = true;

      // std::cout << model.getMergeFlag(i) << "\n";
    }else if(model.m_isBoundary[i] && pd.getIsActive(i) && !model.getMergeFlag(i) && (pd.getMass(i) > model.minMass)){
      // std::cout << "split" << "\n";
      if(model.m_inactive.size() > 0 ){
        Real newMass = pd.getMass(i) * 0.5;   
        Real newRadius = std::cbrt(pow(pd.getRadius(i) ,3) * 0.5);
        Vector3r newPos = pd.getPosition(i) + Vector3r(newRadius,newRadius,0); 
        Vector3r newVel = pd.getVelocity(i)*0.5;
        pd.m_masses[i] = newMass;
        pd.m_radius[i] = newRadius;
        pd.m_v[i] = Vector3r(0.0,0.0,0.0);
        unsigned int n = model.m_inactive.back();
        pd.m_masses[n] = newMass;
        pd.m_radius[n] = newRadius;
        pd.m_x[n] = newPos;
        pd.m_v[n] = Vector3r(0.0,0.0,0.0);
        pd.setIsActive(n, true);
        model.m_inactive.pop_back();
      } 
    }
  }

  for(int i = 0; i < model.m_bigger.size(); ++i){
    unsigned int n_big = model.m_bigger[i];
    unsigned int n_small = model.m_smaller[i];
    if(model.m_mergeCount[i] == 0){
      Real deltaR = pow(pd.getRadius(n_small)* 0.5 ,3);
      Real deltaM = pd.getMass(n_small) * 0.5;
      pd.m_radius[n_big]  =  std::cbrt(pow(pd.getRadius(n_big),3) + deltaR);
      pd.m_radius[n_small]  =  std::cbrt(pow(pd.getRadius(n_small),3) - deltaR);
      pd.m_masses[n_big] += deltaM; 
      pd.m_masses[n_small] -= deltaM; 
      model.m_mergeCount[i]++;
    }else if(model.m_mergeCount[i] == 1){
      // std::cout << 1 << "\n";
      Real deltaR =   pow(pd.getRadius(n_small),3);
      Real deltaM = pd.getMass(n_small);
      pd.m_radius[n_big] = std::cbrt(pow(pd.getRadius(n_big),3) + deltaR);
      pd.m_masses[n_big] += deltaM;
      model.m_mergeFlag[n_big] = false;
      model.m_mergeFlag[n_small] = false;
      pd.setIsActive(n_small, false);
      model.m_inactive.push_back(n_small);
      quickDelete(i, model.m_bigger);
      quickDelete(i, model.m_smaller);
      quickDelete(i, model.m_mergeCount);
      
      int n_neighbors = model.m_neighborsUpsampled[n_small].size();
      unsigned int n = std::min(10,n_neighbors);
      for(int j = 0; j < n; ++j){
        model.m_isActiveUpsampled[model.m_neighborsUpsampled[n_small][j]] = false;
        model.m_inactiveUpsampled.push_back(model.m_neighborsUpsampled[n_small][j]);
      }
    }
  }
}

void TimeStepGranularModel::applyForce(GranularModel &model){
  ParticleData &pd = model.getParticles();

  for(int i = 0; i < pd.size(); ++i){
    pd.m_v[i] += Vector3r(1.0,0.0,0.0);
  }
}
