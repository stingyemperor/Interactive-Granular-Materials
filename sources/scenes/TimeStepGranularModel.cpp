#include "TimeStepGranularModel.hpp"
#include "../entities/TimeManager.hpp"
#include "../entities/Simulation.hpp"
#include "entities/ParticleData.hpp"
#include "entities/TimeIntegration.hpp"
#include "utils/Common.hpp"
#include "utils/CompactNSearch.h"
#include <omp.h>
// #include <algorithm>

using namespace PBD;

void TimeStepGranularModel::step(GranularModel &model, CompactNSearch::NeighborhoodSearch &nsearch){
  
  TimeManager *tm = TimeManager::getCurrent();
  const Real h = tm->getTimeStepSize();
  ParticleData &pd = model.getParticles();

  clearAccelerations(model);

  // TIme intergration
  for(unsigned int i = 0; i < pd.size(); ++i){
    model.getDeltaX(i).setZero();
    pd.getOldPosition(i) = pd.getPosition(i);
    TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
  }

  // Neighborhood search
  model.generateNeighbors(nsearch, model.m_pointId1,model.m_pointId2, model.m_pointId3);  


  // Constraint Projection
  constraintProjection(model);
    
  // Update velocity
  for(unsigned int i = 0; i < pd.size(); ++i){
    TimeIntegration::VelocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));

    // particle sleeping
    Real difference = (pd.getPosition(i) - pd.getOldPosition(i)).norm();
    if(difference > static_cast<Real>(0.01)){
      pd.m_oldX[i] = pd.m_x[i];
    }
  }

  // Updated upsampled particles
  // upsampledParticlesUpdate(model, h);
  // Clear Neighbors
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
  const unsigned int maxIter = 6;
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

      // boundary collision handling
      for(unsigned int boundaryIndex : model.m_boundaryNeighbors[i]){
        boundaryConstraint(model, i, boundaryIndex);
      }
      
      // inter particle collision handling
      for(unsigned int particleIndex : model.m_neighbors[i]){
        contactConstraint(model, i, particleIndex);
      }
    }
    
    // Update estimated position
    for(unsigned int i = 0; i < pd.size(); ++i){
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

    stabalizationIter++;
  }
  
  // solver iterations
  while(iter < maxIter){
    for(unsigned int i = 0; i < pd.size(); ++i){
      model.getDeltaX(i).setZero();
      model.m_numConstraints[i] = 0;
    }

    for(unsigned int i = 0; i < pd.size(); ++i){

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
    
    // Update estimated position
    for(unsigned int i = 0; i < pd.size(); ++i){
      if(model.m_numConstraints[i] == 0){
        model.m_particles.m_x[i] += model.m_deltaX[i]; 
      }else{
        model.m_particles.m_x[i] += model.m_deltaX[i]/model.m_numConstraints[i];
      }
    }

    iter++;
  }

  // Update merge flag for particles
  
}

void TimeStepGranularModel::boundaryConstraint(GranularModel &model, const unsigned int x1, const unsigned int x2){
  Vector3r p1 = model.getParticles().getPosition(x1);
  Vector3r p2 = model.getBoundaryX(x2);

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
  Real massScaled1 =pd.getMass(x1);
  Real massScaled2 =pd.getMass(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real mag = dist - (pd.getRadius(x1) + pd.getRadius(x2));

  if(mag < static_cast<Real>(0.0)){
    Real invMassSum = 1/(massScaled1 + massScaled2);
    model.m_deltaX[x1] -= pd.getMass(x1) * invMassSum * (mag/dist) * p_12;
    model.m_deltaX[x2] += pd.getMass(x2) * invMassSum * (mag/dist) * p_12;
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
  Real massScaled1 =pd.getMass(x1);
  Real massScaled2 =pd.getMass(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real diameter = (pd.getRadius(x1) + pd.getRadius(x2));
  Real mag = dist - diameter;

  if(mag < static_cast<Real>(0.0)){
    Real invMassSum = 1/(massScaled1 + massScaled2);
    Vector3r delta_p1 = massScaled1 * invMassSum * (mag/dist) * p_12;
    Vector3r delta_p2 = massScaled2 * invMassSum * (mag/dist) * p_12;
    model.m_deltaX[x1] -= delta_p1;
    model.m_deltaX[x2] += delta_p2;

    Vector3r delta_p12 = model.m_deltaX[x1] - model.m_deltaX[x2];  
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

  Vector3r g(0.0,-9.81,0.0);
  Real delta_t = static_cast<Real>(0.01);
  Real radiusLR = model.m_particleRadius;
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

  #pragma omp parallel 
  {
    #pragma omp parallel for
    for(unsigned int i = 0; i < model.m_upsampledParticlesX.size(); ++i){
      Real w_ij_sum = 0;
      Vector3r w_ij_v_j_sum(0.0,0.0,0.0); 
      std::vector<Real> weights;
      Real w_ij_max = 0; 
      unsigned int num_neighbors = 0; 
      
      for(unsigned int j = 0; j < model.m_upsampledNeighbors[i].size(); ++j){
        unsigned int particleIndex = model.m_upsampledNeighbors[i][j];  
        Vector3r x_ij = model.m_upsampledParticlesX[i] - model.getParticles().getPosition(particleIndex); 
        Real x_ij_norm_2 = x_ij.norm() * x_ij.norm(); 
        Real temp = (static_cast<Real>(1.0)  - (x_ij_norm_2/(static_cast<Real>(9.0) * radiusLR * radiusLR))); 
        Real w_ij = std::max(static_cast<Real>(0.0), temp*temp*temp);
        weights.push_back(w_ij);
        if(w_ij_max < w_ij){
          w_ij_max = w_ij;
        }
        w_ij_sum += w_ij;
        w_ij_v_j_sum += w_ij * model.getParticles().getVelocity(particleIndex);

        num_neighbors++;
      }
      
      Vector3r v_i_avg;

      if(w_ij_sum <= static_cast<Real>(0.0)){
        v_i_avg.setZero(); 
      }
      else{
        v_i_avg = w_ij_v_j_sum * static_cast<Real>(1.0)/(w_ij_sum);
      }
      
      Real alpha_i = static_cast<Real>(0.0);
      Real c1 = static_cast<Real>(512.0)/static_cast<Real>(729.0);
      Real c2 = static_cast<Real>(0.6);
      bool alphaCheck = (w_ij_max <= c1) || ((w_ij_max/w_ij_sum) >= c2); 

      if(alphaCheck){
        alpha_i = static_cast<Real>(1.0) - w_ij_max;
      }
      Vector3r gravitation = model.getParticles().getAcceleration(0);
      model.m_upsampledParticlesV[i] = (static_cast<Real>(1.0) - alpha_i) * v_i_avg;  
      Vector3r test(0.0,0.0,0.0);
      model.m_upsampledParticlesX[i] +=  delta_t * model.m_upsampledParticlesV[i];
    }
  }
}

