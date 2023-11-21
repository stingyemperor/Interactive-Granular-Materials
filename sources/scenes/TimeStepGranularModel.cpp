#include "TimeStepGranularModel.hpp"
#include "../entities/Simulation.hpp"
#include "../entities/TimeManager.hpp"
#include "entities/ParticleData.hpp"
#include "entities/TimeIntegration.hpp"
#include "utils/Common.hpp"
#include "utils/CompactNSearch.h"
#include <cmath>
#include <fstream>
// #include <omp.h>
#include "random"
#include <algorithm>
#include <chrono>

using namespace PBD;
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                             ", ", "", "", " << ", ";");
std::string sep = "\n-----------------------------------------\n";

void TimeStepGranularModel::step(GranularModel &model,
                                 CompactNSearch::NeighborhoodSearch &nsearch,std::uniform_real_distribution<double> &distribution, std::mt19937 &generator) {

  TimeManager *tm = TimeManager::getCurrent();
  const Real h = tm->getTimeStepSize();
  ParticleData &pd = model.getParticles();
  Vector3r normal(0.0, 1.0, 0.0);

  clearAccelerations(model);

  // Time intergration
  for (unsigned int i = 0; i < pd.size(); ++i) {
    if (pd.getIsActive(i)) {
      model.getDeltaX(i).setZero();
      pd.getOldPosition(i) = pd.getPosition(i);
      TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i),
                                         pd.getVelocity(i),
                                         pd.getAcceleration(i));
      if (pd.getMass(i) <= 0) {
        std::cout << pd.getMass(i) << "\n";
      }
    }
  }

  // Neighborhood search
  model.generateNeighbors(nsearch, model.m_pointId1, model.m_pointId2,
                          model.m_pointId3);

  // Constraint Projection
  constraintProjection(model);

  // Update velocity
  for (unsigned int i = 0; i < pd.size(); ++i) {
    if (pd.getIsActive(i)) {
      TimeIntegration::VelocityUpdateFirstOrder(
          h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i),
          pd.getVelocity(i));
      // particle sleeping
      Real difference = (pd.getPosition(i) - pd.getOldPosition(i)).norm();
      if (difference > static_cast<Real>(0.01)) {
        pd.m_oldX[i] = pd.m_x[i];
      }
      // floor collision check
      if (model.m_floorCollision[i]) {
        pd.m_v[i] = (pd.m_v[i] - 2 * (pd.m_v[i].dot(normal)) * normal) * 0.7;
      }
    }
  }

  // checkBoundary(model);
  //  mergeParticles(model);
  // merge2Particles(model,distribution, generator);
  // std::cout << pd.getNumberOfParticles() << "\n";;
  // Updated upsampled particles
  // upsampledParticlesUpdate(model, h);
  // Clear Neighbors
  //
  model.clearNeighbors();
}

void TimeStepGranularModel::clearAccelerations(GranularModel &granularModel) {
  ParticleData &pd = granularModel.getParticles();
  const unsigned int count = pd.size();
  // Simulation* sim = Simulation::getCurrent();
  const Vector3r grav(0.0, -9.81, 0.0);
  for (unsigned int i = 0; i < count; ++i) {
    if (pd.getMass(i) != 0.0) {
      Vector3r &a = pd.getAcceleration(i);
      a = grav;
    }
  }
}

void TimeStepGranularModel::reset() {}

void TimeStepGranularModel::constraintProjection(GranularModel &model) {
  const unsigned int maxIter = 8;
  const unsigned int maxStabalizationIter = 2;
  unsigned int iter = 0;
  unsigned int stabalizationIter = 0;

  ParticleData &pd = model.getParticles();

  // for(unsigned int i = 0; i < pd.size() ; ++i){
  //   model.getDeltaX(i).setZero();
  //   model.setNumConstraints(i, 0);
  // }

  while (stabalizationIter < maxStabalizationIter) {

    // #ifdef _MSC_VER
    //   concurrency::parallel_for_each(
    // #elif defined(__APPLE__) && defined(__clang__)
    //   std::for_each(oneapi::dpl::execution::par,
    // #else
    //   __gnu_parallel::for_each(
    // #endif

    for (unsigned int i = 0; i < pd.size(); ++i) {
      model.getDeltaX(i).setZero();
      model.m_numConstraints[i] = 0;
      model.m_floorCollision[i] = false;
    }

#pragma omp parallel for
    for (unsigned int i = 0; i < pd.size(); ++i) {
      if (pd.getIsActive(i)) {

        // boundary collision handling
        floorConstraint(model, i);
        // for (unsigned int boundaryIndex : model.m_boundaryNeighbors[i]) {
        //   boundaryConstraint(model, i, boundaryIndex);
        //
        // inter particle collision handling
        for (unsigned int particleIndex : model.m_neighbors[i]) {
          contactConstraint(model, i, particleIndex);
          // contactConstraintFriction(model, i, particleIndex);
        }
      }
    }

#pragma omp parallel for
    // Update estimated position
    for (unsigned int i = 0; i < pd.size(); ++i) {

      if (pd.getIsActive(i)) {

        if (model.m_numConstraints[i] == 0) {
          // vvvvvvvvvv stabalization for collision vvvvvvvvvvvvvvvvvvvvvv
          model.m_particles.m_oldX[i] += model.m_deltaX[i];
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          model.m_particles.m_x[i] += model.m_deltaX[i];
        } else {
          // vvvvvvvvvv stabalization for collision vvvvvvvvvvvvvvvvvvvvvv
          model.m_particles.m_oldX[i] +=
              model.m_deltaX[i] / model.m_numConstraints[i];
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          model.m_particles.m_x[i] +=
              model.m_deltaX[i] / model.m_numConstraints[i];
        }
      }
    }
    stabalizationIter++;
  }

  // solver iterations
  while (iter < maxIter) {
#pragma omp parallel for
    for (unsigned int i = 0; i < pd.size(); ++i) {
      model.getDeltaX(i).setZero();
      model.m_numConstraints[i] = 0;
      model.m_floorCollision[i] = false;
    }

#pragma omp parallel for
    for (unsigned int i = 0; i < pd.size(); ++i) {

      if (pd.getIsActive(i)) {
        // boundary collision handling
        floorConstraint(model, i);

        // for (unsigned int boundaryIndex : model.m_boundaryNeighbors[i]) {
        //   boundaryConstraint(model, i, boundaryIndex);
        // }
        // inter particle collision handling
        for (unsigned int particleIndex : model.m_neighbors[i]) {
          // contactConstraint(model, i, particleIndex);
          contactConstraintFriction(model, i, particleIndex);
        }
      }
    }

// Update estimated position
#pragma omp parallel for
    for (unsigned int i = 0; i < pd.size(); ++i) {
      if (pd.getIsActive(i)) {
        if (model.m_numConstraints[i] == 0) {
          model.m_particles.m_x[i] += model.m_deltaX[i];
        } else {
          model.m_particles.m_x[i] +=
              model.m_deltaX[i] * 1.5 / model.m_numConstraints[i];
        }
      }
    }
    iter++;
  }
  // Update merge flag for particles
  // set up particles to be deleted and add the new particles
}

void TimeStepGranularModel::boundaryConstraint(GranularModel &model,
                                               const unsigned int x1,
                                               const unsigned int x2) {
  Vector3r &p1 = model.getParticles().getPosition(x1);
  Vector3r &p2 = model.getBoundaryX(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real mag =
      dist - (model.getParticles().getRadius(x1) + model.getParticleRadius());

  if (mag < static_cast<Real>(0.0)) {
    model.m_deltaX[x1] -= (mag / dist) * p_12;
    model.m_numConstraints[x1] += 1;
  }
}

void TimeStepGranularModel::floorConstraint(GranularModel &model,
                                            unsigned int x1) {
  Vector3r &p1 = model.getParticles().getPosition(x1);
  Vector3r normal = Vector3r(0.0, 1.0, 0.0);
  Real distance = p1.dot(normal) - model.getParticles().getRadius(x1);

  if (distance <= 0.0) {
    model.m_deltaX[x1] -= (distance)*normal;
    model.m_floorCollision[x1] = true;
    model.m_numConstraints[x1] += 1;
  }
}

void TimeStepGranularModel::contactConstraint(GranularModel &model,
                                              const unsigned int x1,
                                              const unsigned int x2) {
  ParticleData &pd = model.getParticles();

  Vector3r &p1 = pd.getPosition(x1);
  Vector3r &p2 = pd.getPosition(x2);

  // Real massScaled1 = pd.getMass(x1) * exp(-pd.getPosition(x1).y());
  // Real massScaled2 = pd.getMass(x2) * exp(-pd.getPosition(x2).y());
  Real massScaled1 = 1.0 / pd.getMass(x1);
  Real massScaled2 = 1.0 / pd.getMass(x2);

  Vector3r p_12 = p1 - p2;
  Real dist = p_12.norm();
  Real mag = dist - (pd.getRadius(x1) + pd.getRadius(x2));

  if (mag < static_cast<Real>(0.0)) {
    Real invMassSum = 1.0 / (massScaled1 + massScaled2);
    model.m_deltaX[x1] -= massScaled1 * invMassSum * (mag / dist) * p_12;
    model.m_deltaX[x2] += massScaled2 * invMassSum * (mag / dist) * p_12;
    model.m_numConstraints[x1] += 1;
    model.m_numConstraints[x2] += 1;
  }
}

void TimeStepGranularModel::contactConstraintFriction(GranularModel &model,
                                                      const unsigned int x1,
                                                      const unsigned int x2) {

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

  if (mag < static_cast<Real>(0.0)) {
    Real invMassSum = 1.0 / (massScaled1 + massScaled2);
    Vector3r delta_p1 = massScaled1 * invMassSum * (mag / dist) * p_12;
    Vector3r delta_p2 = massScaled2 * invMassSum * (mag / dist) * p_12;
    model.m_deltaX[x1] -= delta_p1;
    model.m_deltaX[x2] += delta_p2;

    // Vector3r delta_p12 = model.m_deltaX[x1] - model.m_deltaX[x2];
    Vector3r delta_p12 = delta_p1 - delta_p2;
    Vector3r delta_p12_tang =
        delta_p12 - (delta_p12.dot(p_12)) * p_12 / (dist * dist);
    Real delta_p12_tang_norm = delta_p12_tang.norm();
    Real mu_k = static_cast<Real>(0.35);
    Real mu_s = static_cast<Real>(0.30);
    Real fric_tangent = diameter * mu_k / delta_p12_tang_norm;
    Real min_fric = std::min(fric_tangent, static_cast<Real>(1.0));

    bool check_fric = delta_p12_tang_norm < diameter * mu_s;

    if (check_fric) {
      model.m_deltaX[x1] -= massScaled1 * invMassSum * delta_p12_tang;
      model.m_deltaX[x2] += massScaled2 * invMassSum * delta_p12_tang;
      model.m_numConstraints[x1] += 1;
      model.m_numConstraints[x2] += 1;
    } else {
      model.m_deltaX[x1] -=
          massScaled1 * invMassSum * delta_p12_tang * min_fric;
      model.m_deltaX[x2] +=
          massScaled2 * invMassSum * delta_p12_tang * min_fric;
      model.m_numConstraints[x1] += 1;
      model.m_numConstraints[x2] += 1;
    }
  }
}

void TimeStepGranularModel::upsampledParticlesUpdate(GranularModel &model,
                                                     const Real h) {

  // Vector3r g(0.0,-9.81,0.0);
  Real delta_t = static_cast<Real>(0.0075);
  Real radiusLR = 0.025;
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", " << ", ";");
  Real h_HR_2 = 9.0 * radiusLR * radiusLR;
  // #pragma omp parallel
  {
#pragma omp parallel for
    for (unsigned int i = 0; i < model.m_upsampledParticlesX.size(); ++i) {
      if (model.m_isActiveUpsampled[i]) {
        Real w_ij_sum = 0;
        Vector3r w_ij_v_j_sum(0.0, 0.0, 0.0);
        std::vector<Real> weights;
        Real w_ij_max = 0;

        for (unsigned int j = 0; j < model.m_upsampledNeighbors[i].size();
             ++j) {
          unsigned int particleIndex = model.m_upsampledNeighbors[i][j];
          if (model.m_particles.getIsActive(particleIndex)) {
            Vector3r x_ij = model.m_upsampledParticlesX[i] -
                            model.getParticles().getPosition(particleIndex);
            Real x_ij_norm_2 = x_ij.norm() * x_ij.norm();
            Real temp = (static_cast<Real>(1.0) - (x_ij_norm_2 / (h_HR_2)));
            Real w_ij = std::max(static_cast<Real>(0.0), temp * temp * temp);
            weights.push_back(w_ij);
            w_ij_sum += w_ij;
            w_ij_v_j_sum +=
                w_ij * model.getParticles().getVelocity(particleIndex);
          }
        }

        for (unsigned int particleIndex :
             model.m_upsampledBoundaryNeighbors[i]) {
          Vector3r vel = model.m_upsampledParticlesV[i];
          if (vel.norm() < 0.5) {
            continue;
          }

          Vector3r x_ij = model.m_upsampledParticlesX[i] -
                          model.getBoundaryX(particleIndex);
          // calculate angle between the veloctiy the vector joining the
          // boundary particle and the upsampled particle
          Real angle = std::atan2(x_ij.cross(vel).norm(), x_ij.dot(vel));
          Vector3r boundaryVel(0.0, 0.0, 0.0);
          if (angle >= 0.5 * M_PI) {
            boundaryVel = vel;
          }

          Real x_ij_norm_2 = x_ij.norm() * x_ij.norm() * 1.0;
          Real temp = (static_cast<Real>(1.0) - (x_ij_norm_2 / (h_HR_2)));
          Real w_ij = std::max(static_cast<Real>(0.0), temp * temp * temp);
          weights.push_back(w_ij);
          w_ij_sum += w_ij;
          w_ij_v_j_sum += w_ij * boundaryVel;
        }

        if (weights.empty()) {
          w_ij_max = 0;
        } else {
          w_ij_max = *max_element(weights.begin(), weights.end());
        }

        Vector3r v_i_avg;

        if (w_ij_sum <= static_cast<Real>(0.0)) {
          v_i_avg.setZero();
        } else {
          v_i_avg = w_ij_v_j_sum / (w_ij_sum);
        }

        Real alpha_i = static_cast<Real>(0.0);
        // Real c1 = static_cast<Real>(512.0)/static_cast<Real>(729.0);
        Real c1 = 0.7023319616;
        Real c2 = static_cast<Real>(0.60);

        Real c2Check = w_ij_max / w_ij_sum;
        bool alphaCheck = (w_ij_max <= c1) || (c2Check >= c2);

        if (alphaCheck) {
          alpha_i = static_cast<Real>(1.0) - w_ij_max;
        }
        // std::cout << w_ij_max<< "\n";
        // alpha_i = 0.1;
        Vector3r gravitation = model.getParticles().getAcceleration(i);
        model.m_upsampledParticlesV[i] =
            (1.0 - alpha_i) * v_i_avg +
            alpha_i * (model.m_upsampledParticlesV[i] +
                       delta_t * Vector3r(0.0, -9.81, 0.0));
        model.m_upsampledParticlesX[i] +=
            delta_t * model.m_upsampledParticlesV[i];
        if (model.m_upsampledParticlesX[i].y() < 0.0) {
          model.m_upsampledParticlesX[i] =
              Vector3r(model.m_upsampledParticlesX[i].x(), 0.0,
                       model.m_upsampledParticlesX[i].z());
        }
      }
    }
  }
}

void TimeStepGranularModel::checkBoundary(GranularModel &model) {
  ParticleData &pd = model.getParticles();
  for (int i = 0; i < pd.size(); ++i) {
    if (pd.getIsActive(i)) {

      if (model.m_neighbors[i].size() == 0) {
        model.m_isBoundary[i] = true;
        continue;
      }

      Vector3r com(0.0, 0.0, 0.0);
      unsigned int count = 0;

      // unsigned int n = model.m_neighbors[i][0];
      // Vector3r a = pd.getPosition(i);
      // Vector3r b = a.cross(pd.getPosition(n));
      // Real b_norm2 =  b.dot(b);
      // Vector3r proj = a.dot(b) * b /b_norm2;
      // Vector3r projSum(0.0,0.0,0.0);

      for (unsigned int particleIndex : model.m_neighbors[i]) {
        Vector3r p = pd.getPosition(particleIndex);
        com += p;
        count++;
        // projSum += (proj) - (p.dot(b) * b/(b_norm2));
      }

      com /= count;

      // Calculate the covariance matrix
      Real xx, xy, xz, yy, yz, zz;
      xx = xy = xz = yy = yz = zz = 0.0;

      for (unsigned int particleIndex : model.m_neighbors[i]) {
        Vector3r p = pd.getPosition(particleIndex);
        Vector3r r = p - com;
        xx += r.x() * r.x();
        xy += r.x() * r.y();
        xz += r.x() * r.z();
        yy += r.y() * r.y();
        yz += r.y() * r.z();
        zz += r.z() * r.z();
      }

      xx /= count;
      xy /= count;
      xz /= count;
      yy /= count;
      yz /= count;
      zz /= count;

      Vector3r weightedDir{0.0, 0.0, 0.0};
      Real detX = yy * zz - yz * yz;
      Vector3r axisDir{detX, xz * yz - xy * zz, xy * yz - xz * yy};
      Real weight = detX * detX;

      if (weightedDir.dot(axisDir) < 0.0) {
        weight = -weight;
      }
      weightedDir += weight * axisDir;

      Real detY = xx * zz - xz * xz;
      axisDir.x() = xz * yz - xy * zz;
      axisDir.y() = detY;
      axisDir.z() = xy * xz - yz * xx;

      weight = detY * detY;
      if (weightedDir.dot(axisDir) < 0.0) {
        weight = -weight;
      }
      weightedDir += weight * axisDir;

      Real detZ = xx * yy - xy * xy;
      axisDir.x() = xy * yz - xz * yy;
      axisDir.y() = xy * xz - yz * xx;
      axisDir.z() = detZ;

      weight = detY * detY;
      if (weightedDir.dot(axisDir) < 0.0) {
        weight = -weight;
      }
      weightedDir += weight * axisDir;

      Vector3r normal = weightedDir.normalized();

      Real avgDistance = 0.0;
      for (unsigned int particleIndex : model.m_neighbors[i]) {
        Vector3r ab = pd.getPosition(particleIndex) - com;
        Real n_norm = normal.norm();
        if (n_norm != 0.0) {
          avgDistance +=
              abs(ab.dot(normal) / n_norm); // projection of ab onto normal
        }
      }
      // std::cout << count << "\n";
      avgDistance /= count;
      Real dist = (pd.getPosition(i) - com).norm();
      dist /= pd.getRadius(i);
      bool projCheck = avgDistance < 0.01;

      // std::cout << dist << "\n";
      if (dist < 1.2 || projCheck) {
        model.m_isBoundary[i] = true;
      } else {
        model.m_isBoundary[i] = false;
      }
      // Real projSumNorm = projSum.norm();

      // std::cout << avgDistance << "\n";

      // bool projCheck = (projSumNorm < 0.00001) && (projSumNorm > -0.00001);

      // if(dist > 0.001 || projCheck){
      //   model.m_isBoundary[i] = true;
      // }else{
      //   model.m_isBoundary[i] = false;
      // }
    }
  }
}

template <typename T> void quickDelete(unsigned int index, std::vector<T> &v) {
  v[index] = v.back();
  v.pop_back();
}

Vector3r samplePointSphere(Real radius, Vector3r &center, std::uniform_real_distribution<double> &distribution, std::mt19937 &generator){
  // Real theta = 2 * M_PI * distribution(generator);
  // Real phi = std::acos(1 - 2 * distribution(generator));
  // Real x = std::sin(phi) * std::cos(theta);
  // Real y = std::sin(phi) * std::sin(theta);
  // Real z = std::cos(phi);

  Real x = 0.0;
  Real y = 0.0;
  Real z = radius;
  Vector3r point = (Vector3r(x,y,z) - center).normalized();
  return point * radius;
}

void TimeStepGranularModel::merge2Particles(GranularModel &model,std::uniform_real_distribution<double> &distribution, std::mt19937 &generator) {
  ParticleData &pd = model.getParticles();
  unsigned int n = pd.size();
  unsigned short maxMergeCount = 3;
  unsigned short maxSplit= 1;
  unsigned short splitCount = 0;


  for (unsigned int i = 0; i < n; ++i) {

    if (!model.m_isBoundary[i] && pd.getIsActive(i) && !model.getMergeFlag(i) &&
        (pd.getMass(i) < model.maxMass)) {
      // if there are no neighbors then skip
      if (model.m_neighbors[i].size() == 0) {
        continue;
      }
      // get closest neighbor
      unsigned int closestParticle;
      Real distance = 10000.0;
      bool validNeighborPresent = false;
      // model.m_deleteFlag[i] = true;

      for (unsigned int particleIndex : model.m_neighbors[i]) {
        Real dist = (pd.getPosition(i) - pd.getPosition(particleIndex)).norm();
        // when there are neighbors but they are all marked for merging
        if ((dist < distance) && (!model.m_mergeFlag[particleIndex]) &&
            pd.getIsActive(particleIndex)) {
          distance = dist;
          closestParticle = particleIndex;
          validNeighborPresent = true;
        }
      }

      // check to see if any neighbors are valid
      if (!validNeighborPresent) {
        continue;
      }

      if (pd.getRadius(i) <= pd.getRadius(closestParticle)) {
        model.m_bigger.push_back(i);
        model.m_smaller.push_back(closestParticle);
      } else {
        model.m_bigger.push_back(closestParticle);
        model.m_smaller.push_back(i);
      }

      model.m_mergeCount.push_back(0);
      model.m_mergeFlag[i] = true;
      model.m_mergeFlag[closestParticle] = true;

      // std::cout << model.getMergeFlag(i) << "\n";
    } else if (model.m_isBoundary[i] && pd.getIsActive(i) &&
               !model.getMergeFlag(i) && (pd.getMass(i) > model.minMass)) {
      if (model.m_inactive.size() > 0) {
        if(splitCount < maxSplit){
          splitCount++;
          Real newMass = pd.getMass(i) * 0.5;
          Real newRadius = std::cbrt(pow(pd.getRadius(i) ,3) * 0.5);

          Vector3r newPos1 = samplePointSphere(newRadius, pd.getPosition(i),distribution, generator);
          // antipodal point
          Vector3r newPos2 = newPos1 * (-1.0);
          newPos1 = newPos1 + pd.getPosition(i);
          newPos2 = newPos2 + pd.getPosition(i);

          // get the index for an inactive particle
          unsigned int n = model.m_inactive.back();
          model.m_inactive.pop_back();
          // set the particle to new pos, mass, radius
          pd.setIsActive(n, true);
          pd.m_masses[n] = newMass;
          pd.m_radius[n] = newRadius;
          pd.m_x[n] = newPos1;
          pd.m_v[n] = pd.getVelocity(i);

          // set the current particle to the new pos, mass, and radius
          pd.m_masses[i] = newMass;
          pd.m_radius[i] = newRadius;
          pd.m_x[i] = newPos2;
        }
        // Vector3r(newRadius,newRadius,0); Vector3r newVel =
        // pd.getVelocity(i)*0.5; pd.m_masses[i] = newMass; pd.m_radius[i] =
        // newRadius; pd.m_v[i] = Vector3r(0.0,0.0,0.0); unsigned int n =
        // model.m_inactive.back(); pd.m_masses[n] = newMass; pd.m_radius[n] =
        // newRadius; pd.m_x[n] = newPos; pd.m_v[n] = Vector3r(0.0,0.0,0.0);
        // pd.setIsActive(n, true);
        // model.m_inactive.pop_back();

        //   Vector3r center = pd.getPosition(i);
        //   Vector3r boundsMin = center - model.samplingRadius;
        //   Vector3r boundsMax = center + model.samplingRadius;
        //   std::uniform_real_distribution<Real>
        //   distX(boundsMin.x(),boundsMax.x());
        //   std::uniform_real_distribution<Real>
        //   distY(boundsMin.y(),boundsMax.y());
        //   std::uniform_real_distribution<Real>
        //   distZ(boundsMin.z(),boundsMax.z());

        //   for(int i = 0; i < 10; ++i){
        //     Vector3r vec(distX(model.gen),distY(model.gen),distZ(model.gen));
        //     unsigned int index = model.m_inactiveUpsampled.back();
        //     model.m_isActiveUpsampled[index] = true;
        //     model.m_upsampledParticlesX[index] = vec;
        //     model.m_upsampledParticlesV[index] = Vector3r(0.0,0.0,0.0);
        //     model.m_inactiveUpsampled.pop_back();
        //   }
        //   // generate new upsampled neigbbors
      }
    }
  }

  for (int i = 0; i < model.m_bigger.size(); ++i) {
    unsigned int n_big = model.m_bigger[i];
    unsigned int n_small = model.m_smaller[i];

    if (model.m_mergeCount[i] < maxMergeCount) {
      Real ratio = (model.m_mergeCount[i] + 1) / (maxMergeCount + 1.0);
      Real deltaR = pow(pd.getRadius(n_small) * ratio, 3);
      Real deltaM = pd.getMass(n_small) * ratio;
      pd.m_radius[n_big] = std::cbrt(pow(pd.getRadius(n_big), 3) + deltaR);
      pd.m_radius[n_small] = std::cbrt(pow(pd.getRadius(n_small), 3) - deltaR);
      pd.m_masses[n_big] += deltaM;
      pd.m_masses[n_small] -= deltaM;
      model.m_mergeCount[i]++;
    } else {
      Real deltaR = pow(pd.getRadius(n_small), 3);
      Real deltaM = pd.getMass(n_small);
      pd.m_radius[n_big] = std::cbrt(pow(pd.getRadius(n_big), 3) + deltaR);
      pd.m_masses[n_big] += deltaM;
      model.m_mergeFlag[n_big] = false;
      model.m_mergeFlag[n_small] = false;
      pd.setIsActive(n_small, false);
      model.m_inactive.push_back(n_small);
      quickDelete(i, model.m_bigger);
      quickDelete(i, model.m_smaller);
      quickDelete(i, model.m_mergeCount);

      // Merge upsampled neigbbors
      int n_neighbors = model.m_neighborsUpsampled[n_small].size();
      unsigned int n = std::min(10, n_neighbors);
      for (int j = 0; j < n; ++j) {
        model.m_isActiveUpsampled[model.m_neighborsUpsampled[n_small][j]] =
            false;
        model.m_inactiveUpsampled.push_back(
            model.m_neighborsUpsampled[n_small][j]);
      }
    }
    // if(model.m_mergeCount[i] == 0){
    //   Real deltaR = pow(pd.getRadius(n_small)* 0.5 ,3);
    //   Real deltaM = pd.getMass(n_small) * 0.5;
    //   pd.m_radius[n_big]  =  std::cbrt(pow(pd.getRadius(n_big),3) + deltaR);
    //   pd.m_radius[n_small]  =  std::cbrt(pow(pd.getRadius(n_small),3) -
    //   deltaR); pd.m_masses[n_big] += deltaM; pd.m_masses[n_small] -= deltaM;
    //   model.m_mergeCount[i]++;
    // }else if(model.m_mergeCount[i] == 1){
    //   // std::cout << 1 << "\n";
    //   Real deltaR =   pow(pd.getRadius(n_small),3);
    //   Real deltaM = pd.getMass(n_small);
    //   pd.m_radius[n_big] = std::cbrt(pow(pd.getRadius(n_big),3) + deltaR);
    //   pd.m_masses[n_big] += deltaM;
    //   model.m_mergeFlag[n_big] = false;
    //   model.m_mergeFlag[n_small] = false;
    //   pd.setIsActive(n_small, false);
    //   model.m_inactive.push_back(n_small);
    //   quickDelete(i, model.m_bigger);
    //   quickDelete(i, model.m_smaller);
    //   quickDelete(i, model.m_mergeCount);

    //   int n_neighbors = model.m_neighborsUpsampled[n_small].size();
    //   unsigned int n = std::min(10,n_neighbors);
    //   for(int j = 0; j < n; ++j){
    //     model.m_isActiveUpsampled[model.m_neighborsUpsampled[n_small][j]] =
    //     false;
    //     model.m_inactiveUpsampled.push_back(model.m_neighborsUpsampled[n_small][j]);
    //   }
    // }
  }
}

void TimeStepGranularModel::applyForce(GranularModel &model) {
  ParticleData &pd = model.getParticles();

  for (int i = 0; i < pd.size(); ++i) {
    pd.m_v[i] += Vector3r(1.0, 0.0, 0.0);
  }
}

void TimeStepGranularModel::calculateAverageEnergy(GranularModel &model,
                                                   std::ofstream &file) {
  ParticleData &pd = model.getParticles();
  Real energy = 0;
  for (int i = 0; i < pd.size(); ++i) {
    energy += static_cast<Real>(0.5) * pd.getMass(i) *
              pd.getVelocity(i).norm() * pd.getVelocity(i).norm();
  }
  energy /= pd.size();
  file << energy << "\n";
}
