#pragma once
#include <eigen3/Eigen/Core>
#include "./utils/Common.hpp"
#include <glm/glm.hpp>
#include <vector>
#define max_count 1000

namespace PBD{

  /** This class encapsulates the state of all particles
  */
  struct ParticleData{
    std::vector<Real> m_masses;
    std::vector<Real> m_invMasses;
    std::vector<Vector3r> m_x0;
    std::vector<Vector3r> m_x;
    std::vector<Vector3r> m_v;
    std::vector<Vector3r> m_a;

    ParticleData(void) : m_masses(),m_invMasses(),m_x0(),m_x(),m_v(),m_a(){}
    ~ParticleData(void){
      m_masses.clear();
      m_invMasses.clear();
      m_x0.clear();
      m_x.clear();
      m_v.clear();
      m_a.clear();
    }

    void addVertex(const Vector3r &vertex){
      m_masses.push_back(1.0f);
      m_invMasses.push_back(1.0f);
      m_x0.push_back(vertex);
      m_x.push_back(vertex);
      m_v.push_back(Vector3r(0.0,0.0,0.0));
      m_a.push_back(Vector3r(0.0,0.0,0.0));
    }

    Real &getMass(const unsigned int i){
      return m_masses[i];
    }

    Real &getInvMass(const unsigned int i){
      return m_invMasses[i];
    }

    void setMass(const unsigned int i, const Real mass){
      m_masses[i] = mass;
    }

    void setInvMass(const unsigned int i, const Real mass){
      m_invMasses[i] = mass;
    }

    Vector3r &getPosition(const unsigned int i){
      return m_x[i];
    }

    Vector3r &getPosition0(const unsigned int i){
      return m_x0[i];
    }

    void setPosition(const unsigned int i, const Vector3r &pos){
      m_x[i] = pos;
    }

    void setPosition0(const unsigned int i, const Vector3r &pos){
      m_x0[i] = pos;
    }

    Vector3r &getVelocity(const unsigned int i){
      return m_v[i];
    }

    Vector3r &getAcceleration(const unsigned int i){
      return m_a[i];
    }

    void setVelocity(const unsigned int i,const Vector3r &vel){
      m_v[i] = vel;
    }

    void setAcceleration(const unsigned int i,const Vector3r &accel){
      m_v[i] = accel;
    }

    const unsigned int getNumberOfParticles() const{
      return (unsigned int)m_x.size();
    }

    const std::vector<Vector3r>& getVertices() const{
      return m_x;
    }

    unsigned int size() const{
      return (unsigned int)m_x.size();
    }

    void resize(const unsigned int newSize){
      m_masses.resize(newSize);
      m_invMasses.resize(newSize);
      m_x0.resize(newSize);
      m_x.resize(newSize);
      m_v.resize(newSize);
      m_a.resize(newSize);
    }

    void release(){
      m_masses.clear();
      m_invMasses.clear();
      m_x.clear();
      m_x0.clear();
      m_v.clear();
      m_a.clear();
    }
  };

}

