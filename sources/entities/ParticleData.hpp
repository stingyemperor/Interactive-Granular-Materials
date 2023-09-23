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
    std::vector<Vector3r> m_oldX;
    std::vector<Vector3r> m_v;
    std::vector<Vector3r> m_a;
    std::vector<Real> m_radius;
    std::vector<bool> m_isActive;

    ParticleData(void) : m_masses(),m_invMasses(),m_x0(),m_x(),m_v(),m_a(){}
    ~ParticleData(void){
      m_masses.clear();
      m_invMasses.clear();
      m_x0.clear();
      m_x.clear();
      m_oldX.clear();
      m_v.clear();
      m_a.clear();
      m_radius.clear();
      m_isActive.clear();
    }

    void addVertex(const Vector3r &vertex){
      m_masses.push_back(static_cast<Real>(1.0f));
      m_invMasses.push_back(static_cast<Real>(1.0f));
      m_x0.push_back(vertex);
      m_x.push_back(vertex);
      m_oldX.push_back(vertex);
      m_v.push_back(Vector3r(0.0,0.0,0.0));
      m_a.push_back(Vector3r(0.0,0.0,0.0));
      m_radius.push_back(static_cast<Real>(1.0));
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

    Vector3r &getOldPosition(const unsigned int i){
      return m_oldX[i];
    }

    void setPosition(const unsigned int i, const Vector3r &pos){
      m_x[i] = pos;
    }

    void setPosition0(const unsigned int i, const Vector3r &pos){
      m_x0[i] = pos;
    }
    
    void setOldPosition(const unsigned int i, const Vector3r &pos){
      m_oldX[i] = pos;   
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

    Real &getRadius(const unsigned int i){
      return m_radius[i];
    }

    void setRadius(const unsigned int i, const Real radius){
      m_radius[i] = radius;
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
    
    void setIsActive(const unsigned int i , const bool b){
      m_isActive[i] = b;
    }
    
    bool getIsActive(const unsigned int i){
      return m_isActive[i]; 
    }

    void resize(const unsigned int newSize){
      m_masses.resize(newSize);
      m_invMasses.resize(newSize);
      m_x0.resize(newSize);
      m_x.resize(newSize);
      m_oldX.resize(newSize);
      m_v.resize(newSize);
      m_a.resize(newSize);
      m_radius.resize(newSize);
      m_isActive.resize(newSize);
    }

    void release(){
      m_masses.clear();
      m_invMasses.clear();
      m_x.clear();
      m_x0.clear();
      m_oldX.clear();
      m_v.clear();
      m_a.clear();
      m_radius.clear();
      m_isActive.clear();
    }

    void removeLastElement(){
      m_masses.pop_back();
      m_invMasses.pop_back();
      m_x.pop_back();
      m_x0.pop_back();
      m_oldX.pop_back();
      m_v.pop_back();
      m_a.pop_back();
      m_radius.pop_back();
    }
    
    template <typename T> void quickDelete(unsigned int index, std::vector<T> &v){
      v[index] = v.back();
      v.pop_back();
    }  

    void removeElement(const unsigned int index){
      quickDelete(index, m_x);
      quickDelete(index, m_oldX);
      quickDelete(index, m_v);
      quickDelete(index, m_a);
      quickDelete(index, m_masses);
      quickDelete(index, m_invMasses);
      quickDelete(index, m_radius);
      // quickDelete(index, model.m_mergeFlag);
      // quickDelete(index, model.m_neighbors);
      // quickDelete(index, model.m_isBoundary);
    }

    // void removeElement(const unsigned int &index, std::unordered_map<unsigned int,unsigned int> &map, const unsigned int &lastIndex, std::vector<bool> &m_mergeFlag, std::vector<bool> &m_deleteFlag){

    //   unsigned int realIndex;
    //   if(map.find(index) == map.end()){
    //     realIndex = index;
    //   }else{
    //     realIndex = map[index];
    //   }
    //   
    //   map[lastIndex] = realIndex;
    //   // add to map

    //   auto itX = m_x.begin() + realIndex;
    //   // auto itX_0 = m_x0.begin() + index;
    //   auto itOld_x = m_oldX.begin() + realIndex;
    //   auto itMasses = m_masses.begin() + realIndex;
    //   auto itInvMasses = m_invMasses.begin() + realIndex;
    //   auto itV = m_v.begin() + realIndex;
    //   auto itA = m_a.begin() + realIndex;
    //   auto itRadius = m_radius.begin() + realIndex;
    //   auto itMerge = m_mergeFlag.begin() + realIndex; 
    //   auto itDelete = m_deleteFlag.begin() + realIndex; 

    //   *itX = std::move(m_x.back());
    //   // *itX_0 = std::move(m_x0.back());
    //   *itOld_x = std::move(m_oldX.back());
    //   *itMasses = std::move(m_masses.back());
    //   *itInvMasses = std::move(m_invMasses.back());
    //   *itV = std::move(m_v.back());
    //   *itA = std::move(m_a.back());
    //   *itRadius = std::move(m_radius.back());
    //   *itMerge = std::move(m_mergeFlag.back());
    //   *itDelete = std::move(m_deleteFlag.back());

    //   m_x.pop_back();
    //   m_oldX.pop_back();
    //   m_masses.pop_back();
    //   m_invMasses.pop_back();
    //   m_v.pop_back();
    //   m_a.pop_back();
    //   m_radius.pop_back();
    //   m_mergeFlag.pop_back();
    //   m_deleteFlag.pop_back();
    // }

    void addElement(const Vector3r &pos, const Vector3r &vel, const Real &mass, const Real &radius){
      m_x.push_back(pos); 
      m_oldX.push_back(pos);
      m_masses.push_back(mass);
      m_invMasses.push_back(static_cast<Real>(1.0)/mass);
      m_v.push_back(vel);
      m_a.push_back(Vector3r(0.0,-9.81,0.0)); 
      m_radius.push_back(radius);
    }
  };
}

