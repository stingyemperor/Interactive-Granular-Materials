#pragma once

#if defined(__linux__)
#include <eigen3/Eigen/Dense>
#include <omp.h>
#elif defined(__APPLE__)
 #include "/usr/local/include/libomp/17.0.1/include/omp.h"
// #include <omp.h>
#include <Eigen/Dense>
#endif

#include <float.h>
#include <iostream>

// // typedef float Real;
typedef double Real;

#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN
#define RealParameter DoubleParameter
#define RealParameterType ParameterBase::DOUBLE
#define RealVectorParameter DoubleVectorParameter
#define RealVectorParameterType ParameterBase::VEC_DOUBLEv

// typedef float Real;
// #define REAL_MAX FLT_MAX
// #define REAL_MIN FLT_MIN

#define FORCE_INLINE __attribute__((always_inline))

using Vector2r = Eigen::Matrix<Real, 2, 1, Eigen::DontAlign>;
using Vector3r = Eigen::Matrix<Real, 3, 1>;
using Vector4r = Eigen::Matrix<Real, 4, 1, Eigen::DontAlign>;
using Vector2i = Eigen::Matrix<int, 2, 1, Eigen::DontAlign>;
using Vector3i = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>;
using Matrix3rx = Eigen::Matrix<Real, 3, Eigen::Dynamic>;
using Matrix3ix = Eigen::Matrix<unsigned int, 3, Eigen::Dynamic>;
// using Vector3x = Eigen::Matrix<Real, 3, 1>;

// void printVector3(Vector3r &v){
//   std::cout << "(" << v.x() << "," << v.y() << "," << v.z() << ")\n";
// }
