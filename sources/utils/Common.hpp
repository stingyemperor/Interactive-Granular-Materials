#pragma once

#include <eigen3/Eigen/Dense>
#include <float.h>

typedef float Real;
#define REAL_MAX FLT_MAX
#define REAL_MIN FLT_MIN
#define FORCE_INLINE __attribute__((always_inline))

using Vector2r = Eigen::Matrix<Real,2,1,Eigen::DontAlign>;
using Vector3r = Eigen::Matrix<Real,3,1,Eigen::DontAlign>;
using Vector4r = Eigen::Matrix<Real,4,1,Eigen::DontAlign>;
using Vector2i = Eigen::Matrix<int ,2,1,Eigen::DontAlign>;






