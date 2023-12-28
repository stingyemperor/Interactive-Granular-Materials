#pragma once
#include "../utils/Common.hpp"

namespace PBD {
struct TimeManager {
  Real time;
  static TimeManager *current;
  Real h;

  TimeManager();
  ~TimeManager();

  static TimeManager *getCurrent();
  static void setCurrent(TimeManager *tm);
  static bool hasCurrent();

  Real getTime();
  void setTime(Real t);
  Real getTimeStepSize();
  void setTimeStepSize(Real tss);
};
} // namespace PBD
