#ifndef __BEHAVIOR_HPP__
#define __BEHAVIOR_HPP__

#include "vehicle.hpp"

class Behavior {
public:
  PARA_STATE state;
  int lane;
  double speed;
  double next_s;
  double next_d;

  Behavior(PARA_STATE _state, int _lane, double _speed,
             double _next_s, double _next_d);
  
  virtual ~Behavior();
};

#endif
