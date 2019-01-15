#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include "state_machine.hpp"

class Trajectory {
public:
  PARA_STATE state;
  int lane;
  double speed;
  double next_s;
  double next_d;

  Trajectory(PARA_STATE _state, int _lane, double _speed,
             double _next_s, double _next_d);
  
  virtual ~Trajectory();
};

#endif
