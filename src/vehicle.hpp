#ifndef __VEHICLE_HPP__
#define __VEHICLE_HPP__


#include <vector>
#include <cmath>
#include "state_machine.hpp"

class Vehicle {

public:
  StateMachine state;
  int id;
  int current_lane;
  int next_lane;
  // Cartesian Coordinates.
  double x;
  double y;
  double vx;
  double vy;
  double speed;
  double yaw;
  // Frenet
  double d;
  double s;
  double end_point_s;
  double end_point_d;

  Vehicle();
  virtual ~Vehicle();

  void set_parameter(int id, int lane, int next_lane,
                             double x, double y, double vx, double vy,
                             double speed, double yaw,
                             double d, double s,
                             double end_point_s, double end_point_d);

  void prediction(double time_period);

};

#endif
