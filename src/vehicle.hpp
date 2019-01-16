#ifndef __VEHICLE_HPP__
#define __VEHICLE_HPP__


#include <vector>
#include <cmath>

// state
enum PARA_STATE {
  STATE_LK, // lane keeping
  STATE_PLCL, // preparing lane change to left
  STATE_PLCR, // preparing lane change to right
  STATE_LCL, // lane change to left lane
  STATE_LCR, // lane change to right lane
  STATE_INVALID // Invalid state.
};

class Vehicle {

public:
  PARA_STATE state;
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
