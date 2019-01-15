#ifndef __SURROUNDING_VEHICLE_HPP__
#define __SURROUNDING_VEHICLE_HPP__

#include "vehicle.hpp"

class SurroundingVehicle : public Vehicle {
public:
  SurroundingVehicle();
  virtual ~SurroundingVehicle();
  
  void set_parameter(int id, double x, double y,
                     double vx, double vy, double d, double s);
};

#endif
