#ifndef __EGO_VEHICLE_HPP__
#define __EGO_VEHICLE_HPP__

#include "vehicle.hpp"
#include "helper_function.hpp"

class EgoVehicle : public Vehicle {
public:
  double ref_speed;
  EgoVehicle();
  virtual ~EgoVehicle();
  void init_state(PARA_STATE given_state) { this->state.set_state(given_state); };
  
  vector<PARA_STATE> get_next_state_candidate(void) {
    return this->state.get_next_state_candidate();
  }
  
  void set_parameter(int lane, double ref_speed,
                     double x, double y, double speed,
                     double yaw, double d, double s,
                     double end_point_s, double end_point_d);
};

#endif
