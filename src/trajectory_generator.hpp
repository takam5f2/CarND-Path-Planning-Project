#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include "ego_vehicle.hpp"
#include "prediction.hpp"
#include "trajectory.hpp"
#include <vector>
#include <map>

using namespace std;

Trajectory generate_lane_keep(const EgoVehicle ego_vehicle,
                              const Prediction prediction,
                              PARA_STATE next_state);

Trajectory generate_lane_change(const EgoVehicle ego_vehicle,
                                const Prediction prediction,
                                PARA_STATE next_state);

bool get_vehicle_front(const EgoVehicle ego_vehicle,
                       const Prediction prediction,
                       int next_lane,
                       SurroundingVehicle & front_vehicle);

bool get_vehicle_behind(const EgoVehicle ego_vehicle,
                       const Prediction prediction,
                       int next_lane,
                       SurroundingVehicle & behind_vehicle);



#endif
