#ifndef __BEHAVIOR_PLANNING__
#define __BEHAVIOR_PLANNING__

#include "ego_vehicle.hpp"
#include "prediction.hpp"
#include "behavior_generator.hpp"

class BehaviorPlanner {
public:
  BehaviorPlanner();
  ~BehaviorPlanner();
  Behavior behavior_planning(EgoVehicle ego_vehicle,
                                 vector<vector<double>> sensor_fusion, double time_step);
};
#endif
