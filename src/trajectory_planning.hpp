#ifndef __TRAJECTORY_PLANNING__
#define __TRAJECTORY_PLANNING__

#include "ego_vehicle.hpp"
#include "prediction.hpp"
#include "trajectory_generator.hpp"

class TrajectoryPlanner {
public:
  TrajectoryPlanner();
  ~TrajectoryPlanner();
  Trajectory trajectory_planning(EgoVehicle ego_vehicle,
                                 vector<vector<double>> sensor_fusion, double time_step);
  double calculate_cost(EgoVehicle ego_vehicle, Prediction prediction, Trajectory trajectory);
};
#endif
