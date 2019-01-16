#include "trajectory_planning.hpp"

TrajectoryPlanner::TrajectoryPlanner() {}
TrajectoryPlanner::~TrajectoryPlanner() {}

Trajectory TrajectoryPlanner::trajectory_planning(EgoVehicle ego_vehicle, vector<vector<double>> sensor_fusion,
                               double time_step) {

  Prediction prediction;
  prediction.read_sensor_fusion(sensor_fusion);
  prediction.prediction(time_step);

  int ego_lane = ego_vehicle.current_lane;
  SurroundingVehicle front_vehicle;
  SurroundingVehicle behind_vehicle;
  bool ego_exist = get_vehicle_front(ego_vehicle, prediction, ego_lane, front_vehicle);
  cout << "ego lane obstacle exist: " << ego_exist << endl;
  int right_lane = ego_lane + 1;
  int left_lane = ego_lane - 1;

  bool right_exist = false;
  bool right_exist_front = get_vehicle_front(ego_vehicle, prediction, right_lane, front_vehicle);
  bool right_exist_behind = get_vehicle_behind(ego_vehicle, prediction, right_lane, behind_vehicle);
  bool left_exist = false;
  bool left_exist_front = get_vehicle_front(ego_vehicle, prediction, left_lane, front_vehicle);
  bool left_exist_behind = get_vehicle_front(ego_vehicle, prediction, left_lane, front_vehicle);

  if (right_exist_front & right_exist_behind) {
    right_exist = true;
  }

  if (left_exist_front & left_exist_behind) {
    left_exist = true;
  }

  Trajectory traj = generate_lane_keep(ego_vehicle, prediction, STATE_LK);
  Trajectory lcl_traj = generate_lane_change(ego_vehicle, prediction, STATE_LCL);
  Trajectory lcr_traj = generate_lane_change(ego_vehicle, prediction, STATE_LCR);
  if (ego_exist) {
    if (!left_exist && (lcl_traj.state != STATE_INVALID)) {
      cout << "<< ----left lane change----" << endl;
      traj = lcl_traj;
    }
    else if (!right_exist && (lcr_traj.state != STATE_INVALID)) {
      cout << "<< ----right lane change----" << endl;
      traj = lcr_traj;
    }
    else {
      cout << " ---- lane keep ----  " << endl;
    }
  } else {
      cout << " ---- lane keep ----  " << endl;
  }
  return traj;

  
}
