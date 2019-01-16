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

  
  /*
  vector<Trajectory> trajectory = generate_trajectory(ego_vehicle, prediction);
   double min_cost = 2000.0;
  Trajectory chosen_traj(STATE_LK, ego_vehicle.current_lane, ego_vehicle.ref_speed,
                         ego_vehicle.s, ego_vehicle.d);

  for (int i = 0; i < trajectory.size(); i++) {
    double cost = calculate_cost(ego_vehicle, prediction, trajectory[i]);

    if (min_cost > cost) {
      min_cost = cost;
      chosen_traj = trajectory[i];
    }
  }
  cout << "chosen next state is: " << chosen_traj.state << endl;
  cout << "next lane is: " << chosen_traj.lane << endl;
  return chosen_traj;
  */
}

double TrajectoryPlanner::calculate_cost(EgoVehicle ego_vehicle, Prediction prediction, Trajectory trajectory){

  // lane cost
  double cost = 0.0;
  const double BIG_COST = 1000.0;
  if (trajectory.state == STATE_INVALID) {
    cost += BIG_COST;
  }

  SurroundingVehicle front_vehicle;
  SurroundingVehicle behind_vehicle;
  bool front_exist_ego = get_vehicle_front(ego_vehicle, prediction, ego_vehicle.current_lane,
                                           front_vehicle);
  // redundant lane change cost
  /*
  if (trajectory.lane != ego_vehicle.current_lane
      && !front_exist_ego) {
    cost += 1.0;
  }
  */

  // stay same lane even though side lane is not occupied.
  if (trajectory.state == STATE_LK && front_exist_ego) {
    cost += 1.0;
  }
  
  bool front_exist_target_lane = get_vehicle_front(ego_vehicle, prediction, trajectory.lane,
                                           front_vehicle);
  bool behind_exist_target_lane = get_vehicle_behind(ego_vehicle, prediction, trajectory.lane,
                                           behind_vehicle);

  // collide other vehicle after lane change.
  if ((front_exist_target_lane || behind_exist_target_lane)
      && (trajectory.state != STATE_LK)) {
    cost += 1.0;
  }

  // speed doesn't match target speed
  if (trajectory.speed < (ego_vehicle.speed - 3.0)) {
    cost += 1.0;
  }

  return cost;
}
