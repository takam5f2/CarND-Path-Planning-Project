#include "cost_function.hpp"
#include "behavior_planning.hpp"
#include <functional>
#include <iterator>
#include <map>
#include <cmath>

const double REACH_GOAL = pow(10, 6);
const double EFFICIENCY = pow(10, 5);

double goal_distance_cost(const EgoVehicle_st & ego_vehicle, const vector<Vehicle_st> & predictions,
                          const vector<EgoVehicle_st> & trajectory, const TrajData_st & traj_data){
  double cost;
  double distance = traj_data.distance_to_goal;
  int goal_lane = 1;

  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*(double)goal_lane - (double)traj_data.intended_lane - (double)traj_data.final_lane)
                       / (distance+1E-5)));
  } else {
    cost = 1;
  }
  return cost;
}

double inefficiency_cost(const EgoVehicle_st & ego_vehicle, const vector<Vehicle_st> & predictions,
                         const vector<EgoVehicle_st> & trajectory, const TrajData_st & traj_data) {
  
  double proposed_speed_intended = lane_speed(predictions, traj_data.intended_lane);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = ego_vehicle.speed;
  }

  double proposed_speed_final = lane_speed(predictions, traj_data.final_lane);
  if (proposed_speed_final < 0) {
    proposed_speed_intended = ego_vehicle.speed;
  }

  double cost = (2.0* ego_vehicle.speed - proposed_speed_intended - proposed_speed_final);
  return cost;
}


double lane_speed(const vector<Vehicle_st> & predictions, int lane) {
  
  for (int i = 0; i < predictions.size(); i++) {
    if (predictions[i].lane_id == lane) {
      return predictions[i].speed;
    }
  }
  return -1.0;
}

double calculate_cost(const EgoVehicle_st & ego_vehicle, const vector<Vehicle_st> & predictions,
                      const vector<EgoVehicle_st> & trajectory, const vector<e_state> & traj_states) {
  TrajData_st trajectory_data = get_helper_data(ego_vehicle, trajectory, traj_states, predictions);
  double cost = 0.0;

  vector< function<double(const EgoVehicle_st &, const vector<Vehicle_st> &,
                          const vector<EgoVehicle_st> &, const TrajData_st &)>> cf_list = {goal_distance_cost, inefficiency_cost};
  vector<double> weight_list = {REACH_GOAL, EFFICIENCY};

  for (int i = 0; i < cf_list.size(); i++) {
    double new_cost = weight_list[i] * cf_list[i](ego_vehicle, predictions, trajectory, trajectory_data);
    cost += new_cost;
  }
  return cost;
}

TrajData_st get_helper_data(const EgoVehicle_st & ego_vehicle,
                                    const vector<EgoVehicle_st> & trajectory,
                                    const vector<e_state> & traj_states,
                                    const vector<Vehicle_st> & predictions) {
  TrajData_st trajectory_data;

  EgoVehicle_st trajectory_last = trajectory[1];
  e_state traj_state = traj_states[1];
  int intended_lane;
  if (traj_state == STATE_PLCL) {
    intended_lane = trajectory_last.lane_id + 1;
  } else if (traj_state == STATE_PLCR) {
    intended_lane = trajectory_last.lane_id - 1;
  } else {
    intended_lane = trajectory_last.lane_id;
  }
  
  double distance_to_goal = ego_vehicle.end_path_d - trajectory_last.s;
  int final_lane = trajectory_last.lane_id;
  trajectory_data.intended_lane = intended_lane;
  trajectory_data.final_lane = final_lane;
  trajectory_data.distance_to_goal = distance_to_goal;
  return trajectory_data;
}


