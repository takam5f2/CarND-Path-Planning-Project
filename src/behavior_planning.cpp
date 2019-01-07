#include "behavior_planning.hpp"
#include <algorithm>
#include <cmath>
#include <cassert>

BehaviorPlanning::BehaviorPlanning()
:ego_state(STATE_KL)
{
}

BehaviorPlanning::~BehaviorPlanning(){}


vector<Vehicle_st> BehaviorPlanning::prediction(const vector<Vehicle_st> surr_vehicles, int horizon) {
  vector<Vehicle_st> predictions;

  for (int i = 0; i < surr_vehicles.size(); i++) {
    Vehicle_st pred_vehicle = surr_vehicles[i];
    double vx = pred_vehicle.vx;
    double vy = pred_vehicle.vx;
    pred_vehicle.speed = sqrt((vx*vx)+(vy*vy));
    pred_vehicle.s += (double)horizon * 0.02 * pred_vehicle.speed;

    for (unsigned int j = 0; j < lane_num; j++) {
      if (pred_vehicle.d < (2+4*j+2) && pred_vehicle.d > (2+4*j-2)) {
        pred_vehicle.lane_id = j;
        break;
      }
    } // for (unsigned int j = 0; j < lane_num; j++)
    predictions.push_back(pred_vehicle);
  }
  
  return predictions;
}

void BehaviorPlanning::planning(const EgoVehicle_st ego_vehicle,
                                const vector<Vehicle_st> predictions,
                                const double period) {
  vector<e_state> states = successor_states();

  double cost;
  vector<double> traj_costs;
  vector<vector<EgoVehicle_st>> final_trajectories;

  for (vector<e_state>::iterator it = states.begin(); it != states.end(); ++it) {
    traj_states.clear();
    vector<EgoVehicle_st> trajectory = generate_trajectory(*it, ego_vehicle,
                                                        predictions, period);
    assert(traj_states.size() == trajectory.size());
    if (trajectory.size() != 0) {
      cost = calculate_cost(ego_vehicle, predictions, trajectory);
      traj_costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }

  }
  vector<double>::iterator best_cost = min_element(begin(traj_costs), end(traj_costs));
  int best_idx = distance(begin(traj_costs), best_cost);
  expected_speed =  final_trajectories[best_idx][1].speed;
  expected_lane =  final_trajectories[best_idx][1].lane_id;
}

vector<e_state> BehaviorPlanning::successor_states(void) {
  vector<e_state> candidate_states;
  candidate_states.push_back(STATE_KL);

  if (this->ego_state == STATE_KL) {
    candidate_states.push_back(STATE_PLCL);
    candidate_states.push_back(STATE_PLCR);
  }
  else if (this->ego_state == STATE_PLCL) {
    candidate_states.push_back(STATE_PLCL);
    candidate_states.push_back(STATE_LCL);
  }
  else if (this->ego_state == STATE_PLCR) {
    candidate_states.push_back(STATE_PLCR);
    candidate_states.push_back(STATE_LCR);
  }

  return candidate_states;
}

vector<EgoVehicle_st> BehaviorPlanning::generate_trajectory(const e_state cand_state,
                                                            const EgoVehicle_st ego_vehicle,
                                                            const vector<Vehicle_st> predictions,
                                                            const double period) {
  vector<EgoVehicle_st> trajectory;

  if (cand_state == STATE_CS) {
    trajectory = constant_speed_trajectory(ego_vehicle, period);
  }
  else if (cand_state == STATE_KL) {
    trajectory = keep_lane_trajectory(ego_vehicle, predictions, period);
  }
  else if (cand_state == STATE_LCL || cand_state == STATE_LCR) {
    trajectory = lane_change_trajectory(cand_state, ego_vehicle, predictions, period);
  }
  else if (cand_state == STATE_PLCL || cand_state == STATE_PLCR) {
    trajectory = prep_lane_change_trajectory(cand_state, ego_vehicle, predictions, period);
  }

  return trajectory;
}

vector<EgoVehicle_st> BehaviorPlanning::constant_speed_trajectory(const EgoVehicle_st ego_vehicle,
                                                                  const double period){
  vector<EgoVehicle_st> trajectory;
  double next_pos = ego_vehicle.s + period * ego_vehicle.speed;
  EgoVehicle_st tmp_ego = ego_vehicle;

  // copy with acceleration
  tmp_ego.s = next_pos;
  tmp_ego.speed += acceleration;
  trajectory.push_back(tmp_ego);
  traj_states.push_back(STATE_CS);
  // copy without acceleration
  tmp_ego = ego_vehicle;
  tmp_ego.s = next_pos;
  trajectory.push_back(tmp_ego);
  traj_states.push_back(STATE_CS);
  return trajectory;
}

vector<EgoVehicle_st> BehaviorPlanning::keep_lane_trajectory(const EgoVehicle_st ego_vehicle,
                                                             const vector<Vehicle_st> predictions,
                                                             const double period){
  vector<EgoVehicle_st> trajectory = {ego_vehicle};
  traj_states.push_back(ego_state);

  vector<double> kinematics = get_kinematics(ego_vehicle, predictions, period);
  double new_s = kinematics[0];
  double new_speed = kinematics[1];

  EgoVehicle_st tmp_ego = ego_vehicle;
  tmp_ego.s = new_s;
  tmp_ego.speed = new_speed;
  trajectory.push_back(tmp_ego);
  traj_states.push_back(STATE_KL);
  return trajectory;
}

vector<EgoVehicle_st> BehaviorPlanning::prep_lane_change_trajectory(const e_state cand_state,
                                                                    const EgoVehicle_st ego_vehicle,
                                                                    const vector<Vehicle_st> predictions,
                                                                    const double period) {
  float new_s;
  float new_v;

  Vehicle_st vehicle_behind;

  int new_lane = ego_vehicle.lane_id + lane_dir[cand_state];

  vector<EgoVehicle_st> trajectory = {ego_vehicle};
  traj_states.push_back(ego_state);

  vector<double> curr_lane_new_kinematics = get_kinematics(ego_vehicle, predictions, period);
  EgoVehicle_st ego_on_next_lane = ego_vehicle;
  ego_on_next_lane.lane_id = new_lane;
  vector<double> next_lane_new_kinematics = get_kinematics(ego_on_next_lane, predictions, period);

  if (get_vehicle_behind(ego_vehicle, predictions, vehicle_behind)) {
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
  }
  else {
    vector<double> best_kinematics;
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    }
    else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
  }
  EgoVehicle_st new_ego_vehicle = ego_vehicle;
  new_ego_vehicle.s = new_s;
  new_ego_vehicle.y = new_v;

  trajectory.push_back(new_ego_vehicle);
  traj_states.push_back(cand_state);
  return trajectory;
}

vector<EgoVehicle_st> BehaviorPlanning::lane_change_trajectory(const e_state cand_state,
                                                               const EgoVehicle_st ego_vehicle,
                                                               const vector<Vehicle_st> predictions,
                                                               const double period) {
  int new_lane = ego_vehicle.lane_id + lane_dir[cand_state];
  vector<EgoVehicle_st> trajectory;

  Vehicle_st next_lane_vehicle;
  
  EgoVehicle_st ego_on_next_lane;
  ego_on_next_lane.lane_id = new_lane;
  Vehicle_st ahead_vehicle, behind_vehicle;
  if (get_vehicle_ahead(ego_vehicle, predictions, ahead_vehicle)
      || get_vehicle_ahead(ego_vehicle, predictions, behind_vehicle)) {
    return trajectory;
  }

  trajectory.push_back(ego_vehicle);
  traj_states.push_back(ego_state);

  EgoVehicle_st new_ego_vehicle;
  new_ego_vehicle.lane_id = new_lane;
  vector<double> kinematics = get_kinematics(ego_vehicle, predictions, period);
  new_ego_vehicle.s = kinematics[0];
  new_ego_vehicle.speed = kinematics[1];
  
  trajectory.push_back(new_ego_vehicle);
  traj_states.push_back(cand_state);
  return trajectory;
}


vector<double> BehaviorPlanning::get_kinematics(const EgoVehicle_st ego_vehicle,
                                                const vector<Vehicle_st> predictions,
                                                const double period){
  vector<double> kinematics;
  double max_accel_velocity = acceleration + ego_vehicle.speed;
  double new_position;
  double new_velocity;

  Vehicle_st vehicle_ahead;
  Vehicle_st vehicle_behind;

  if (get_vehicle_ahead(ego_vehicle, predictions, vehicle_ahead)) {
    if (get_vehicle_behind(ego_vehicle, predictions, vehicle_behind)) {
      new_velocity = vehicle_ahead.speed;
    } else {
      double max_velocity_in_front = (vehicle_ahead.s - ego_vehicle.s) / (period + 1E-4)
        + vehicle_ahead.speed;
      new_velocity = min(max_velocity_in_front, max_accel_velocity);
      new_velocity = min(new_velocity, max_speed);
    }
  }
  else {
    new_velocity = min(max_accel_velocity, max_speed);
  }
  
  new_position = ego_vehicle.s + new_velocity * period;
  
  return {new_position, new_velocity};

}

bool BehaviorPlanning::get_vehicle_behind(const EgoVehicle_st ego_vehicle,
                                          const vector<Vehicle_st> predictions,
                                          Vehicle_st &behind_vehicle) {
  int max_s = -1; // tuning parameter
  bool found_vehicle = false;

  for (int i = 0; i < predictions.size(); i++) {
    if (predictions[i].lane_id == ego_vehicle.lane_id
        && predictions[i].s < ego_vehicle.s
        && predictions[i].s > max_s) {
      max_s = predictions[i].s;
      behind_vehicle = predictions[i];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}
bool BehaviorPlanning::get_vehicle_ahead(const EgoVehicle_st ego_vehicle,
                                         const vector<Vehicle_st> predictions,
                                         Vehicle_st &ahead_vehicle) {
  int min_s = ego_vehicle.end_path_s;
  bool found_vehicle = false;
  for (int i = 0; i < predictions.size(); i++) {
    if (predictions[i].lane_id == ego_vehicle.lane_id
        && predictions[i].s < min_s
        && predictions[i].s > ego_vehicle.s) {
      min_s = predictions[i].s;
      ahead_vehicle = predictions[i];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}
