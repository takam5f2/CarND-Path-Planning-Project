#ifndef __COST_FUNCTION_HPP__
#define __COST_FUNCTION_HPP__
#include "behavior_planning.hpp"

using namespace std;

typedef struct {
  int intended_lane;
  int final_lane;
  double distance_to_goal;
}TrajData_st;

double calculate_cost(const EgoVehicle_st & ego_vehicle, const vector<Vehicle_st> & predictions,
                      const vector<EgoVehicle_st> & trajectory, const vector<e_state> & traj_states);

double goal_distance_cost(const EgoVehicle_st & ego_vehicle, const vector<Vehicle_st> & predictions,
                          const vector<EgoVehicle_st> & trajectory, const TrajData_st & traj_data);

double inefficiency_cost(const EgoVehicle_st & ego_vehicle, const vector<Vehicle_st> & predictions,
                         const vector<EgoVehicle_st> & trajectory, const TrajData_st & traj_data);

double lane_speed(const vector<Vehicle_st> & predictions, int lane);

TrajData_st get_helper_data(const EgoVehicle_st & ego_vehicle,
                                    const vector<EgoVehicle_st> & trajectory,
                                    const vector<e_state> & traj_states,
                                    const vector<Vehicle_st> & predictions);

#endif
