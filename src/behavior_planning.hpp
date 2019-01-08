#ifndef __BEHAVIOR_PLANNING_H__
#define __BEHAVIOR_PLANNING_H__
#include <iostream>
#include <vector>
#include <map>

using namespace std;

enum e_state {
  STATE_CS,
  STATE_KL,
  STATE_PLCL,
  STATE_PLCR,
  STATE_LCL,
  STATE_LCR
};

typedef struct {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double end_path_s;
  double end_path_d;
  unsigned int lane_id;
} EgoVehicle_st;

typedef struct {
  unsigned int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double speed;
  unsigned int lane_id;
} Vehicle_st;

const int lane_num = 3;
const int lane_dir[] = {0, 0, 1, -1, 1, -1};
const double acceleration = 0.224; // fixed acceleration
const double max_speed = 49.5; // fixed acceleration

class BehaviorPlanning {
 private:
  e_state ego_state;
  vector <e_state> traj_states;
  double expected_speed;
  double expected_lane;
  vector<e_state> successor_states(void);
  vector<EgoVehicle_st> generate_trajectory(const e_state cand_state, const EgoVehicle_st ego_vehicle,
                                         const vector<Vehicle_st> predictions,
                                         const double period=.02);

  vector<EgoVehicle_st> constant_speed_trajectory(const EgoVehicle_st ego_vehicle,
                                               const double period=.02);
  
  vector<EgoVehicle_st> keep_lane_trajectory(const EgoVehicle_st ego_vehicle,
                                             const vector<Vehicle_st> predictions,
                                             const double period=.02);
  
  vector<EgoVehicle_st> lane_change_trajectory(const e_state cand_state,
                                               const EgoVehicle_st ego_vehicle,
                                               const vector<Vehicle_st> predictions,
                                               const double period=.02);
  
  vector<EgoVehicle_st> prep_lane_change_trajectory(const e_state cand_state,
                                                    const EgoVehicle_st ego_vehicle,
                                                    const vector<Vehicle_st> predictions,
                                                    const double period=.02);
  vector<double> get_kinematics(const EgoVehicle_st ego_vehicle,
                                const vector<Vehicle_st> predictions,
                                const double period=.02);
  
  bool get_vehicle_behind(const EgoVehicle_st ego_vehicle,
                          const vector<Vehicle_st> predictions,
                          Vehicle_st &behind_vehicle);
  bool get_vehicle_ahead(const EgoVehicle_st ego_vehicle,
                         const vector<Vehicle_st> predictions,
                         Vehicle_st &ahead_vehicle);

 public:
  // constructor
  BehaviorPlanning();
  // destructor
  virtual ~BehaviorPlanning();

  /* @fn Prediction
     @param surr_vehicle Surrounding vehicle
   */
  vector<Vehicle_st> prediction(const vector<Vehicle_st> surr_vehicles, int horizon=2);
  
  void planning(const EgoVehicle_st ego_vehicle,
                const vector<Vehicle_st> predictions,
                const double period=.02);
  double get_expected_speed(void) { return expected_speed; };
  int get_expected_lane(void) { return expected_lane; };
  
};


#endif
