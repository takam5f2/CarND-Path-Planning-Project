#include "trajectory_generator.hpp"
#include "prediction.hpp"

// constant value;
const double TARGET_SPEED = 49.5;
const double ACCELERATION = 0.224;
const int LANE_LEFT = 0;
const int LANE_CENTER = 1;
const int LANE_RIGHT = 2;

Trajectory generate_lane_keep(const EgoVehicle ego_vehicle,
                              const Prediction prediction,
                              PARA_STATE next_state) {
  int lane = ego_vehicle.current_lane;
  int too_close = false;

  vector<SurroundingVehicle> vehicles = prediction.surrounding_vehicles;

  SurroundingVehicle front_vehicle;
  bool front_exist = get_vehicle_front(ego_vehicle, prediction, lane, front_vehicle);
  
  Trajectory trajectory(next_state, ego_vehicle.current_lane,
                        ego_vehicle.speed, ego_vehicle.s, ego_vehicle.d);

  if (front_exist) {
    trajectory.speed = ego_vehicle.ref_speed - ACCELERATION;
  } else if (ego_vehicle.ref_speed < TARGET_SPEED) {
    trajectory.speed = ego_vehicle.ref_speed + ACCELERATION;
  }

  return trajectory;
}


Trajectory generate_lane_change(const EgoVehicle ego_vehicle,
                                const Prediction prediction,
                                PARA_STATE next_state) {
  int lane = ego_vehicle.current_lane;
  int next_lane = lane;
  Trajectory trajectory(next_state, ego_vehicle.current_lane,
                        ego_vehicle.speed, ego_vehicle.s, ego_vehicle.d);


  SurroundingVehicle front_vehicle;
  SurroundingVehicle behind_vehicle;
  bool front_exist = get_vehicle_front(ego_vehicle, prediction, next_lane, front_vehicle);
  bool behind_exist = get_vehicle_front(ego_vehicle, prediction, next_lane, behind_vehicle);

  if (next_state == STATE_LCL) {
    next_lane = lane - 1;
  } else if (next_state == STATE_LCR) {
    next_lane = lane + 1;
  }

  if (next_lane < LANE_LEFT || next_lane > LANE_RIGHT) {
    // lane does not exist, and so does new trajectory
    trajectory.state = STATE_INVALID;
    return trajectory;
  }
  trajectory.lane = next_lane;
  trajectory.speed = ego_vehicle.ref_speed - ACCELERATION;

  return trajectory;
}

bool get_vehicle_front(const EgoVehicle ego_vehicle,
                       const Prediction prediction,
                       int next_lane,
                       SurroundingVehicle & front_vehicle) {

  vector<SurroundingVehicle> vehicles =  prediction.surrounding_vehicles;
  bool front_exist = false;
  double distance_min = 30.0;
  const double DIST_TH = 30.0;
  for (int i = 0; i < vehicles.size(); i++) {
    if (vehicles[i].d > (2+4*next_lane-2) && vehicles[i].d < (2+4*next_lane+2)) {
      if ((vehicles[i].s > ego_vehicle.end_point_s) &&
          ((vehicles[i].s - ego_vehicle.end_point_s) < DIST_TH)) {
        front_exist = true;
        // nearest vehicle
        if (distance_min > (vehicles[i].s - ego_vehicle.end_point_s)) {
          front_vehicle = vehicles[i];
          distance_min = (vehicles[i].s - ego_vehicle.end_point_s);
        }
      }
    }
  }
  cout << "ego vehicle S:"<< ego_vehicle.s << endl; 
  cout << "front vehicle S:"<< front_vehicle.s << endl;
  cout << "diff S:"<< front_vehicle.s - ego_vehicle.s << endl;
  cout << "ego lane:"<< ego_vehicle.current_lane << endl;
  cout << "front exist:"<< front_exist << endl;
  
  return front_exist;
}

bool get_vehicle_behind(const EgoVehicle ego_vehicle,
                        const Prediction prediction,
                        int next_lane,
                        SurroundingVehicle & behind_vehicle) {

  vector<SurroundingVehicle> vehicles =  prediction.surrounding_vehicles;
  bool behind_exist = false;
  double distance_min = 30.0;
  const double DIST_TH = 30.0;

  for (int i = 0; i < vehicles.size(); i++) {
    if (vehicles[i].d > (2+4*next_lane-2) && vehicles[i].d < (2+4*next_lane+2)) {
      if (vehicles[i].s <= ego_vehicle.end_point_s &&
          (ego_vehicle.end_point_s - vehicles[i].s) > DIST_TH) {
        behind_exist = true;
        // choose behind vehicle
        if ((ego_vehicle.end_point_s - vehicles[i].s) > distance_min) {
          distance_min = (ego_vehicle.end_point_s - vehicles[i].s);
          behind_vehicle = vehicles[i];
        }
      }
    }
  }
  return behind_exist;
}
