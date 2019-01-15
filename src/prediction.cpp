#include "prediction.hpp"

Prediction::Prediction(void) {
  surrounding_vehicles.clear();
}

Prediction::~Prediction() {
}

void Prediction::read_sensor_fusion(vector<vector<double>> sensor_fusion) {
  surrounding_vehicles.clear();
  for (int i = 0; i < sensor_fusion.size(); i++) {
    SurroundingVehicle tmp_vehicle;
    tmp_vehicle.set_parameter((int)sensor_fusion[i][0],
                              sensor_fusion[i][1],
                              sensor_fusion[i][2],
                              sensor_fusion[i][3],
                              sensor_fusion[i][4],
                              sensor_fusion[i][6],
                              sensor_fusion[i][5]);
    surrounding_vehicles.push_back(tmp_vehicle);
  }
}


vector<SurroundingVehicle> Prediction::prediction(double timeperiod) {
  for (int i = 0; i < surrounding_vehicles.size(); i++) {
    surrounding_vehicles[i].prediction(timeperiod);
  }
  return surrounding_vehicles;
}

