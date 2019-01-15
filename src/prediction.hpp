#ifndef __PREDICTION_HPP__
#define __PREDICTION_HPP__

#include <vector>
#include <iostream>
#include "surrounding_vehicle.hpp"

using namespace std;

class Prediction {
public:
  vector<SurroundingVehicle> surrounding_vehicles;
  Prediction(void);
  virtual ~Prediction();
  void read_sensor_fusion(vector<vector<double>> sensor_fusion);
  vector<SurroundingVehicle> prediction(double timeperiod);
  vector<SurroundingVehicle> get_prediction(void) { return surrounding_vehicles; }
};

#endif
