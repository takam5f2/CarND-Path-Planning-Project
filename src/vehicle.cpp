#include "vehicle.hpp"

Vehicle::Vehicle(){}
Vehicle::~Vehicle(){}


void Vehicle::set_parameter(int id, int lane, int next_lane,
                             double x, double y, double vx, double vy,
                             double speed, double yaw,
                             double d, double s,
                             double end_point_s, double end_point_d) {
  this->id = id;
  this->current_lane = lane;
  this->next_lane = next_lane;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;

  this->speed = speed;
  this->yaw = yaw;
  this->d = d;
  this->s = s;
  this->end_point_s = end_point_s;
  this->end_point_d = end_point_d;
}

void Vehicle::prediction(double time_period) {
  this->x = this->x + this->vx * time_period;
  this->y = this->y + this->vy * time_period;
  this->s = this->s + this->speed * time_period;
}
