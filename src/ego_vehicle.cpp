#include "ego_vehicle.hpp"

EgoVehicle::EgoVehicle(){}
EgoVehicle::~EgoVehicle(){}

void EgoVehicle::set_parameter(int lane, double ref_speed, double x, double y, double speed,
                               double yaw, double d, double s,
                               double end_point_s, double end_point_d) {
  this->current_lane = lane;
  this->ref_speed = ref_speed;
  this->x = x;
  this->y = y;
  this->vx = speed * cos(deg2rad(yaw));
  this->vy = speed * sin(deg2rad(yaw));
  this->speed = speed;
  this->yaw = yaw;
  this->d = d;
  this->s = s;
  this->end_point_s = end_point_s;
  this->end_point_d = end_point_d;
}

