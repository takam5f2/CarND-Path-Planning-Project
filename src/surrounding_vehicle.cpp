#include "surrounding_vehicle.hpp"

SurroundingVehicle::SurroundingVehicle(){}
SurroundingVehicle::~SurroundingVehicle(){}

void SurroundingVehicle::set_parameter(int id, double x, double y,
                                       double vx, double vy, double d, double s) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->speed = sqrt(vx*vx + vy*vy);
  this->d = d;
  this->s = s;
}

