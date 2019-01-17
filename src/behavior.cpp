#include "behavior.hpp"

Behavior::Behavior (PARA_STATE _state, int _lane, double _speed,
             double _next_s, double _next_d)
    :state(_state),
     lane(_lane),
     speed(_speed),
     next_s(_next_s),
     next_d(_next_d)
  {}
  
Behavior::~Behavior(){}

