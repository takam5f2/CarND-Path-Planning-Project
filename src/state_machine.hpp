#ifndef __STATE_MACHINE_HPP__
#define __STATE_MACHINE_HPP__

#include <vector>
#include <map>
#include <iostream>

using namespace std;

// state
enum PARA_STATE {
  STATE_LK, // lane keeping
  STATE_PLCL, // preparing lane change to left
  STATE_PLCR, // preparing lane change to right
  STATE_LCL, // lane change to left lane
  STATE_LCR, // lane change to right lane
  STATE_INVALID // Invalid state.
};


class StateMachine {
private :
	PARA_STATE state;
public:
	StateMachine(void);
	virtual ~StateMachine(void);
  void set_state(PARA_STATE state) { this->state = state; };
	PARA_STATE get_state(void) { return this->state; };
	vector<PARA_STATE> get_next_state_candidate(void);

};

#endif
