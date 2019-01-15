#include "state_machine.hpp"

StateMachine::StateMachine(void)
  :state(STATE_LK)
{
}

StateMachine::~StateMachine(void){}


vector<PARA_STATE> StateMachine::get_next_state_candidate(void) {
	vector<PARA_STATE> next_state;
	if (state == STATE_LK) {
		next_state.push_back(STATE_LK);
		next_state.push_back(STATE_LCL);
		next_state.push_back(STATE_LCR);
	} else if (state == STATE_PLCL) {
		next_state.push_back(STATE_LK);
		// next_state.push_back(STATE_PLCL);
		// next_state.push_back(STATE_LCL);
	} else if (state == STATE_PLCR) {
		next_state.push_back(STATE_LK);
		// next_state.push_back(STATE_PLCR);
		// next_state.push_back(STATE_LCR);
	} else if (state == STATE_LCL) {
		next_state.push_back(STATE_LK);
		next_state.push_back(STATE_LCL);
	} else if (state == STATE_LCR) {
		next_state.push_back(STATE_LK);
		next_state.push_back(STATE_LCR);
	}

	return next_state;
}

