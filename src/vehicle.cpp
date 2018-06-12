#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "cost.h"

/**
 * Initialize Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;

}

Vehicle::~Vehicle(){}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<vehicle>> predictions) {

    //TODO: behavior planning

    return generate_trajectory("CS", predictions);
}

vector<string> Vehicle::successor_state() {
    /*
    Provide the possible next states given by the current state for the FSM.
    Lane changes happen instantaneously, so LCL and LCR can only transition back to KL.
    */

    vector<string> states;
    states.push_back("KL");
    string state = this->state;

    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if(state.compare("PLCL" == 0) {
        if(lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if(state.compare("PLCR" == 0) {
        if(lane != 0) {
            states.push_back("PLCR");
            states.push_back("PLCR");
        }
    }

    // If state is "LCL" or "LCR", just return "KL" 
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible state, generate the appropriate trajectory to realize the next state.
    */

    vector<Vehicle> trajectory;

    if(state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if(state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if(state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if(state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }

    return trajectory;
}
