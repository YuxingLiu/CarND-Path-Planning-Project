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


vector<Vehicle> Vehicle::choose_next_state(vector<vector<double>> sensor_fusion, int prev_size) {
    /*
    Behavior planning
    */

    // Predict other vehicles using sensor fusion data.
    map<int, vector<Vehicle>> predictions;

    for(int i = 0; i < sensor_fusion.size(); i++) {
        int v_id = sensor_fusion[i][0];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double vi = sqrt(vx*vx + vy*vy);
        double si = sensor_fusion[i][5];
        si += (double)prev_size * dt * vi;      // to compensate measurement latency.
        double di = sensor_fusion[i][6];
        int lanei = di / 4;
        int horizon = 50;       // predict over 1 sec.

        Vehicle veh = Vehicle(lanei, si, vi, 0, "CS");     // Assume constant speed in prediction.
        vector<Vehicle> preds = veh.generate_predictions(horizon);
        predictions[v_id] = preds;
    }

    // Only consider reachable states from current FSM state.
    vector<string> states = successor_state();

    // Calculate total cost of each possible state.
    vector<double> costs;
    vector<vector<Vehicle>> final_trajectories;

    for(int i = 0; i < states.size(); i++) {
        vector<Vehicle> trajectory = generate_trajectory(states[i], predictions);
        if(trajectory.size() != 0) {
            costs.push_back(calculate_cost(*this, predictions, trajectory));
            final_trajectories.push_back(trajectory);
        }
    }

    vector<double>::iterator best_cost = min_element(costs.begin(), costs.end());
    int best_idx = distance(costs.begin(), best_cost);

    return final_trajectories[best_idx];
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
        if(lane != 0) {
            states.push_back("LCL");
        }

        if(lane != lanes_available - 1) {
            states.push_back("LCR");
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
    }

    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    Get next timestep kinematics (position, velocity, acceleration) for a given lane.
    Try to choose max velocity and accel, given other vehicle positions and accel/velocity constrtaints.
    */

    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;

    // Calculate acceleration limits.
    double max_accel = min(this->a + this->max_jerk * dt,  this->max_acceleration);
    double min_accel = max(this->a - this->max_jerk * dt, -this->max_acceleration);

    // Calculate velocity limits.
    double max_vel = min(this->v + max_accel * dt, this->target_speed);
    double min_vel = max(this->v + min_accel * dt, 0.0);

    // Update velocity according to the distance to the ahead vehicle.
    if(get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        double max_vel_in_front;
        double gap = vehicle_ahead.s - this->s - 3 * preferred_buffer;

        if(gap < 0) {
            // max deceleration
            max_vel_in_front = min_vel;
        } else {
            // keep distance to ahead vehicle
            max_vel_in_front = (gap + vehicle_ahead.v * dt + 0.5 * this->v * dt) / (1.5 * dt);
        }

        new_velocity = max(min(max_vel_in_front, max_vel), min_vel);
    } else {
        new_velocity = max_vel;
    }

    new_accel = (new_velocity - this->v) / dt;
    new_position = this->s + new_velocity * dt + 0.5 * new_accel * dt * dt;

    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory
    */

    double next_pos = position_at(dt);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};

    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */

    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));

    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory
    */

    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;

    // Check if a lane change is possible (if the behind vehicle is too close).
    if(get_vehicle_behind(predictions, new_lane, next_lane_vehicle)) {
        // If lane change is not possible, return empty trajectory.
        return trajectory;
    }

    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));

    return trajectory;
}

double Vehicle::position_at(double t) {
    return this->s + this->v * t + 0.5 * this->a * t * t;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Return true if a vehicle is found behind the curretn vehicle, and the distance is too small, false otherwise.
    The passed reference rVehicle is updated if a vehicle is found.
    */

    double max_s = this->s - 30;        // only account for vehicles 30m behind.
    bool found_vehicle = false;
    Vehicle temp_vehicle;

    for(map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if(temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            double gap = this->s - temp_vehicle.s;

            // check if it is too close to the ego vehicle
            if((gap < preferred_buffer && temp_vehicle.v > this->v) || gap < 0.5 * preferred_buffer) {
                rVehicle = temp_vehicle;
                found_vehicle = true;
            }
        }
    }

    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Return true if a vehicle is found ahead of the curretn vehicle within a certain range, false otherwise.
    The passed reference rVehicle is updated if a vehicle is found.
    */

    double min_s = this->s + 60;        // only account for vehicles 60m ahead.
    bool found_vehicle = false;
    Vehicle temp_vehicle;

    for(map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if(temp_vehicle.lane == lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generate predictions for non-ego vehicles.
    */

    vector<Vehicle> predictions;

    for(int i = 0; i < horizon; i++) {
        double next_s = position_at(dt*i);
        predictions.push_back(Vehicle(this->lane, next_s, this->v, 0, "CS"));
    }

    return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Set state and kinematics for ego vehicle using the second state of the trajectory
    */

    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<double> road_data) {
    /*
    Called before simulation begins.
    Set various parameters which will impact the ego vehicle.
    */

    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
    max_jerk = road_data[5];
}

void Vehicle::update(int lane, double s, double v, double a) {
    /*
    Update ego vehicle's states when new measurement is available.
    */

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
}
