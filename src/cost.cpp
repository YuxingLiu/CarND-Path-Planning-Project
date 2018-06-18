#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <cmath>


// Set weights for cost functions.
const double REACH_GOAL = pow(10, 0);
const double EFFICIENCY = pow(10, 2);


double goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases based on distance of intended lane and final lane of trajectory.
    Cost of being out of goal lane becomes larger as vehicle approaches goal distance.
    */

    double cost;
    double distance = data["distance_to_goal"];

    if(distance > 0) {
        cost = 1 - 2*exp(-abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance);
    } else {
        cost = 1;
    }

    return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target speed.
    */

    double proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
    if(proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    double proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
    if(proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }

    double cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed;
    return cost;
}

double lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    If a vehicle is found ahead of the ego vehicle in a lane (within certain range), lane speed is the traffic speed.
    */

    double min_s = vehicle.s + 60;
    bool found_vehicle = false;
    Vehicle vehicle_ahead;
    Vehicle temp_vehicle;

    for(map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if(temp_vehicle.lane == lane && temp_vehicle.s > vehicle.s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            vehicle_ahead = temp_vehicle;
            found_vehicle = true;
        }
    }

    if(found_vehicle) {
        return vehicle_ahead.v;
    }

    // Fount no vehicle in the lane.
    return vehicle.target_speed;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */

    map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    vector< function<double(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
    vector<double> weight_list = {REACH_GOAL, EFFICIENCY};

    for(int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;
}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost function:
        intended_lane: +/- 1 from current lane if the vehicle is planning or executing a lane change.
        final_lane: The lane of the vehicle at the end of the trajectory. Unchanged for KL and PLCL/PLCR.
        distance_to_goal: The s distance of the vehicle to the goal.
    */

    map<string, double> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    double intended_lane;

    if(trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane - 1;
    } else if(trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane + 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    double distance_to_goal = vehicle.goal_s - trajectory_last.s;
    double final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;

    return trajectory_data;
}
