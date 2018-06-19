#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <cmath>


// Set weights for cost functions.
const double REACH_GOAL = pow(10, 0);
const double EFFICIENCY = pow(10, 2);
const double DIST_AHEAD = pow(10, 2);


double goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases based on distance of final lane of trajectory.
    Cost of being out of goal lane becomes larger as vehicle approaches goal distance.
    */

    double cost;
    double distance = data["distance_to_goal"];

    if(distance > 0) {
        cost = 1 - exp(-abs(vehicle.goal_lane - data["final_lane"]) / distance);
    } else {
        cost = 1;
    }

    return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with small final speed.
    */

    double cost = (vehicle.target_speed - data["final_speed"]) / vehicle.target_speed;
    return cost;
}

double ahead_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with small distance to the front vehicle.
    */

    double gap = get_ahead_distance(vehicle, predictions, data["final_lane"]);
    double cost = 1 - gap / 60;

    return cost;
}

double get_ahead_distance(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Calculate the distance to a vehicle ahead of the ego vehicle in a lane (within certain range).
    */

    double min_s = vehicle.s + 60;
    Vehicle vehicle_ahead;
    Vehicle temp_vehicle;

    for(map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if(temp_vehicle.lane == lane && temp_vehicle.s > vehicle.s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            vehicle_ahead = temp_vehicle;
        }
    }

    return min_s - vehicle.s;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */

    map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    vector< function<double(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = {goal_distance_cost, inefficiency_cost, ahead_distance_cost};
    vector<double> weight_list = {REACH_GOAL, EFFICIENCY, DIST_AHEAD};

    for(int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;
}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost function.
    */

    map<string, double> trajectory_data;
    Vehicle trajectory_last = trajectory[1];

    double distance_to_goal = vehicle.goal_s - trajectory_last.s;
    double final_lane = trajectory_last.lane;
    double final_speed = trajectory_last.v;

    trajectory_data["distance_to_goal"] = distance_to_goal;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["final_speed"] = final_speed;

    return trajectory_data;
}
