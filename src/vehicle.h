#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

    double preferred_buffer = 10;   // impact "keep lane" behavior

    int lane;

    double s;

    double v;

    double a;

    double target_speed;

    int lanes_available;

    double max_acceleration;

    double max_jerk;

    int goal_lane;

    double goal_s;

    string state;

    double dt = 0.02;   // sampling time

    /**
    * Constructor
    */
    Vehicle();
    Vehicle(int lane, double s, double v, double a, string state = "CS");

    /**
    * Destructor
    */
    virtual ~Vehicle();

    vector<Vehicle> choose_next_state(vector<vector<double>> sensor_fusion, int prev_size);

    vector<string> successor_state();

    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    double position_at(double t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    vector<Vehicle> generate_predictions(int horizon=2);

    void realize_next_state(vector<Vehicle> trajectory);

    void configure(vector<double> road_data);

    void update(int lane, double s, double v, double a);

};


#endif
