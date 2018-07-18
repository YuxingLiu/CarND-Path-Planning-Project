# Path Planning Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to design a path planner in C++ that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic in a [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). The path planner is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

The path planner consists of two modules: behavior planning and trajectory generation. When no vehicle is found ahead of the ego vehicle, the reference speed gradually increases to max value and lane is unchanged. When it is too close to the ahead vehicle, a finite state machine (FSM) is activated to update reference speed and lane, according to a set of cost functions and safety constraints. Given the next step reference speed and lane, spline fitting is used to generate a feasible trajectory. A detailed description of the code used in each module is in the following sections.

---

## Behavior Planning

Using localization and sensor fusion data, detect if the ego vehicle is too close to the ahead vehicle:
```cpp
for(int i=0; i < sensor_fusion.size(); i++)
{
    // car in my lane
    double d = sensor_fusion[i][6];
    //if(d < (2+4*lane+2) && d > (2+4*lane-2))
    if(d < car_d + 2  && d > car_d - 2)
    {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

        // project s value forward in time (to compensate measurement latency)
        check_car_s += ((double)prev_size * .02 * check_speed);
        // check s values greater than mine and s gap
        if(check_car_s > car_s && check_car_s < car_s + 45)
        {
            too_close = true;
        }
    }
}
```
Note that in the first if statement, `car_d` is used to calculate the lateral distance, instead of `lane`. This can also cover the case when vehicle is chaning lanes. Due to a measurement latency of about 1 sec, a larger 45 m gap is used.

When `too_close` is true and `lane_change` is false, activate the FMS to prepare for lane change:
```cpp
if(too_close && lane_change == false)
{
    ego.update(lane, car_s, car_v, car_a);
    vector<Vehicle> trajectory = ego.choose_next_state(sensor_fusion, prev_size);
    ego.realize_next_state(trajectory);
    ref_vel = ego.v * 2.24;
    if(ego.lane != lane) {
        lane = ego.lane;
        lane_change = true;
    }
}
```
If `lane_change ` is true, adjust the speed if necessary and follow a smooth lane changing path. When the ego vehicle is in the new lane, reset FSM and `lane_change` flag:
```cpp
else if(lane_change) {
    if(car_d > 4*lane && car_d < 4+4*lane) {
        lane_change = false;
        ego.state = "KL";
    }

    if(too_close) {
        ref_vel -= .224;
    }
}
```
If no vehicle ahead, stay at current lane and accelerate up to max speed:
```cpp
else
{
    if(ref_vel < 49.5) {
        ref_vel += .224;
    }
}
```

To implement the FSM, a class `Vehicle` is used for ego and non-ego vehicles, whose members and methods are defined in [Vehicle.h](https://github.com/YuxingLiu/CarND-Path-Planning-Project/blob/master/src/vehicle.h). The `Vehicle` class is similar to the one defined in [Behavior Planning Quiz](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/56274ea4-277d-4d1e-bd95-ce5afbad64fd/concepts/2c4a8fdd-4072-425f-b5ae-95849d5fd4d8), where the differences are highlighted as follows:
1. The FSM only has three states (`KL`, `LCL`, `LCR`). If lane change is infeasible, FSM will stay at `KL` and re- evaluate at next sample time. If lane change is feasible, FSM will immediately switch to the goal lane, and the smooth transition is ensured by the trajectory generator.
2. The acceleration `a` is updated as a state variable, with consideration of acceleration and jerk limits.
3. Since the FSM is not always activated, the ego vehicle's `lane`, `s`, `v`, and `a` are reinitialized every sample time by `update()` function.
4. The positions `s` of ego and non-ego vehicles are compensted for measurement latency at every sample time.
5. The safety constraints (whether lane change is feasible) are implemented differently, due to the removal of `PLCL` and `PLCR` states.
6. The distance to ahead vehicle is accounted for in cost functions.

More details can be found in the following subsections.

### Transition Function

The transition function `choose_next_state()` is defined in [Vehicle.cpp](https://github.com/YuxingLiu/CarND-Path-Planning-Project/blob/master/src/vehicle.cpp#L29) starting at line 29.

First, other vehicles are predicted over 1 sec using sensor fusion data, with the assumption of constant speed:
```cpp
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
```

Note that the positions of non-ego vehicles `si` are compensted for measurement latency. 

Then, rough trajectory for each reachable state is generated, and the cost of each state is calculated:
```cpp
vector<string> states = successor_state();

vector<double> costs;
vector<vector<Vehicle>> final_trajectories;

for(int i = 0; i < states.size(); i++) {
    vector<Vehicle> trajectory = generate_trajectory(states[i], predictions);
    if(trajectory.size() != 0) {
        costs.push_back(calculate_cost(*this, predictions, trajectory));
        final_trajectories.push_back(trajectory);
    }
}
```

Finally, the state with minimum cost is chosen as the next state of FSM:
```cpp
    vector<double>::iterator best_cost = min_element(costs.begin(), costs.end());
    int best_idx = distance(costs.begin(), best_cost);

    return final_trajectories[best_idx];
```

### Cost Functions



```cpp

```

## Trajectory Generation
