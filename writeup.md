# Path Planning Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to design a path planner in C++ that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic in a [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). The path planner is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

The path planner consists of two modules: behavior planning and trajectory generation. When no vehicle is found ahead of the ego vehicle, the reference speed gradually increases to 50 mph and lane is unchanged. When it is too close to the ahead vehicle, a finite state machine is activated to update reference speed and lane, according to a set of cost functions and safety constraints. Given the next step reference speed and lane, spline fitting is used to generate a feasible trajectory. A detailed description of the code used in each module is in the following sections.

---

## Behavior Planning

Using localization and sensor fusion data, detect if the ego vehicle is too clsoe to the ahead vehicle:
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
Note that in the first if statement, `car_d` is used to calculate the lateral distance, instead of `lane`. 

When no vehicle is found ahead of the ego vehicle, the reference speed gradually increases to 50 mph and lane is unchanged.
```cpp

```
