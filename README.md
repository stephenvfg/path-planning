# Path Planning for Highway Driving

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# Description

This repository contains the files for a project from the [Udacity Self Driving Car Nanodegree Program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013). The goal of this project was to use sensor data, cost functions, and a finite state machine to plan the behavior and trajectory of a vehicle along a highway.

<img src="https://github.com/stephenvfg/path-planning/blob/master/vis.gif" width="500px">

The project code was compiled and used in tandem with a simulator from Udacity that placed the vehicle on a highway alongside other traffic. The simulator kept track of distance, time on the road, speed, acceleration, and jerk to make sure that the vehicle behavior conformed to speed, comfort, and other physical limitations. 

**Project code:**

* [Behavior and path planning pipeline (`main.cpp`)](https://github.com/stephenvfg/path-planning/blob/master/src/main.cpp)
* [Helper functions (cost, states, conversions) (`helpers.h`)](https://github.com/stephenvfg/path-planning/blob/master/src/helpers.h)

# Path Generation

The path planning pipeline reads sensor and localization data, processes it to determine the next course of action, and then plots next steps along a smooth path for the car to follow.

1. Process localization and sensor fusion data to under stand the position and velocity of our car and the cars around us. (`main.cpp` lines 106 to 144)
2. Based on the current state of the car, identify a list of all possible future states of the car (`keep in the same lane` or `plan to change lanes to the left`, for example). (`helpers.h` lines 157 to 181)
3. Calculate the cost of each possible next state. The cost of the state is based on the following:
    - The speed of the target lane. This cost function penalizes lanes that are moving at a slower pace.
    - The collision risk of the target lane. This cost function penalizes lanes when there is another vehicle in close proximity to the position of our vehicle.
    - Whether or not we are changing lanes. This cost function penalizes states that require us to move into another lane.
The cost function calculates the individual costs of the above elements and applies weights to determine the total cost of a state. The cost function heavily weights the collision risk cost over the others. (`helpers.h` lines 183 to 273)
4. The pipeline iterates through each possible state and then selects the state with the lowest cost to be the behavior of our vehicle in the next iteration. (`main.cpp` lines 144 to 181)
5. Our vehicle may modify its behavior based on the next state: (`main.cpp` lines 181 to 208)
    - If we are keeping in the same lane, the only thing our vehicle does is check to make sure we are not at risk of colliding into a car in front of us. If so we slow down our vehicle.
    - If we are planning to change lanes, we slow down our vehicle slightly as preparation.
    - If we are making a lane change, then we set the new lane.
6. Once the behavior is defined, then we plot the points on our path to execute the behavior. This consists of: (`main.cpp` lines 208 to 318)
    - Definining two initial path points at the position of the vehicle.
    - Placing addition path points 30, 60, and 90 meters ahead along the road using frenet coordinates and the intended lane.
    - Creating a spline (curve) along those points which we will use to plot our actual path. The spline helps stay along a smooth path and prevent excessive jerk.
    - Plot a set number of points along the spline according to the speed we are aiming for the how many points ahead we want to plot.
7. This pipeline is repeated indefinitely to keep the car moving along the highway while showing preference for higher-speed lanes while avoiding risky movements. At each iteration, the pipeline appends new path points to replace however many path points were consumed in the previous iteration.