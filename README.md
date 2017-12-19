# Model Predictive Control

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

In this project the goal is to implement a model predictive controller (MPC) in C++. The communication between the project and the Udacity simulator is done using WebSocket, where the simulator provides x and y positions, speed, and orientation of the vehicle along with the reference trajectory. The project must respond with steering and throttle data to reliably drive a car around the track simulator.

![]( https://github.com/shmulik-willinger/model_predictive_control/blob/master/readme_img/img1.jpg?raw=true)

Implementation
---
#### The Model
The model is based on the 'kinematic bicycle model' where the state includes the following values:
* positions- x
* position- y
* velocity- v
* orientation- psi
* cross-track error- cte
* orientation error- epsi

and the actuators (control inputs) are:
* steering angle- delta
* acceleration- a

The state update and error equations based on the vehicle model are the main part of the MPC model. The 'Model Predictive Control' loop described below:

![]( https://github.com/shmulik-willinger/model_predictive_control/blob/master/readme_img/MPC_loop.jpg?raw=true)

The blue line is the reference trajectory and the red line is the trajectory computed by the 'model predictive control'

![]( https://github.com/shmulik-willinger/model_predictive_control/blob/master/readme_img/reference_trajectory.jpg?raw=true)


#### Timestep Length and Elapsed Duration

I set the prediction horizon to 1 second with N = 15 and dt = 0.1 (lines 10-11 on MPC.cpp), where:
* N = number of timesteps in the horizon
* dt = how much time elapses between each actuation

When I tried different combinations of the following 2 parameters, I observed that when N is bigger and the car overshot the reference trajectory - it makes the car get off the path very quickly. And when N is too low - the car drove slower. (I played with 5<=N<=25 and 0.05<=dt<=0.2)

#### Polynomial Fitting and MPC Preprocessing

The steps are as follow:
1. Transform the point to the car coordinate system (since the fitting process is in the car coordinates), by subtracting each point from the current position of the vehicle. (lines 105-111 on main.cpp)
2. Transform the orientation to 0 (so the car will be heading straight forward) rotate each point by psi degrees (lines 113-114 on main.cpp)
3. Convert the vector of points to an Eigen vector, in order for it to be an argument in the 'polyfit' function (the points are fitted to a 3rd order polynomial) (lines 115-116 on main.cpp)
4. Evaluate the polynomial to calculate the cross-track error, by the 'polyfit' function. (line 118 on main.cpp)

#### Model Predictive Control with Latency
There is a 100ms delay need to be take of, so I set the Delta time latency value to 0.1s for the solution from the solver method. This helps the car to better 'predict' its location in the future instead of the current position.

I set the max speed of the car to 100 mph, since the car can drive smoothly at this speed, and on higher speed I observed some strong braking at the corners (and the speed is not the main issue at this project)

#### Process results
The vehicle successfully drive a lap around the track. No tire leave the drivable portion of the track surface. The car can go up to 100 mph on the track, while smoothly slowing down in the curves.


Dependencies and Executing
---

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


#### Build Instructions:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
