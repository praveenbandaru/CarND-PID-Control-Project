# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Introduction
In this project, the goal is to implement a PID controller in C++ to maneuver the vehicle around the track!
The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle by the PID controller. This project uses the [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation to communicate with the [simulator](https://github.com/udacity/self-driving-car-sim/releases). This project is based on the [starter code](https://github.com/udacity/CarND-PID-Control-Project) provided by Udacity.

# [Rubric points](https://review.udacity.com/#!/rubrics/824/view)  
## Compilation

### Your code should compile.

The code compiles without any errors.

## Implementation

### The PID procedure follows what was taught in the lessons.

The PID controller implementation was done in the file `PID.cpp`. The coefficients Kp, Ki & Kd were initialized using the method `PID::Init(double Kp, double Ki, double Kd)`. The P, I & D errors were calculated using the method `PID::UpdateError(double cte)`. The total error was calculated using the method `PID::TotalError()`.


## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

-   The proportional component **( P )** represents our actual error **( CTE )**, the value between where it is now and where it should be. This component in our controller helps to steer the car into the center of the lane.
    
-   The integral component **( I )** represents the running sum of previous errors, used for making fine movements when the error **( CTE )** is small. This component in our controller helps to eliminate a possible bias present in the system. It helps to reduce the error around sharp curves. 
    
-   The differential component **( D )** represents the change in errors, used to predict what the next error might be. This component in our controller helps to eliminate the overshoot of center of the line by smoothing the approach to it.
    

### Describe how the final hyperparameters were chosen.

The parameters were chosen by using a manual tuning method. The way I tuned my parameters is as follows:

 - Set Kp, Ki and Kd to 0.
 - Increase Kp until the error is fairly small, but still the car is able to drive around the track (atleast part of it) successfully.
 - Increase Kd until any overshoot you may have is fairly minimal. But be careful with Kd - too much will make it overshoot.
 - Increase Ki until any error that is still existing is eliminated. Start with a really small number for Ki, don't be surprised if it is as small as 0.0001 or even smaller.
 - Fine tune the above parameters (a little bit) to get the controller working to the best performance.

The final parameters were [P: 0.16, I: 0.0006, D: 3.6].

## Simulation

### The vehicle must successfully drive a lap around the track.

A short video with the final parameters is [here](https://www.youtube.com/watch?v=UB7Y2DgnFhE).

[![PID Controller](https://img.youtube.com/vi/UB7Y2DgnFhE/0.jpg)](https://www.youtube.com/watch?v=UB7Y2DgnFhE)


## Dependencies

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
