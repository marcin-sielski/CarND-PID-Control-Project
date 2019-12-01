# CarND-Controls-PID

This project implments simple PID Controller.

The first evelolution of PID Controller was invented in 1911 by Elmer Sperry. PID Controllers are widely used in control systems in many industries.
For example industrial control applications uses PID controllers to regulate temperature, flow, pressure speed. In this project PID Controller is
used to actuate steering wheel angle for the vehicle moving around the virtual racetrack within the simulator.

---

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Project Description

PID Controller automatically applies the correction based on proportional, integral and derivative terms (denoted P, I and D respectively).
Each term has hyperparameter (coefficient) which value affects driving path.

* The P (proportional) term drives steering wheel angle of the ego vehicle to keep center of the track as close as possible.
  If Kp is the only coefficent not equal to zero the driving path resembles a sine wave. Increasing Kp causes ego vehicle to drive outside of the track on the
  stright part. Decreasing Kp couses ego vehicle to drive outside of the track on the turn.
  
* The D (derivative) term predicts the future behaviour. The term provides stabilization to the driving behaviour and smoothes ego vehicle movement canelling
  left and right oscillation.

* The I (integral) term corrects bias or the systematic error. This project does not use integral term becasue it is assumed that controlling steering wheel
  angle is perfect and does not need correction. Ki is set to zero.
  
## Results

The result video can be found at following [link](./pid.mp4)

## Hyperparameters Selection and Tuning

The hyperparmaters values were tuned manually to following values [Kp, Kd, Ki] = [0.2, 0.00001, 0].
It turned out that throttle value has great inpact on hyperparameters selection. Following method was applied to select hyperparameters:
1. Set Kp to some value and Kd, Ki to zero,
2. Manipulate Kp up and down to see the effect,
3. While ego vehicle accelerates notice that throttle has great effect on Kp,
4. Decrease throttle to 0.1,
5. Stabilize the ego vehicle path with small Kd value,
6. Ki was set to zero with the assumption there is no systematic error in the simulated environment.
  