# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Compilation

1. Install all dependencies (below)
2. Clone this repo.
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./mpc`.

## Simulation

![alt](https://img.youtube.com/vi/Gq_VPqJBU6c/hqdefault.jpg)

The [YouTube video seen here](https://www.youtube.com/watch?v=Gq_VPqJBU6c) shows our 
car driving around the track with a speed limited to 50mph.  The yellow lines
indicate the "waypoints" from the simulator showing a preferred path forward.
The green lines and dots show an optimal path predicted by our model
controller.

## The Model

We use a kinematic model for car's state with six variables:

* _x, y_ position
* velocity _v_
* heading _psi_
* turn rate _epsi_
* cross-track error _cte_
	
Cross track error _cte_ refers to the distance we're to the left or right of the
center line. Our model ignores dynamics such as momentum, road conditions, tire patch dynamics
on the road surface, and wind resistance.  These very well might be part of a
model in production. We also model two actuators to drive the car around the track:

* Steering adjustment (-1 to 1), _delta_, varying +/- 25 degrees
* Throttle adjustment (-1 to 1), _a_, varying from full power reverse to full power forward
	
At every time step we

* Obtain "waypoints" from the simulator showing our preferred path forward
* Fit a "guidewire" quadratic curve to these points
* Create a nonlinear optimization problem with constraints as follows:
* We create N sets of simultaneous equations, one for each timestep dt
* For adjacent timesteps t and t+1, we constrain values using Newtonian mechanics:

```
x(t+1) = x(t) + v*cos(psi)*dt
y(t+1) = y(t) + v*sin(psi)*d5
v(t+1) = v(t) + a*dt
cte(t+1) = cte + v*sin(epsi)*dt
epsi(t+1) = epsi + (1/Lf)*v*delta*dt
```

* We specify a "cost" to choosing values for a and delta.
* We run an optimizer to choose a sequence of throttle _a_ and steering changes _delta_
over all N steps that minimizes total cost.
* Given an optimal plan forward, we return the initial throttle _a_ and steering _delta_
as input to our simulated actuators.

The cost function is a weighted sum of several factors.  The weights were initially
uniform (1).  Through experimentation we observed optimal paths had numerous
twists and turns which caused instability.  We penalized changes in
steering direction by a factor of 1e6, and values of steering by a factor of 100.  This 
was sufficient to navigate the track at a speed limit of 50mph.

The cost function includes the following factors:

* The squared difference between our current speed, steering, and cross track error
against a desired speed of 50mph, steering of 0, and track error of 0.
* Our chosen steering and throttle values for each time step, squared
* Our changes between adjacent time steps for steering and throttle, squared 


## Timestep length & duration

We chose to model one second forward in time, 100 (_N_) simulated steps of 10 (_dt_) milliseconds each.  Fewer
steps were insufficient to model behavior at tight corners, which led to instability.  Larger values
of N provided little if any improvement to the prediction of the next throttle _a_ and steering angle _delta_. 

The timestep _dt_ was set intuitively as 1/10th of the latency of 100ms.  We found that even smaller timesteps
may more accurately predict a path forward, but that the end result was marginally better while introducing
additional computational cost.  Larger timesteps began to degrade performance of the vehicle, largely through
excess correction of steering and more braking and accelerating.


## Polynomial Fitting & Preprocessing

We accepted waypoints as-is.  We did notice waypoints degraded over time, particularly at
higher speeds and around twisted sections of road.  A better approach could mitigate these errors by
averaging waypoints between subsequent timesteps, perhaps using a sliding average to stabilize
changes.

We fit a 2-degree polynomial to the waypoints, which we used as an optimal path forward
for our vehicle.  Higher degrees were at times more accurate.  However,
we witnessed overfitting at higher speeds that caused unnatural behaviors such as rapid changes between
accelerating and braking, or constant adjustments of the steering wheel.  The higher dimensional 
paths also seemed erratic with oscillation.

We guided our model to use a strive for a velocity of 50mph, with no cross track error
and a straight heading.  We constrained our turning radius to +/- 25 degrees, then normalized
these values to the interval [-1, 1] before sending to the simulator. 
We found that we had to invert the predicted steering angle, as apparently
this value was subtracted vs. added to the current heading in the simulator.  

## Model with Latency

We account for latency by assuming the current car drifts at the current speed, heading,
and rate of turn for the entire interval forward.  These become the initial state for our
model.  Our algorithm then selects an optimal sequence of steering and throttle adjustments,
100 times a second, for that time forward.  This is equivalent to looking ahead while you're
driving, realizing you can't do that much about what's immediately in front of you at
highway speeds.  Your decisions now affect your location, heading and speed
a few feet in front of you, not where you are at the current instant.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
