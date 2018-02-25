# Path Planning Project 

## For Udacity Self-Driving Car Engineer Nanodegree Program
by Anthony M. Knight

## Original Framework
This Project was built on top of the Path Planning framework provided by [Stanislav Olekhnovich in his Github Path Planning Starter](https://github.com/fspirit/path-planning-starter).  I used this in my project in order to try to learn how to separate and use object oriented C++ coding rather than the default starter code [provided in Udacity's Github](https://github.com/udacity/CarND-Path-Planning-Project), which tends to be long blocks of procedural code mixed in main() with the WebSocket handling code.

## PathPlanners
3 path planners are included built on the PathPlanner Interface:
1. KeepLanePathPlanner, that just goes straight with fixed speed, keeping lane (Generally unchanged from the starter code)
2. SimpleSplineBasedPlanner, that uses a spline to create paths, and uses the first version of prediction code to determine othercar locations in the future, and if lanechanges should or can be made.
3. JMTBasedPlanner, that solely uses jerk minimizing code to generate the vehicle trajectory (no spline) and uses a more sophisticated future prediction code to determine othercar locations in the future, and if lanechanges should or can be made. More details for the Udacity assignment are provided in [Model Documentation](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/ModelDocumentation.md)

## Cartesian - Frenet Co-ordinate Conversion
The default code to convert between Frenet and Cartesian Co-ordinates provided in the Udacity project is generally insufficient to produce good results in this project.  The simulator also produces bad results when locating points beyond the next waypoint.  The code to convert co-ordinates was augmented using the spline library to produce smoothed X, Y, dX, dY co-ordinates at 1 m intervals of S around the track.  These more granular values were used in higher resolution algorithms to generate better results. 

## Logging
I integrated [SPDLog](https://github.com/gabime/spdlog), a "Very fast, header only, C++ logging library" into the project to add fast controllable logging from each section of code to logfiles and console.

## To Do
Although this code works quite well, it does not compare the result of various alternative trajectories to evaluate which is best, but uses a procedural block to select a specific trajectory which is used.  In certain situations, (like another car suddenly braking in front of the Ego Car faster than default deceleration allows), this might allow the Car to determine that it can squeeze into another lane which it would normally not do based on default clearance requirements for lane changes, or apply stronger deceleration than is normally permitted, in order to avoid a collision.

The prediction code does not take into account frenet "D" motion of othercars in order to predict that a car will be changing lanes in the future.

## Using Windows Visual Studio 2017 CMAke folder open with local Ubuntu Bash on Windows
I worked on [CMakeSettings.json](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/CMakeSettings.json) and [CppProperties.json](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/CppProperties.json) to enable Intellisense to work fairly well in Visual Studio 2017, and to be able to use Visual Studio with gdb for debugging with the Linux target.  These files need to be adapted to match your local paths, but should help point you in the right direction.

-----

# From Udacity [README.md](https://github.com/udacity/CarND-Path-Planning-Project/blob/master/README.md)

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

### Dependencies

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