## Path Planning Project
### Writeup by Tony Knight - 2018/02/22

---


<img src="https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/images/PathPlanningScreenshot.png?raw=true"  width=700>

<i><u>Figure 1: Snapshot from Video of Path Planner Code successfully navigating car though traffic</u></i>

---

### Building code

The code can be built by running
```sh
mkdir build
cmake..
make
```

The model can be run from the build directory by calling:
```sh
./path-plan
```

The program was built starting with [Stanislav Olekhnovich's path planning starter code](https://github.com/fspirit/path-planning-starter).  I used this code to try to create a cleaner separation of code than is present in the default code provided by Udacity.

I integrated the external logging code [spdlog](https://github.com/gabime/spdlog) to write output to the screen and logfiles.  The executable writes out to a logfile `PPlan.log` in the local directory where the program is run.

The [header based spline code](http://kluge.in-chemnitz.de/opensource/spline/) was also used for both trajectory generation , and to implement a better frenet to cartesian mapping.

The program consists of the following code modules

| module | Description |
|--------|-------|
|[spline.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/spline.h) | external code used to build splines. |
|[json.hpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/json.hpp) | external code used to decode json in websocket messages.|
|[main.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/main.cpp) | main module which initiates logging and websocket|
|[WebSocketMessageHandler.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/WebSocketMessageHandler.cpp)  [WebSocketMessageHandler.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/WebSocketMessageHandler.h) | module which decodes websocket json messages, and passes them to the PathPlanner, and returns new path produced by PathPlanner to simulator.|
|[CartesianPoint.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/CartesianPoint.h) | defines structure of cartesian point with (x, y, theta). |
|[FrenetPoint.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/FrenetPoint.h)| defines structures of FrenetPoint with S,D, and FrenetDescription with Frenet references to position, velocity, acceleration and jerk at any point in time.|
|[HighwayMap.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/HighwayMap.cpp) [HighwayMap.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/HighwayMap.h)| Module which maps out the Frenet to Cartesian co-ordinates of the centreline of the highway track.|
|[PathPlannerInput.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathPlannerInput.h) | defines the structure that encompasses all the information passed from the Websocket handler to the Pathplanner.|
|[OtherCar.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/OtherCar.h) | defines the structure of details of other cars reported from the simulator.|
|[PathPlanner.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathPlanner.cpp)  [PathPlanner.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathPlanner.h)| base class for PathPlanners |
|[KeepLanePathPlanner.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/KeepLanePathPlanner.cpp) [KeepLanePathPlanner.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/KeepLanePathPlanner.h) | PathPlanner which produces a fixed velocity straight line path along the central lane without concern for acceleration or other cars.  This code is all present in the module.
|[SimpleSplineBasedPlanner.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/SimpleSplineBasedPlanner.cpp) [SimpleSplineBasedPlanner.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/SimpleSplineBasedPlanner.h) | PathPlanner which produces a path using the spline library in combination with the Jerk minimizing Trajectory (JMT) code, and takes into account other cars, acceleration and velocity.|
|[JMTBasedPlanner.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMTBasedPlanner.cpp) [JMTBasedPlanner.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMTBasedPlanner.h) | PathPlanner that uses JMT code to build trajectories, taking into account velocity, acceleration and other cars.|
|[RoadMap.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.cpp) [RoadMap.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.h) | Module used by JMTBasedPathPlanner to map out Other Cars and project their locations in the future to decide if lane or velocity changes should be made.|
|[Trajectory.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.cpp) [Trajectory.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.h) | Module used by JMTBasedPathPlanner to build and connect JMT trajectories |
|[JMT.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMT.cpp) [JMT.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMT.h) | Module used by Trajectory to calculate JMT Frenet paths |
|[PathTracking.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathTracking.cpp) [PathTracking.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathTracking.h) | Module implementing a deque which is used to store Cartesian and Frenet information regarding the Trajectory.|

### Spline Based Trajectories and Planner

Initially, I worked on developing a method to use the spline to generate trajectories.  This was relatively easy, and worked well to adjoin new trajectories onto the existing path.  Experimentation with displacements in S between starting and ending the move from the center of one lane to the center of the adjoining lane, demonstrated safe limits to prevent excessive acceleration and jerk.

### JMT Based Planner

I then attempted to build a planner which used Jerk minimizing trajectory (JMT) code to generate the path without using the spline code.  This proved to be a lot more tricky and finicky than I had expected.  I eventually realized that I needed to keep track of the JMT Frenet details of each point in the path (location, velocity, and acceleration which I built into a structure called FrenetDescriptors) between steps, so that I could use them to generate new paths.  

``` c++
//from FrenetPoint.h
struct FrenetPoint
{
    double S;
    double D;
.
.
.
}

struct FrenetDescriptors
{
	FrenetPoint Displacement;
	FrenetPoint Velocity;
	FrenetPoint Acceleration;
	FrenetPoint Jerk;
	FrenetDescriptors() = default;
};
```
I developed `PathTracking` code which stored the FrenetDescriptors generated by JMT code in a deque.  A deque was used since we frequently need to trim it at both ends when matching it with the Path provided by the simulator, and inserting a new path.  Cutting elements from the start of a deque is much less costly than with a vector.


## Finite State Machine
I determined that I needed a state machine design with only three states - `Uninitialized`, `DriveInLane`, and `ChangeLane`:  


### 1. `Uninitialized`
---
The role of the `Uninitialized` state is to start the car moving forward in a straight line with no change in direction.  Once the car has moved forward along almost all the path generated in the first iteration of this state (after about 1.2 seconds when there are < 20 pathpoints left), the car is moved into the 'DriveInLane' state.


### 2. `DriveInLane`
---
In the `DriveInLane` state the car moves along the centreline of the lane it is in attempting to achieve the maximum speed of 49 mph (unless there is a slower car ahead, which it will attempt to match velocity with).  The code will evaluate if it should switch to `ChangeLane` to move to a lane with greater horizon before another car.  I added a restriction that the car had to maintain this state for a minimum time period `KEEP_LANE_MINIMUM_TIME` of 2 seconds before it could look to switch to 'ChangeLane`. 


### 3. `ChangeLane`
---
In this state the car travels along a trajectory between the centrelines of two adjacent lanes.  The car is returned to the `DriveinLane` state when it is found to be in the centre of the target lane.  

Currently there are no collision avoidance measures in this state which can lead to issues if a car changes lane into the path of the lane change while it is ongoing.







