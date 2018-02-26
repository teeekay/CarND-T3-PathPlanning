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
|[SimpleSplineBasedPlanner.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/SimpleSplineBasedPlanner.cpp) [SimpleSplineBasedPlanner.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/SimpleSplineBasedPlanner.h) | PathPlanner which produces a path using the spline libraryand takes into account other cars, acceleration and velocity.|
|[JMTBasedPlanner.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMTBasedPlanner.cpp) [JMTBasedPlanner.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMTBasedPlanner.h) | PathPlanner that uses Jerk minimizing Trajectory (JMT) code to build trajectories directly, taking into account velocity, acceleration and other cars.|
|[RoadMap.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.cpp) [RoadMap.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.h) | Module used by JMTBasedPathPlanner to map out Other Cars and project their locations in the future to decide if lane or velocity changes should be made.|
|[Trajectory.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.cpp) [Trajectory.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.h) | Module used by JMTBasedPathPlanner to build and connect JMT trajectories |
|[JMT.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMT.cpp) [JMT.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMT.h) | Module used by Trajectory to calculate JMT Frenet paths |
|[PathTracking.cpp](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathTracking.cpp) [PathTracking.h](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathTracking.h) | Module implementing a deque which is used to store Cartesian and Frenet information regarding the Trajectory.|

### Spline Based Trajectories and Planner

Initially, I worked on developing a method to use the spline to generate trajectories in [SimpleSplineBasedPlanner](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/SimpleSplineBasedPlanner.cpp).  This was relatively easy, and worked well to adjoin new trajectories onto the existing path.  Experimentation with displacements in S between starting and ending the move from the center of one lane to the center of the adjoining lane, demonstrated safe limits to prevent excessive acceleration and jerk.  I also improved rudimentary logic to change lanes and prevent rear-ending other cars.

### JMT Based Planner

I then attempted to build a planner which used Jerk minimizing trajectory (JMT) code to generate the path without using the spline code.  This proved to be a lot more tricky and finicky than I had expected.  I eventually realized that I needed to keep track of the JMT Frenet details of vehicle pose at each point in the path (location, velocity, and acceleration) which I built into a structure called [`FrenetDescriptors`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/FrenetPoint.h#L56-L63) between steps, so that I could use them to generate new paths.  

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
I developed [`PathTracking`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathTracking.cpp) code which stored the FrenetDescriptors generated by [`JMT::JMTFrenetDescriptorsAt()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/JMT.cpp#L112-L146) code in a [deque](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/PathTracking.h#L44).  A deque was used since we frequently need to trim it at both ends when matching it with the Path provided by the simulator, and inserting a new path.  Cutting elements from the start of a deque is much less costly than with a vector.

### Finite State Machine in JMTBasedPlanner
I determined that I needed a state machine design with only the following three states: - `Uninitialized`, `DriveInLane`, and `ChangeLane`.

---
#### 1. `Uninitialized`

The role of the `Uninitialized` state is to start the car moving forward in a straight line with no change in direction.  Once the car has moved forward along almost all the path generated in the first iteration of this state (after about 1.2 seconds when there are < 20 pathpoints left), the car is moved into the 'DriveInLane' state.


#### 2. `DriveInLane`

In the `DriveInLane` state the car moves along the centreline of the lane it is in attempting to achieve the maximum speed of 49 mph (unless there is a slower car ahead, which it will attempt to match velocity with).  The code will evaluate if it should switch to `ChangeLane` to move to a lane with greater horizon before another car.  I added a restriction that the car had to maintain this state for a minimum time period `KEEP_LANE_MINIMUM_TIME` of 2 seconds before it could look to switch to 'ChangeLane`. 


#### 3. `ChangeLane`
In this state the car travels along a trajectory between the centrelines of two adjacent lanes.  The car is returned to the `DriveinLane` state when it is found to be in the centre of the target lane.  

\* Currently there are no collision avoidance measures in this state which can lead to issues if a car changes lane into the path of the lane change while it is ongoing.

---

### Trajectory
Custom trajectory code was designed to build trajectories for each state [`Trajectory::InitiateTrajectory()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.cpp#L94-L127), [`Trajectory::GenerateKeepInLaneTrajectory()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.cpp#L134-L253), & [`Trajectory::GenerateJMTLaneChangeTrajectory()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.cpp#L260-L388).  These methods could/should be generalized into a single function, possibly with wrapper functions, to make it more maintainable.  

By ensuring that the conditions at the start of any path generated by the JMT code matched the conditions at the point where it meshed with a previously generated trajectory, and by limiting the acceleration/deceleration rate to 0.225 * g, the code prevented excessive acceleration or jerk.  [`Trajectory::CheckPath()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/Trajectory.cpp#L391-L445) is used to assess the trajectory after it has been mapped to Cartesian co-ordinates, and to detect any acceleration or velocity exceeding the restrictions. 
 Excessive values found with CheckPath() cause the trajectory to be recalculated in JMT with a modified acceleration rate.  Testing has demonstrated that this effectively keeps the car within the required velocity, acceleration, and jerk restrictions.

### Prediction - RoadMap

The [`RoadMap::CreateRoadMap()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.cpp#L30-L133) code was used to build a [RoadMapDeck](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.h#L31-L35) which consisting of one layer containing the position of the EgoCar as time progressed, and one layer that contained the position of all the other cars as time progressed.  The relative car positions were mapped through time at 0.1 s intervals for the next 2 seconds, with each time interval mapping as a separate dimension on the respective EgoCar and OtherCar layers.  In this way the relative clearances in each lane between the EgoCar and any other car at any time interval could be easily calculated.  Furthermore the minimum clearances over the next 2 seconds could be used both to evaluate if lane changes were possible, and if they might be advantageous. At present the algorithm only considers motion along the lane the car is currently in and does not predict lane changes of other cars that could be detected based on their velocity.

The [`RoadMap::CheckForSlowCarsAhead()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.cpp#L293-L317) and [`RoadMap::CheckForSlowCarInOtherLane()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.cpp#L319-L330) methods also look for cars ahead of the EgoCar within a certain distance threshold, to determine if the EgoCar should be decelerating, and by how much to avoid a rear end collision.

I thought that the RoadMapDeck structure could also be used to set up a grid for an A* style pathfinding algorithm, but did not have time to follow this up.

### Behaviour - RoadMap

LaneChange options are evaluated in [`RoadMap::CheckForLaneChange()`](https://github.com/teeekay/CarND-T3-PathPlanning/blob/master/src/RoadMap.cpp#L180-L290). The following decision tree was used to determine if a lanechange should be made:

```
If the current lane is clear for at least 60m over the next 2 seconds:
 1. Stay in lane if you are in the center lane.
 2. If in an outside lane, move to center lane if lanechange clearance is met and middle lane is also clear ahead for at least 60m for next 2 seconds.

If The current lane has a minimum clearance over next 2 seconds of less than 60m:
 1. Move from outside lanes to middle lane if lanechange clearance is met and:
    1. Middle lane has greater length of clear lane than current lane over next 2 seconds
    2. Middle lane has at least 40 m of clearance over next 2 seconds
    3. Current outside lane has less than 40 m clearance, but opposite outside lane has more than 55 m clearance over next 2 seconds (middle lane is just a transit to other outside lane).
  2. Move from the centre lane to the outside lane which has the greater min clearance exceeding that of the centre lane over the next 2 seconds.
```

The minimum clearances assessed to be relatively safe to permit lane changes were 6m behind the car location point and 16 m in front of the car location point over the next 2 seconds in the target lane.  These are more aggressive than would be recommended in a driving school, but were effective to move through tight traffic when possible in the simulator.

The car was heavily biased to move to the center lane, as it appeared that this would allow greater degree of choice, and reduce the possibility of getting caught without lane change possibilities that would occur more often in outside lanes.

## Improvements

Several possible improvements have been identified above.  Additional possible improvements include:
 1. Adding simulation of various alternative trajectories to avoid a predicted collision with a car ahead when no lane change options are available from the decision tree above, and default deceleration will not prevent the collision.
  2. Adding simulation of reducing EgoCar velocity when stuck in traffic in an outside lane, while other Outside lane is clear of traffic to evaluate if a lanechange would become possible.


 








