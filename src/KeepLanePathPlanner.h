//
// Created by Stanislav Olekhnovich on 12/10/2017.
// see https://github.com/fspirit/path-planning-starter
//

#ifndef PATH_PLANNING_KEEPLANEPATHPLANNER_H
#define PATH_PLANNING_KEEPLANEPATHPLANNER_H

#include "PathPlanner.h"

class KeepLanePathPlanner : public PathPlanner
{
public:
    explicit KeepLanePathPlanner(const HighwayMap &map, int startingLane): PathPlanner(map, startingLane) {};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
};


#endif //PATH_PLANNING_KEEPLANEPATHPLANNER_H
