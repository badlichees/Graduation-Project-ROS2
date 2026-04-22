#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// Greedy Best-First Search: f = h only, ignores path cost.
// Fastest to reach goal but does NOT guarantee an optimal path.
class GBFSPlanner : public AStarPlanner
{
public:
  GBFSPlanner() : AStarPlanner("GBFSPlanner") {}
  ~GBFSPlanner() override = default;

protected:
  float priority(float /*g*/, float h) const override { return h; }
};

}  // namespace grid_planners
