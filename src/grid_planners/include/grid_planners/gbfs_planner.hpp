#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// 只看启发式，适合做最快响应的对照组，不保证路径最优
class GBFSPlanner : public AStarPlanner
{
public:
  GBFSPlanner() : AStarPlanner("GBFSPlanner") {}
  ~GBFSPlanner() override = default;

protected:
  float priority(float /*g*/, float h) const override { return h; }
};

}
