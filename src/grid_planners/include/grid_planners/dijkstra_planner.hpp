#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// 零启发式用于提供最稳的最优性基线，代价是展开更多节点
class DijkstraPlanner : public AStarPlanner
{
public:
  DijkstraPlanner() : AStarPlanner("DijkstraPlanner") {}
  ~DijkstraPlanner() override = default;

protected:
  float heuristic(int /*idx*/, int /*gx*/, int /*gy*/, int /*W*/) const override
  {
    return 0.0f;
  }
};

}
