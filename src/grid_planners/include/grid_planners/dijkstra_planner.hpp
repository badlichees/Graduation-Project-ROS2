#pragma once

#include "grid_planners/astar_planner.hpp"

namespace grid_planners
{

// Dijkstra = A* with zero heuristic. Guarantees optimal path; explores more nodes than A*.
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

}  // namespace grid_planners
