#pragma once

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace grid_planners
{

// D* Lite 从目标反向维护代价，地图小变化时只修补受影响区域
class DLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DLitePlanner();
  ~DLitePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override {}
  void activate()   override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // 字典序 key 让主排序和稳定性排序分开表达
  struct Key {
    float k1{0.0f}, k2{0.0f};
    bool operator<(const Key & o) const {
      return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2);
    }
    bool operator>(const Key & o) const { return o < *this; }
    bool operator<=(const Key & o) const { return !(o < *this); }
  };

  // 版本号让旧队列项自然失效
  struct QEntry {
    Key  key;
    int  idx{-1};
    int  ver{0};
    bool operator>(const QEntry & o) const { return key > o.key; }
  };

  using MinPQ = std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>>;

  float h(int a, int b) const;

  // 目的栅格代价和 A* 保持一致，方便比较统计结果
  float edgeCost(int from, int to, int d, nav2_costmap_2d::Costmap2D * cm) const;

  Key calcKey(int s) const;

  void  pqClean();
  bool  pqEmpty();
  Key   pqTopKey();
  int   pqTopIdx();
  void  pqPop();
  void  pqInsert(int idx, Key k);
  void  pqRemove(int idx);

  void initDLite(int goal_idx, nav2_costmap_2d::Costmap2D * cm);
  void updateVertex(int u, nav2_costmap_2d::Costmap2D * cm);
  void computeShortestPath(nav2_costmap_2d::Costmap2D * cm);

  // 搜索完成后只需沿代价梯度取路径
  nav_msgs::msg::Path extractPath(
    nav2_costmap_2d::Costmap2D * cm,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  std::vector<float> g_;
  std::vector<float> rhs_;
  std::vector<int>   ver_;
  MinPQ pq_;

  float km_{0.0f};
  int s_start_{-1};
  int s_goal_{-1};
  int s_last_{-1};
  int W_{0};
  int H_{0};
  int N_{0};

  // 快照用于把重规划范围收缩到变化栅格附近
  std::vector<unsigned char> prev_cm_;

  bool allow_unknown_{true};
  rclcpp_lifecycle::LifecycleNode::WeakPtr        node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>  costmap_ros_;
  std::string  name_;
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  int expanded_count_{0};

  static constexpr float INF     = std::numeric_limits<float>::infinity();
  static constexpr int   DX[8]   = {-1,  0,  1, -1,  1, -1,  0,  1};
  static constexpr int   DY[8]   = {-1, -1, -1,  0,  0,  1,  1,  1};
  static constexpr float STEP[8] = {1.414f, 1.0f, 1.414f, 1.0f,
                                     1.0f, 1.414f, 1.0f, 1.414f};
};

}
