/**
 * @file a_star_util.h
 * @brief JPS::AStar Class
 */
#ifndef JPS_ASTAR_UTIL_H
#define JPS_ASTAR_UTIL_H

#include <planner/planner_base.h>
#include <yagsbpl/yagsbpl_base.h>
#include <yagsbpl/A_star.h>

namespace JPS {
  /**
   * @brief 3D AStar planner 
   */
class AStarUtil : public PlannerBase
{
 public:
   ///Node is defined as the 3D integer indices
   typedef Vec3i Node;
   ///Simple constructor, verbose is set to be False by default
   AStarUtil(bool verbose = false);
   ///Planning function, the default epsilon is set to be 1
   bool plan(const Vec3f &start, const Vec3f &goal, decimal_t eps = 1);
 protected:
   int getHashBin(Node &n);
   bool isAccessible(Node &n);
   void getSuccessors(Node &n, std::vector<Node> *s, std::vector<float> *c);

   float getHeuristics(Node &n1, Node &n2);
   bool storePath(Node &n);
   bool stopSearch(Node &n);

   GenericSearchGraphDescriptor<Node, float> myGraph;

   Node _goal_node;
};
}
#endif
