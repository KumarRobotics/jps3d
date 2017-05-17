#include <planner/planner_base.h>
#include <yagsbpl/yagsbpl_base.h>
#include <yagsbpl/A_star.h>

namespace JPS {
class AStarUtil : public PlannerBase
{
 public:
  AStarUtil(bool verbose = false);
  bool plan(const Vec3f &start, const Vec3f &goal, decimal_t eps = 1);
 protected:
  typedef Vec3i Node;
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
