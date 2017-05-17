#include <planner/planner_base.h>
#include <yagsbpl/yagsbpl_base.h>
#include <yagsbpl/A_star.h>
#include <unordered_map>

namespace JPS {
class JPS2DUtil : public PlannerBase
{
  public:
    class Node {
      public:
        Vec3i pn;
        Vec3i dir;

        Node() {}
        Node(const Vec3i &n, const Vec3i &d) : pn(n), dir(d) {}

        bool operator==(const Node &n) { return pn == n.pn; }
    };

    template <typename Container> struct container_hash {
      std::size_t operator()(Container const &c) const {
        return boost::hash_range(c.data(), c.data() + 2);
      }
    };

    JPS2DUtil(bool verbose = false);
    bool plan(const Vec3f &start, const Vec3f &goal, decimal_t eps = 1);


    vec_Vec3f ps_;
  protected:
    int getHashBin(Node &n);

    bool isAccessible(Node &n);

    void getSuccessors(Node &n, std::vector<Node> *s, std::vector<float> *c);

    float getHeuristics(Node &n1, Node &n2);

    bool storePath(Node &n);

    bool stopSearch(Node &n);

    vec_Vec3i prune(const Vec3i &pn, const Vec3i &dir);
    bool hasForced(const Vec3i &pn, const Vec3i &dir);
    Vec3i jump(Vec3i pn, Vec3i dir);
    bool linkToGoal(const Vec3i &pn);

    GenericSearchGraphDescriptor<Node, float> myGraph;

    bool _goal_outside;
    Node _goal_node;
    Vec3i Vec3i_null;
    Vec3i _start_int;

    std::unordered_map<Vec3i, vec_Vec3i, container_hash<Vec3i>> _obs_map;
    std::unordered_map<Vec3i, vec_Vec3i, container_hash<Vec3i>> _adds_map;
};
}
