/**
 * @file jps_3d_util.h
 * @brief JPS::JPS3DUtil Class
 */
#ifndef JPS_JPS3D_UTIL_H
#define JPS_JPS3D_UTIL_H

#include <planner/planner_base.h>
#include <yagsbpl/yagsbpl_base.h>
#include <yagsbpl/A_star.h>
#include <unordered_map>

namespace JPS {
 /**
   * @brief 3D JPS Planner
   */
class JPS3DUtil : public PlannerBase
{
  public:
    ///Node class for JPS3D
    class Node {
      public:
        Vec3i pn; /**< Indices of this node */
        Vec3i dir; /**< Direction from its paretn toward the node*/

        Node() {}
        Node(const Vec3i &n, const Vec3i &d) : pn(n), dir(d) {}

        bool operator==(const Node &n) { return pn == n.pn; }
    };

    template <typename Container> struct container_hash {
      std::size_t operator()(Container const &c) const {
        return boost::hash_range(c.data(), c.data() + 2);
      }
    };

    JPS3DUtil(bool verbose = false);

    bool plan(const Vec3f &start, const Vec3f &goal, decimal_t eps = 1);

    vec_Vec3f ps_; /**< Debug var that stores expanded nodes*/

  protected:
    int getHashBin(Node &n);

    bool isAccessible(Node &n);

    void getSuccessors(Node &n, std::vector<Node> *s, std::vector<float> *c);

    float getHeuristics(Node &n1, Node &n2);

    bool storePath(Node &n);

    bool stopSearch(Node &n);

    ///Pruning function
    vec_Vec3i prune(const Vec3i &pn, const Vec3i &dir);
    ///Checking if current node is forced
    bool hasForced(const Vec3i &pn, const Vec3i &dir);
    ///Jump function
    Vec3i jump(Vec3i pn, Vec3i dir);

    GenericSearchGraphDescriptor<Node, float> myGraph;

    Vec3i Vec3i_null;/**< Null node*/
    Node _goal_node;

    std::unordered_map<Vec3i, vec_Vec3i, container_hash<Vec3i>> _obs_map;
    std::unordered_map<Vec3i, vec_Vec3i, container_hash<Vec3i>> _adds_map;
};
}
#endif
