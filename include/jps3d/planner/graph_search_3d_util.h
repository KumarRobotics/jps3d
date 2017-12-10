/**
 * @file graph_search_3d_util.h
 * @brief JPS::GraphSearch3DUtil Class
 */

#include <jps3d/planner/planner_util_base.h>
#include <jps3d/planner/graph_search.h>

namespace JPS {
  /// GraphSearch3DUtil class
  class GraphSearch3DUtil: public PlannerUtilBase {
    public:
      /// Simple 3D planner constructor
      GraphSearch3DUtil(bool verbose = false);
      /** 
       * @brief start planning thread.
       *
       * @param start start coordinate in 3D
       * @param goal goal coordinate in 3D
       * @param eps weight of heuristic, optional, default as 1
       * @param use_jps if true, enable JPS search; else the planner is implementing A*
       */
      bool plan(const Vec3f &start, const Vec3f &goal, decimal_t eps = 1.0, bool use_jps = false);
  };
}
