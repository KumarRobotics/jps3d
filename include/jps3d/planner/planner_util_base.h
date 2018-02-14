/**
 * @file planner_util_base.h
 * @brief JPS::PlannerUtilBase Abstract Class
 */
#ifndef JPS_PLANNER_BASE_H
#define JPS_PLANNER_BASE_H

#include <iostream>
#include <jps3d/planner/graph_search.h>
#include <jps3d/basic_type/data_utils.h>
#include <jps3d/collision_checking/voxel_map_util.h>

namespace JPS {
  class GraphSearch;
  /**
   * @brief Abstract base for planning
   */
  class PlannerUtilBase
  {
    public:
      /**
       * @brief Simple constructor
       */
      PlannerUtilBase();

      ///Set map util for collistion checking
      void setMapUtil(std::shared_ptr<JPS::VoxelMapUtil>& map_util);
      ///@brief Status of the planner
      //
      /// 0 --- exit normally;
      /// -1 --- no path found;
      /// 1, 2 --- start or goal is not free.
      int status();
      ///Check if goal is outside of map
      bool goalOutside();
      ///Get the modified path
      vec_Vec3f getPath();
      ///Get the raw path
      vec_Vec3f getRawPath();
      ///Remove redundant waypoints in a line 
      vec_Vec3f removePts(const vec_Vec3f &path);
      ///Remove some corner waypoints
      vec_Vec3f optimize(const vec_Vec3f &path);
      ///Crop the raw path such that the new path will be inside the map
      vec_Vec3f crop(const vec_Vec3f& path);

      ///Need to be specified in Child class, main planning function
      virtual bool plan(const Vec3f &start, const Vec3f &goal, decimal_t eps = 1, bool use_jps = false) = 0;
      ///Get the nodes in open set
      vec_Vec3f getOpenSet() const;
      ///Get the nodes in close set
      vec_Vec3f getCloseSet() const;
      ///Get all the nodes
      vec_Vec3f getAllSet() const;

    protected:
      ///Assume using 3D voxel map for all 2d and 3d planning
      std::shared_ptr<VoxelMapUtil> map_util_;
      ///The planner
      std::shared_ptr<GraphSearch> graph_search_;
      ///Raw path from planner
      vec_Vec3f raw_path_;
      ///Modified path for future usage
      vec_Vec3f path_;
      ///Flag indicating the success of planning
      int status_ = 0;
      ///Enabled for printing info
      bool planner_verbose_;
      ///If goal is outside map, set to True, vice verse.
      bool goal_outside_;

  };
}
#endif
