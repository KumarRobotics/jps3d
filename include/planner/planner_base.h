#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <stack>
#include <boost/bind.hpp>
#include <basic_type/data_utils.h>
#include <collision_checking/jps_voxel_map_util.h>

class PlannerBase
{
 public:
  PlannerBase();

  void setMapUtil(JPS::VoxelMapUtil* map_util);
  int status();
  bool goal_outside();
  vec_Vec3f getPath();
  vec_Vec3f getRawPath();

  vec_Vec3f removePts(const vec_Vec3f &path);
  vec_Vec3f optimize(const vec_Vec3f &path);
  vec_Vec3f crop(const vec_Vec3f& path);

  virtual bool plan(const Vec3f &start, const Vec3f &goal) = 0;

  JPS::VoxelMapUtil* _map_util;

  vec_Vec3i _ns;
  vec_Vec3f _raw_path;
  vec_Vec3f _path;

  int _status;
  bool _planner_verbose;
  bool _goal_outside;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
