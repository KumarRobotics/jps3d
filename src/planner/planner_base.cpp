#include <planner/planner_base.h>

PlannerBase::PlannerBase() {
  Vec3i add;
  for (add(0) = -1; add(0) <= 1; add(0)++) {
    for (add(1) = -1; add(1) <= 1; add(1)++) {
      for (add(2) = -1; add(2) <= 1; add(2)++) {
        if (add == Vec3i::Zero())
          continue;
        _ns.push_back(add);
      }
    }
  }
  _status = 0;
  //**** status:
  //*** 0 -- exit normally,
  //*** 1 -- start is not free,
  //*** 2 -- goal is occupied
  //*** -1 -- no path when s & g are available
}

void PlannerBase::setMapUtil(VoxelMapUtil *map_util) { _map_util = map_util; }

int PlannerBase::status() { return _status; }

bool PlannerBase::goal_outside() { return _goal_outside; }

vec_Vec3f PlannerBase::getPath() { return _path; }

vec_Vec3f PlannerBase::getRawPath() { return _raw_path; }

vec_Vec3f PlannerBase::removePts(const vec_Vec3f &path) {
  if (path.size() < 3)
    return path;

  vec_Vec3f new_path;
  new_path.push_back(path.front());
  for (unsigned int i = 1; i < path.size() - 1; i++) {
    Vec3f p = (path[i + 1] - path[i]) - (path[i] - path[i - 1]);
    if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
      new_path.push_back(path[i]);
  }
  new_path.push_back(path.back());
  return new_path;
}

vec_Vec3f PlannerBase::crop(const vec_Vec3f& path){
  vec_Vec3f new_path;
  new_path.push_back(path.front());
  for(int i = 1; i < (int) path.size(); i ++)
  {
    Vec3i pn = _map_util->floatToInt(path[i]);
    if(_map_util->isOutSide(pn))
    {
      vec_Vec3i pns = _map_util->rayTrace(path[i-1], path[i]);
      if(pns.size() > 2)
      {
        bool add_end = false;
        for(const auto& it: pns){
          if(_map_util->isOccupied(it)){
            add_end = true;
            break;
          }
        }
        if(!add_end)
          new_path.push_back(_map_util->intToFloat(pns.back()));
      }
      break;
    }
    else
      new_path.push_back(path[i]);
  }
  return new_path;
}

vec_Vec3f PlannerBase::optimize(const vec_Vec3f &path) {
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  vec_Vec3f optimized_path;
  Vec3f pose1 = path[0];
  Vec3f pose2 = path[1];
  Vec3f prev_pose = pose1;
  optimized_path.push_back(pose1);
  decimal_t cost1, cost2, cost3;

  if (!_map_util->isBlocked(pose1, pose2))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<decimal_t>::max();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (!_map_util->isBlocked(pose1, pose2))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<decimal_t>::max();

    if (!_map_util->isBlocked(prev_pose, pose2))
      cost3 = (prev_pose - pose2).norm();
    else
      cost3 = std::numeric_limits<decimal_t>::max();

    if (cost3 < cost1 + cost2)
      cost1 = cost3;
    else {
      optimized_path.push_back(path[i]);
      cost1 = (pose1 - pose2).norm();
      prev_pose = pose1;
    }
  }

  optimized_path.push_back(path.back());
  return optimized_path;
}
