#include <jps3d/planner/planner_util_base.h>

using namespace JPS;

PlannerUtilBase::PlannerUtilBase() {}

void PlannerUtilBase::setMapUtil(std::shared_ptr<JPS::VoxelMapUtil> &map_util) { map_util_ = map_util; }

int PlannerUtilBase::status() { return status_; }

bool PlannerUtilBase::goalOutside() { return goal_outside_; }

vec_Vec3f PlannerUtilBase::getPath() { return path_; }

vec_Vec3f PlannerUtilBase::getRawPath() { return raw_path_; }

vec_Vec3f PlannerUtilBase::removePts(const vec_Vec3f &path) {
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

vec_Vec3f PlannerUtilBase::crop(const vec_Vec3f& path){
  vec_Vec3f new_path;
  new_path.push_back(path.front());
  for(int i = 1; i < (int) path.size(); i ++)
  {
    Vec3i pn = map_util_->floatToInt(path[i]);
    if(map_util_->isOutSide(pn))
    {
      vec_Vec3i pns = map_util_->rayTrace(path[i-1], path[i]);
      if(pns.size() > 2)
      {
        bool add_end = false;
        for(const auto& it: pns){
          if(map_util_->isOccupied(it)){
            add_end = true;
            break;
          }
        }
        if(!add_end)
          new_path.push_back(map_util_->intToFloat(pns.back()));
      }
      break;
    }
    else
      new_path.push_back(path[i]);
  }
  return new_path;
}

vec_Vec3f PlannerUtilBase::optimize(const vec_Vec3f &path) {
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  vec_Vec3f optimized_path;
  Vec3f pose1 = path[0];
  Vec3f pose2 = path[1];
  Vec3f prev_pose = pose1;
  optimized_path.push_back(pose1);
  decimal_t cost1, cost2, cost3;

  if (!map_util_->isBlocked(pose1, pose2))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<decimal_t>::max();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (!map_util_->isBlocked(pose1, pose2))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<decimal_t>::max();

    if (!map_util_->isBlocked(prev_pose, pose2))
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


vec_Vec3f PlannerUtilBase::getOpenSet() const {
  vec_Vec3f ps;
  std::vector<StatePtr> ss = graph_search_->getOpenSet();
  for(const auto& it: ss) 
    ps.push_back(map_util_->intToFloat(Vec3i(it->x, it->y, it->z)));
  return ps;
}

vec_Vec3f PlannerUtilBase::getCloseSet() const {
  vec_Vec3f ps;
  std::vector<StatePtr> ss = graph_search_->getCloseSet();
  for(const auto& it: ss) 
    ps.push_back(map_util_->intToFloat(Vec3i(it->x, it->y, it->z)));
  return ps;
}

vec_Vec3f PlannerUtilBase::getAllSet() const {
  vec_Vec3f ps;
  std::vector<StatePtr> ss = graph_search_->getAllSet();
  for(const auto& it: ss) 
    ps.push_back(map_util_->intToFloat(Vec3i(it->x, it->y, it->z)));
  return ps;
}
