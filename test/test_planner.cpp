#include "map_reader.hpp"
#include <collision_checking/jps_voxel_map_util.h>
#include <planner/jps_2d_util.h>

using namespace JPS;

std::unique_ptr<JPS2DUtil> planner_util_;

void solve(const Vec3f& start, const Vec3f& goal) {
  bool valid = planner_util_->plan(start, goal);
}


int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!" ANSI_COLOR_RESET);
    return -1;
  }

  MapReader<Vec3i, Vec3f> reader(argv[1]);
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot find input file [%s]!" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  std::unique_ptr<VoxelMapUtil> map_util;
  map_util.reset(new VoxelMapUtil);
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  map_util->dilate(0.1, 0.1);
  map_util->dilating();

  planner_util_.reset(new JPS2DUtil(true));
  planner_util_->setMapUtil(map_util.get());

  return 0;
}
