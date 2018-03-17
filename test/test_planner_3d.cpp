#include "timer.hpp"
#include "read_map.hpp"
#include <jps3d/common/data_utils.h>
#include <jps3d/planner/planner_util.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace JPS;

int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read the map from yaml
  MapReader<Vec3i, Vec3f> reader(argv[1], true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  // store map in map_util
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  const Vec3f start(reader.start(0), reader.start(1), reader.start(2));
  const Vec3f goal(reader.goal(0), reader.goal(1), reader.goal(2));

  std::unique_ptr<GraphSearch3DUtil> planner_jps(new GraphSearch3DUtil(true)); // Declare a planner
  planner_jps->setMapUtil(map_util); // Set collision checking function

  std::unique_ptr<GraphSearch3DUtil> planner_astar(new GraphSearch3DUtil(false)); // Declare a planner
  planner_astar->setMapUtil(map_util); // Set collision checking function

  Timer time_jps(true);
  planner_jps->updateMap();
  bool valid_jps = planner_jps->plan(start, goal, 1, true); // Plan from start to goal using JPS
  double dt_jps = time_jps.Elapsed().count();
  printf("JPS Planner takes: %f ms\n", dt_jps);
  printf("JPS Path Distance: %f\n", total_distance3f(planner_jps->getRawPath()));

  Timer time_astar(true);
  planner_astar->updateMap();
  bool valid_astar = planner_astar->plan(start, goal, 1, false); // Plan from start to goal using A*
  double dt_astar = time_astar.Elapsed().count();
  printf("AStar Planner takes: %f ms\n", dt_astar);
  printf("AStar Path Distance: %f\n", total_distance3f(planner_astar->getRawPath()));

  return 0;
}
