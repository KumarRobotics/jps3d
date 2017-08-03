#include <jps3d/planner/a_star_util.h>

using namespace JPS;

AStarUtil::AStarUtil(bool verbose) {
  myGraph.getHashBin_fp = boost::bind(&AStarUtil::getHashBin, this, _1);
  myGraph.isAccessible_fp = boost::bind(&AStarUtil::isAccessible, this, _1);
  myGraph.getSuccessors_fp =
    boost::bind(&AStarUtil::getSuccessors, this, _1, _2, _3);
  myGraph.getHeuristics_fp =
    boost::bind(&AStarUtil::getHeuristics, this, _1, _2);
  myGraph.storePath_fp = boost::bind(&AStarUtil::storePath, this, _1);
  myGraph.stopSearch_fp = boost::bind(&AStarUtil::stopSearch, this, _1);
  myGraph.hashTableSize = 0;

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

  _planner_verbose = verbose;
  if(_planner_verbose)
    printf(ANSI_COLOR_CYAN "PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

int AStarUtil::getHashBin(Node &n) {
  int a = std::min(n(1), myGraph.hashTableSize - 1);
  a = std::max(a, 0);
  return a;
}

bool AStarUtil::isAccessible(Node &n) {
  return _map_util->isFree(n) || n == _goal_node;
}

void AStarUtil::getSuccessors(Node &n, std::vector<Node> *s,
                              std::vector<float> *c) {
  s->clear();
  c->clear();

  for (const auto &add : _ns) {
    Vec3i new_pn = n + add;
    if (!_map_util->isFree(new_pn))
      continue;
    Node tn = new_pn;
    s->push_back(tn);
    c->push_back((n - tn).cast<float>().norm());
  }
}

float AStarUtil::getHeuristics(Node &n1, Node &n2) {
  return (n1 - n2).cast<float>().norm();
}

bool AStarUtil::storePath(Node &n) { return n == _goal_node; }

bool AStarUtil::stopSearch(Node &n) { return n == _goal_node; }

bool AStarUtil::plan(const Vec3f &start, const Vec3f &goal, decimal_t eps) {
  _raw_path.clear();
  _path.clear();

  Vec3i start_int = _map_util->floatToInt(start);
  Vec3i goal_int = _map_util->floatToInt(goal);
  Node start_node = start_int;
  _goal_node = goal_int;
  if (_map_util->isOccupied(start_int) || _map_util->isOccupied(goal_int)) {
  if(_planner_verbose)
    printf(ANSI_COLOR_RED "start / goal is occupied!\n" ANSI_COLOR_RESET);
  return false;
  }

  if (_map_util->isOutSide(start_int) || _map_util->isOutSide(goal_int)) {
    if(_planner_verbose)
      printf(ANSI_COLOR_RED "start / goal is outside, Abort!" ANSI_COLOR_RESET
             "\n");
    return false;
  }

  Vec3i dim = _map_util->getDim();
  myGraph.hashTableSize = dim(1) + 1;

  myGraph.SeedNode = start_node;
  myGraph.TargetNode = _goal_node;

  A_star_planner<Node, float> planner;
  planner.setParams(eps, 10); // optional.
  planner.init(myGraph);
  planner.plan();

  std::vector<std::vector<Node>> paths = planner.getPlannedPaths();

  if (paths.empty()) {
    if(_planner_verbose)
    printf(ANSI_COLOR_RED "Cannot find a path, Abort!" ANSI_COLOR_RESET "\n");
    return false;
  }

  vec_Vec3f path;
  for (const auto &it_node : paths[0])
    path.push_back(_map_util->intToFloat(it_node));
  _raw_path = path;

  std::reverse(std::begin(path), std::end(path));
  path.back() = goal;
  path.front() = start;

  vec_Vec3f poses = path;

  poses = removePts(poses);
  //_raw_path = poses;
  poses = optimize(poses);

  _path = poses;

  return true;
}
