#include <planner/jps_2d_util.h>

using namespace JPS;

JPS2DUtil::JPS2DUtil(bool verbose) {
  myGraph.getHashBin_fp = boost::bind(&JPS2DUtil::getHashBin, this, _1);
  myGraph.isAccessible_fp = boost::bind(&JPS2DUtil::isAccessible, this, _1);
  myGraph.getSuccessors_fp =
      boost::bind(&JPS2DUtil::getSuccessors, this, _1, _2, _3);
  myGraph.getHeuristics_fp = boost::bind(&JPS2DUtil::getHeuristics, this, _1, _2);
  myGraph.storePath_fp = boost::bind(&JPS2DUtil::storePath, this, _1);
  myGraph.stopSearch_fp = boost::bind(&JPS2DUtil::stopSearch, this, _1);
  myGraph.hashTableSize = 0;

  Vec3i_null = Vec3i(-1, -1, -1);

  Vec3i add = Vec3i::Zero();
  for (add(0) = -1; add(0) <= 1; add(0)++) {
    for (add(1) = -1; add(1) <= 1; add(1)++) {
        if (add == Vec3i::Zero())
          continue;
        _ns.push_back(add);
    }
  }

  _planner_verbose = verbose;
  if(_planner_verbose)
    printf(ANSI_COLOR_CYAN "PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);

  //**** neightbors and moves
  for (auto it : _ns) {
    vec_Vec3i adds;
    vec_Vec3i obs;

    //*** straight move
    if (it.lpNorm<1>() == 1) {
      adds.push_back(it);

      for (auto itt : _ns) {
        if (it.dot(itt) == 0)
          obs.push_back(itt);
      }
    }

    //*** one-d diagonal move
    else if (it.lpNorm<1>() == 2) {
      adds.push_back(it);
      for (int i = 0; i < 2; i++) {
        if (it(i) != 0) {
          Vec3i d = Vec3i::Zero();
          d(i) = it(i);
          adds.push_back(d);
          obs.push_back(-d);
        }
      }
    }

    _adds_map.insert(std::make_pair(it, adds));
    _obs_map.insert(std::make_pair(it, obs));
  }

  _status = 0;
}

vec_Vec3i JPS2DUtil::prune(const Vec3i &pn, const Vec3i &dir) {
  if (dir == Vec3i::Zero())
    return _ns;
  else {
    vec_Vec3i ns;
    for (const auto &it : _adds_map[dir]) {
      if (_map_util->isFree(pn + it))
        ns.push_back(it);
    }

    if (dir.lpNorm<1>() == 1) {
      for (const auto &it : _obs_map[dir]) {
        if (!_map_util->isFree(pn + it))
          if (_map_util->isFree(pn + it + dir))
            ns.push_back(it + dir);
      }
    }

    else if (dir.lpNorm<1>() == 2) {
      for (const auto &it : _obs_map[dir]) {
        if (!_map_util->isFree(pn + it)) {
          if (_map_util->isFree(pn + 2 * it + dir))
            ns.push_back(2 * it + dir);
          Vec3i d = it.cross(dir);
          if (!_map_util->isFree(pn + it + d)) {
            if (_map_util->isFree(pn + 2 * it + dir + d))
              ns.push_back(2 * it + dir + d);
          }
          d = -d;
          if (!_map_util->isFree(pn + it + d)) {
            if (_map_util->isFree(pn + 2 * it + dir + d))
              ns.push_back(2 * it + dir + d);
          }
        }
      }
    }

    return ns;
  }
}

bool JPS2DUtil::hasForced(const Vec3i &pn, const Vec3i &dir) {
  if (dir.lpNorm<1>() == 1) {
    for (const auto &it : _obs_map[dir]) {
      if (!_map_util->isFree(pn + it))
        // if (_map_util->isFree(pn + it + dir))
        return true;
    }
  }

  else if (dir.lpNorm<1>() == 2) {
    for (const auto &it : _obs_map[dir]) {
      if (!_map_util->isFree(pn + it)) {
        // if (_map_util->isFree(pn + 2 * it + dir))
        return true;
      }
    }
  }

  return false;
}

Vec3i JPS2DUtil::jump(Vec3i pn, Vec3i dir) {
  Vec3i n = pn + dir;
  if (!_map_util->isFree(n))
    return Vec3i_null;

  if (n == _goal_node.pn)
    return n;

  if (hasForced(n, dir))
    return n;

  if (dir.lpNorm<1>() >= 2) {
    for (const auto &it : _adds_map[dir]) {
      if (it != dir && jump(n, it) != Vec3i_null)
        return n;
    }
  }

  return jump(n, dir);
}

int JPS2DUtil::getHashBin(Node &n) {
  int a = std::min(n.pn(1), myGraph.hashTableSize - 1);
  a = std::max(a, 0);
  return a;
}

bool JPS2DUtil::isAccessible(Node &n) {
  return _map_util->isFree(n.pn) || n == _goal_node;
}

bool JPS2DUtil::linkToGoal(const Vec3i &pn) {
  if(_map_util->isEdge(pn)){
    return true;
    Vec3f pt = _map_util->intToFloat(pn);
    decimal_t t = atan2(pt(2), sqrt(pt(0) * pt(0) + pt(1) * pt(1)));
    if(fabs(t) < M_PI / 12){
      if(!_map_util->isBlocked(Vec3f(0, 0, 0), pt))
        return true;
    }
  }
  else{
    Vec3f pt = _map_util->intToFloat(pn);
    Vec3f goal_pt = _map_util->intToFloat(_goal_node.pn);
    return !_map_util->isBlocked(pt, goal_pt);
  }
	return false;
}

void JPS2DUtil::getSuccessors(Node &n, std::vector<Node> *s,
                              std::vector<float> *c) {
  s->clear();
  c->clear();

  if (_goal_outside && _map_util->isFree(n.pn) && linkToGoal(n.pn)) {
    s->push_back(_goal_node);
    c->push_back((n.pn - _goal_node.pn).cast<float>().norm());
    //c->push_back((n.pn - _goal_node.pn).lpNorm<Eigen::Infinity>());
  }

  vec_Vec3i ns;
  ns = prune(n.pn, n.dir);

  for (const auto &add : ns) {
    Vec3i new_pn = jump(n.pn, add);
    if (new_pn != Vec3i_null) {
      Node jpn(new_pn, add);
      s->push_back(jpn);
      c->push_back((n.pn - new_pn).cast<float>().norm());
      ps_.push_back(_map_util->intToFloat(new_pn));
    }
  }
}

float JPS2DUtil::getHeuristics(Node &n1, Node &n2) {
  return (n1.pn - n2.pn).cast<float>().norm();
}

bool JPS2DUtil::storePath(Node &n) { return n == _goal_node; }

bool JPS2DUtil::stopSearch(Node &n) { return n == _goal_node; }


bool JPS2DUtil::plan(const Vec3f &start, const Vec3f &goal, decimal_t eps) {
  _path.clear();
  _raw_path.clear();
  _status = 0;
  ps_.clear();

  _start_int = _map_util->floatToInt(start);
  Node start_node(_start_int, Vec3i::Zero());

  if(_planner_verbose){
    std::cout <<"Start: " << start.transpose() << std::endl;
    std::cout <<"Goal:  " << goal.transpose() << std::endl;
  }

  if (!_map_util->isFree(start_node.pn)) {
    if(_planner_verbose)
      printf(ANSI_COLOR_RED "start is not free! " ANSI_COLOR_RESET "\n");
    _status = 1;
    //_map_util->clearAround(start_node.pn);
    return false;
  }


  Vec3i goal_int = _map_util->floatToInt(goal);
  _goal_outside = false;
  if (_map_util->isOutSideXYZ(goal_int, 0) ||
      _map_util->isOutSideXYZ(goal_int, 1))
    _goal_outside = true;
  else
  {
    if(_map_util->isOutSideXYZ(goal_int, 2)) {
      goal_int(2) = _start_int(2);
      if(_planner_verbose)
	printf(ANSI_COLOR_RED "change goal z! " ANSI_COLOR_RESET "\n");
    }


    if (!_map_util->isFree(goal_int)) {
      if(_planner_verbose)
	printf(ANSI_COLOR_RED "goal is not free! Abort! " ANSI_COLOR_RESET "\n");
      _status = 2;
      return false;
    }
  }

  _goal_node = Node(goal_int, Vec3i::Zero());
  Vec3i dim = _map_util->getDim();
  myGraph.hashTableSize = dim(1) + 1;

  myGraph.SeedNode = start_node;
  myGraph.TargetNode = _goal_node;

  A_star_planner<Node, float> planner;
  planner.setParams(eps, 10); // optional.
  planner.init(myGraph);
  planner.plan();

  std::vector<std::vector<Node>> paths = planner.getPlannedPaths();

  if (paths.empty() || (!paths.empty() && paths.front().size() < 2)) {
    if(_planner_verbose)
      printf(ANSI_COLOR_RED "Cannot find a path from [%f, %f, %f] to [%f, %f, %f], Abort!" ANSI_COLOR_RESET "\n",
          start(0), start(1), start(2), goal(0), goal(1), goal(2));
    _status = -1;
    return false;
  }

  //**** raw path, g --> s
  for (const auto &it_node : paths[0])
    _raw_path.push_back(_map_util->intToFloat(it_node.pn));
  std::reverse(std::begin(_raw_path), std::end(_raw_path));

  _path = removePts(_raw_path);
  _path = crop(_path);
  _path = optimize(_path);
  _path.front() = start;
  //_raw_path.front() = start;

  return true;
}

