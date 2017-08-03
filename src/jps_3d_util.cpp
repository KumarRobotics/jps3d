#include <jps3d/planner/jps_3d_util.h>

using namespace JPS;

JPS3DUtil::JPS3DUtil(bool verbose) {
  myGraph.getHashBin_fp = boost::bind(&JPS3DUtil::getHashBin, this, _1);
  myGraph.isAccessible_fp = boost::bind(&JPS3DUtil::isAccessible, this, _1);
  myGraph.getSuccessors_fp =
      boost::bind(&JPS3DUtil::getSuccessors, this, _1, _2, _3);
  myGraph.getHeuristics_fp = boost::bind(&JPS3DUtil::getHeuristics, this, _1, _2);
  myGraph.storePath_fp = boost::bind(&JPS3DUtil::storePath, this, _1);
  myGraph.stopSearch_fp = boost::bind(&JPS3DUtil::stopSearch, this, _1);
  myGraph.hashTableSize = 0;

  Vec3i_null = Vec3i(-1, -1, -1);

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
      for (int i = 0; i < 3; i++) {
        if (it(i) != 0) {
          Vec3i d = Vec3i::Zero();
          d(i) = it(i);
          adds.push_back(d);
          obs.push_back(-d);
        }
      }
    }

    //*** two-d diagonal move
    else if (it.lpNorm<1>() == 3) {
      for (auto itt : _ns) {
        if ((it - itt).lpNorm<1>() <= 1 ||
            ((it - itt).lpNorm<1>() == 2 && (it - itt).norm() == 1))
          adds.push_back(itt);
      }

      obs.push_back(Vec3i(-it(0), 0, 0));
      obs.push_back(Vec3i(0, -it(1), 0));
      obs.push_back(Vec3i(0, 0, -it(2)));
    }

    _adds_map.insert(std::make_pair(it, adds));
    _obs_map.insert(std::make_pair(it, obs));
  }

  _status = 0;
}

vec_Vec3i JPS3DUtil::prune(const Vec3i &pn, const Vec3i &dir) {
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

    else if (dir.lpNorm<1>() == 3) {
      for (const auto &it : _obs_map[dir]) {
        if (!_map_util->isFree(pn + it)) {
          Vec3i d = dir + it;
          if (_map_util->isFree(pn + d + it))
            ns.push_back(d + it);

          for (int i = 0; i < 3; i++) {
            if (d(i) != 0) {
              Vec3i di(0, 0, 0);
              di(i) = d(i);
              if (_map_util->isFree(pn + di + it))
                ns.push_back(di + it);
            }
          }
        }
      }
    }
    return ns;
  }
}

bool JPS3DUtil::hasForced(const Vec3i &pn, const Vec3i &dir) {
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

  else if (dir.lpNorm<1>() == 3) {
    for (const auto &it : _obs_map[dir]) {
      if (!_map_util->isFree(pn + it)) {
        Vec3i d = dir + it;
        // if (_map_util->isFree(pn + d + it))
        return true;
     }
    }
  }
  return false;
}

Vec3i JPS3DUtil::jump(Vec3i pn, Vec3i dir) {
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

int JPS3DUtil::getHashBin(Node &n) {
  int a = std::min(n.pn(1), myGraph.hashTableSize - 1);
  a = std::max(a, 0);
  return a;
}

bool JPS3DUtil::isAccessible(Node &n) {
  return _map_util->isFree(n.pn) || n == _goal_node;
}


void JPS3DUtil::getSuccessors(Node &n, std::vector<Node> *s,
                              std::vector<float> *c) {
  s->clear();
  c->clear();

  if (_goal_outside && _map_util->isFree(n.pn) && _map_util->isEdge(n.pn)) {
    s->push_back(_goal_node);
    c->push_back((n.pn - _goal_node.pn).cast<float>().norm());
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

float JPS3DUtil::getHeuristics(Node &n1, Node &n2) {
  return (n1.pn - n2.pn).cast<float>().norm();
}

bool JPS3DUtil::storePath(Node &n) { return n == _goal_node; }

bool JPS3DUtil::stopSearch(Node &n) { return n == _goal_node; }


bool JPS3DUtil::plan(const Vec3f &start, const Vec3f &goal, decimal_t eps) {
  _path.clear();
  _raw_path.clear();
  _status = 0;
  ps_.clear();

  const Vec3i start_int = _map_util->floatToInt(start);
  Node start_node(start_int, Vec3i::Zero());

  if(_planner_verbose){
    std::cout <<"Start: " << start.transpose() << std::endl;
    std::cout <<"Goal:  " << goal.transpose() << std::endl;
    std::cout <<"Start Int: " << start_int.transpose() << std::endl;
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
      goal_int(2) = start_int(2);
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

