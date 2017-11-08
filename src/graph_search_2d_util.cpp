#include <jps3d/planner/graph_search_2d_util.h>

using namespace JPS;

GraphSearch2DUtil::GraphSearch2DUtil(bool verbose) {
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "GRAPHSEARCH2D PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);

  // -------
  // 0, normal
  // -1, goal is occupied or cannot find a path
  // 1, too close, dont plan
}

bool GraphSearch2DUtil::plan(const Vec3f &start, const Vec3f &goal, decimal_t eps, bool use_jps) {
  if(planner_verbose_){
    std::cout <<"Start: " << start.transpose() << std::endl;
    std::cout <<"Goal:  " << goal.transpose() << std::endl;
    std::cout <<"Epsilon:  " << eps << std::endl;
  }

  path_.clear();
  raw_path_.clear();
  status_ = 0;

  Vec3i start_int = map_util_->floatToInt(start);

  if (!map_util_->isFree(start_int)) {
    if(planner_verbose_)
      printf(ANSI_COLOR_YELLOW "start is not free! cleared" ANSI_COLOR_RESET "\n");
    if(map_util_->isOutSide(start_int)) {
      Vec3i dim = map_util_->getDim();
      if(planner_verbose_)
        printf(ANSI_COLOR_RED "Start is outside [%d, %d, %d], dim: [%d, %d, %d]" ANSI_COLOR_RESET "\n",
               start_int(0), start_int(1), start_int(2),
               dim(0), dim(1), dim(2));
      status_ = 1;
      return false;
    }
    //_map_util->clearAround(_start_int);
    status_ = 1;
    return false;
  }


  Vec3i goal_int = map_util_->floatToInt(goal);
  goal_outside_ = false;
  if((goal_int - start_int).topRows<2>().norm() == 0)
  {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "start and goal are too close! " ANSI_COLOR_RESET "\n");
    status_ = 1;
    return false;
  }

  if (map_util_->isOutSideXYZ(goal_int, 0) ||
      map_util_->isOutSideXYZ(goal_int, 1))
    goal_outside_ = true;
  else
  {
    if (!map_util_->isFree(goal_int) ) {
      Vec3f g = map_util_->intToFloat(goal_int);
      vec_Vec3i poses = map_util_->rayTrace(start, g);
      int id = -1;
      for(int i = poses.size() - 1; i > 0; i-- ){
        if(map_util_->isFree(poses[i]) &&
           map_util_->isFree(poses[i-1]) )
        {
          id = i - 1;
          break;
        }
      }
      if(id >= 0) {
        goal_int = poses[id];
        if(planner_verbose_)
          printf(ANSI_COLOR_CYAN "change goal! " ANSI_COLOR_RESET "\n");

      }
      else{
        if(planner_verbose_)
          printf(ANSI_COLOR_RED "goal is not free! Abort! " ANSI_COLOR_RESET "\n");
        status_ = -1;
        return false;
      }
    }
  }

  Vec3i dim = map_util_->getDim();

  std::vector<char> cmap(dim(0)*dim(1));
  for( int y = 0; y < dim(1); ++y)
    for( int x = 0; x < dim(0); ++x)
    {
      if(map_util_->isOccupied(Vec3i(x,y,0)))
        cmap[x +y*dim(0)] = 1;
      else
        cmap[x +y*dim(0)] = 0;
    }

  graph_search_ = std::make_shared<JPS::GraphSearch>(cmap.data(), dim(0), dim(1), eps, planner_verbose_);
  graph_search_->plan(start_int(0), start_int(1), goal_int(0),  goal_int(1), use_jps);

  std::vector<StatePtr> path = graph_search_->getPath();
  if (path.size() < 1) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "Cannot find a path fron [%f, %f, %f] to [%f, %f, %f], Abort!" ANSI_COLOR_RESET "\n",
	  start(0), start(1), start(2), goal(0), goal(1), goal(2));
    status_ = -1;
    return false;
  }

  //**** raw path, s --> g
  vec_Vec3f ps;
  for (const auto &it : path)
    ps.push_back(map_util_->intToFloat(Vec3i(it->x, it->y, it->z)));
  raw_path_ = ps;
  std::reverse(std::begin(raw_path_), std::end(raw_path_));

  /*
  if (_goal_outside) // if the second last point is frontier, remove goal
    _raw_path.pop_back();
    */

  path_ = removePts(raw_path_);
  path_ = crop(path_);
  path_ = optimize(path_);
  std::reverse(std::begin(path_), std::end(path_));
  path_ = optimize(path_);
  std::reverse(std::begin(path_), std::end(path_));

  return true;
}



