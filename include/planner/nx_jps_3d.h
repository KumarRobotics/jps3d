#include <planner/planner_base.h>
#include <planner/jps_3D.h>
#include <iostream>

class NXJPS3DUtil : public PlannerBase {
public:
  NXJPS3DUtil(bool verbose = false);
  bool plan(const Vec3f &start, const Vec3f &goal);
private:
  bool linkToGoal(const Vec3i &pn);

  Vec3i _start_int;
  Vec3i _goal_int;
};
