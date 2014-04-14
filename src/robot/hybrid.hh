#ifndef ROBOT_HYBRYD_HPP
#define ROBOT_HYBRYD_HPP 
#include "robot/robot.hh"

namespace robot
{
  /// a basic hylos-like quadruped hybrid
  class Hybrid : public Robot
  {
  public:
    Hybrid(ode::Environment& env, const Eigen::Vector3d& pos) { _build(env, pos); }
  protected:
    void _build(ode::Environment& env, const Eigen::Vector3d& pos);
  };
}

#endif
