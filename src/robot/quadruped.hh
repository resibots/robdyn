#ifndef ROBOT_QUADRUPED_HPP
#define ROBOT_QUADRUPED_HPP
#include "robot/robot.hh"
#include "ode/ax12.hh"

namespace robot
{
  /// a basic hylos-like quadruped
  class Quadruped : public Robot
  {
  public:
    Quadruped(ode::Environment& env, const Eigen::Vector3d& pos) { _build(env, pos); }
  protected:
    void _build(ode::Environment& env, const Eigen::Vector3d& pos);
  };
}

#endif
