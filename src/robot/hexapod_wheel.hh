#ifndef ROBOT_HEXAPOD_HPP
#define ROBOT_HEXAPOD_HPP
#include "robot/robot.hh"
#include "ode/environment_hexa.hh"
namespace robot
{
    class Hexapod : public Robot
    {
    public:
        Hexapod(ode::Environment_hexa & env, const Eigen::Vector3d & pos,std::vector<int> brokenLegs):_brokenLegs(brokenLegs)
        {
            _build(env, pos);
        }
        Hexapod(const Hexapod &o, ode::Environment_hexa & env) :
                Robot(o, env)
        {
        }
        boost::shared_ptr<Robot> clone(ode::Environment_hexa& env) const
        {
            return boost::shared_ptr<Robot>(new Hexapod(*this, env));
        }
    protected:
        std::vector<int> _brokenLegs;
        void _build(ode::Environment_hexa& env, const Eigen::Vector3d& pos);
    };
}

#endif
