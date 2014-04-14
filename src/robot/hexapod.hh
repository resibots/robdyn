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
	    
	    /*std::cout<<_bodies.size()<<std::endl;
	    for(int i=0; i<_bodies.size();i++)
	    std::cout<< _bodies[i]->get_pos()<<std::endl;*/

        }
        Hexapod(const Hexapod &o, ode::Environment_hexa & env) :
                Robot(o, env)
        {
	 
	  for(int i=1;i<_bodies.size();i++)
	    {
	      env.add_leg_object((i-1)/3,*_bodies[i]);
	    }
        }
        boost::shared_ptr<Hexapod> clone(ode::Environment_hexa& env) const
        {
	 
            return boost::shared_ptr<Hexapod>(new Hexapod(*this, env));
        }
    protected:
        std::vector<int> _brokenLegs;
        void _build(ode::Environment_hexa& env, const Eigen::Vector3d& pos);
    };
}

#endif
