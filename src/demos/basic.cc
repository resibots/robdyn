#include <iostream>

#include <boost/foreach.hpp>
#include "ode/environment.hh"
#include "ode/capped_cyl.hh"
#include "renderer/osg_visitor.hh"

int main()
{
  dInitODE();

  renderer::OsgVisitor v;
  ode::Environment env;
  
  std::vector<ode::Object::ptr_t> robot;
  std::vector<ode::Servo::ptr_t> servos;
  
  ode::Object::ptr_t p1
    (new ode::CappedCyl(env,Eigen::Vector3d(0, 0.0, 1), 
			0.1, 0.1, 0.5));
  robot.push_back(p1);
  ode::Object::ptr_t p2
    (new ode::CappedCyl(env, Eigen::Vector3d(0, 0.25, 1 + 0.25), 
			0.1, 0.1, 0.5));
  p2->set_rotation(M_PI / 2.0, 0, 0);
  robot.push_back(p2);

  ode::Servo::ptr_t s1
    (new ode::Servo(env, Eigen::Vector3d(0, 0, 1 + 0.25), *p1, *p2));
  servos.push_back(s1);
  
  float x = 0;
  while(!v.done())
    {
      v.visit(robot);
      v.update();
      env.next_step(0.001);
      BOOST_FOREACH(ode::Servo::ptr_t s, servos) 
	s->next_step(0.001);
      s1->set_angle(ode::Servo::DIHEDRAL, cos(x));
      x += 0.01;
    }
    
  return 0;
}

