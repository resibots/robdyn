#include <iostream>

#include <boost/foreach.hpp>
#include "ode/environment.hh"
#include "ode/capped_cyl.hh"
#include "renderer/osg_visitor.hh"
#include "ode/mx28.hh"
#include "ode/box.hh"
int main()
{
  dInitODE();
  

  renderer::OsgVisitor v;
  ode::Environment env;
  env.set_gravity(0, 0,0);
  std::vector<ode::Object::ptr_t> robot;
  std::vector<ode::Servo::ptr_t> servos;
  
  ode::Object::ptr_t p1
    (new ode::Box(env,Eigen::Vector3d(0, 0.0, 1), 
		  0.1,0.2, 0.1, 0.5));
  robot.push_back(p1);


 
  /*  p1->set_rotation(Eigen::Vector3d(0,1 , 0),
      Eigen::Vector3d(0, 0, 1));*/


  ode::Object::ptr_t p2
    (new ode::Box(env, Eigen::Vector3d(0, 0, 1 + 0.5), 
		  0.1, 0.2,0.1, 0.5));

  robot.push_back(p2);

  ode::Servo::ptr_t s1
    (new ode::Mx28(env, Eigen::Vector3d(0, 0, 1 + 0.25), *p1, *p2));
  servos.push_back(s1);
  s1->set_axis(0, Eigen::Vector3d(1,0,0));
  s1->set_axis(2, Eigen::Vector3d(0,1,0));
 
 p1->set_rotation(0,1,0,M_PI/4);
  p1->fix();
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

