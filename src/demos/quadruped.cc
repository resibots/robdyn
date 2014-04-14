#include <iostream>

#include <boost/foreach.hpp>
#include "ode/environment.hh"
#include "robot/quadruped.hh"
#include "robot/hybrid.hh"
#include "ode/box.hh"
#include "renderer/osg_visitor.hh"

int main()
{
  dInitODE();
    
  renderer::OsgVisitor v;
  ode::Environment env;
  robot::Quadruped quad(env, Eigen::Vector3d(0, 0, 0));
  robot::Hybrid hyb(env, Eigen::Vector3d(2, 0, 0.3));
  float x = 0;
  float ampl = 0.75;
  static const float step = 0.005;
  quad.accept(v);
  hyb.accept(v);

  ode::Environment env2;
  robot::Robot::ptr_t q2 = hyb.clone(env2);
  q2->accept(v);

  std::vector<ode::Box::ptr_t> g;
  for (size_t i = 0; i < 20; ++i)
    {
      ode::Box::ptr_t 
	b(new ode::Box(env, Eigen::Vector3d(i*0.25,0,0), 1000,
		       0.01, 1, 0.15));
      b->fix();
      b->accept(v);
      env.add_to_ground(*b);
      g.push_back(b);
    }
  // for (size_t i = 0; i < 4; ++i)
  //   hyb.motors()[i]->set_efficiency(0);
      
  while(!v.done())
    {
      v.update();
      env.next_step(step);
      env2.next_step(step);
      quad.next_step(step);
      hyb.next_step(step);
      // std::cout<<" ----- "<<std::endl;
      // BOOST_FOREACH(ode::Object::ptr_t o, quad.bodies())
      // 	std::cout<<o->get_pos()<<"\n --"<<std::endl;
      for (size_t i = 0; i < 8; ++i)
	{
	  quad.servos()[i]->set_mode(ode::Servo::M_VEL);
	  quad.servos()[i]->set_vel(2, 0.1* cos(x+ i % 2));	  
	  //	  quad.servos()[i]->set_angle(ode::Servo::TWIST,
	  //	  sin(x+i) * ampl);
	  hyb.servos()[i]->set_angle(ode::Servo::TWIST, sin(x+(i%4)) * ampl);
	}
      for (size_t i = 0; i < 4; ++i)
      	hyb.motors()[i]->set_vel(cos(x) * 2.5);
      x += 0.001;
    }
    
  return 0;
}

