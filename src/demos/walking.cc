#include <iostream>

#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
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
  robot::Hybrid hyb(env, Eigen::Vector3d(2, 0, 0.3));
  static const float step = 0.005;
  hyb.accept(v);

  float init = 0;
  float ampl = 0.3;
  float center = 0.6;
  static const int clockwise = -1;
  static const int counter_clockwise = 1;
  
  using namespace boost::assign;
  std::vector<size_t> b_servos = list_of(0)(3)(4)(7);
  
  float x = 0;
  while(!v.done())
    {
      v.update();      
      env.next_step(step);
      hyb.next_step(step);
      for (size_t i = 0; i < 8; ++i)
	{
	  float phase = (i < 4 ? 0 : 1) * M_PI / 2.0f;
	  bool bot = std::find(b_servos.begin(), b_servos.end(), i) != b_servos.end();
	  int c_shift = (bot ? -1 : 1);
	  hyb.servos()[i]->set_angle(ode::Servo::TWIST, center * c_shift);
	  //				     + (i % 2 ? -1 : 1) * sin(x + phase) * ampl);
	}
      // for (size_t i = 0; i < 4; ++i)
      // 	hyb.motors()[i]->set_vel(cos(x) * 2.5);
      x += 0.01;
    }
    
  return 0;
}

