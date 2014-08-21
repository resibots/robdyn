#include <iostream>

#include <boost/foreach.hpp>
#include "ode/environment.hh"
#include "ode/box.hh"
#include "ode/capped_cyl.hh"
#include "ode/sphere.hh"
#include "ode/motor.hh"
#include "robot/quadruped.hh"
#include "robot/hybrid.hh"
#include "ode/box.hh"
#include "renderer/osg_visitor.hh"


using namespace ode;
using namespace Eigen;

namespace robot
{
  class Rescue : public Robot
  {
  public:
    Rescue(ode::Environment& env, const Eigen::Vector3d& pos) { _build(env, pos); }
  protected:
    void _build(ode::Environment& env, const Eigen::Vector3d& pos);
  };

  void Rescue :: _build(Environment& env, const Vector3d& pos)
  {
    static const double body_mass = 1;
    static const double body_length = 0.75;
    static const double body_width = 0.5;
    static const double body_height = 0.2;
    static const double leg_w = 0.05;
    static const double leg_length = 0.5;
    static const double leg_dist = 0.2;
    static const double leg_mass = 0.2;
    static const double wheel_rad = 0.1;
    static const double wheel_mass = 0.05;

    _main_body = Object::ptr_t
      (new Box(env, pos + Vector3d(0, 0, leg_length),
	       body_mass, body_length, body_width, body_height));
    _bodies.push_back(_main_body);

    for (size_t i = 0; i < 4; ++i)
      {
	int left_right = i > 1 ? -1 : 1;
	int back_front = i % 2 == 0 ? -1 : 1;

	Object::ptr_t l1
	  (new CappedCyl(env, pos + Vector3d(back_front * (leg_dist / 2 + leg_length / 2),
					     left_right * (body_width / 2 + leg_w),
					     leg_length),
			 leg_mass, leg_w, leg_length));
	l1->set_rotation(0, M_PI / 2.0, 0);
	_bodies.push_back(l1);

	Servo::ptr_t s1
	  (new Servo(env, pos + Vector3d(back_front * leg_dist / 2,
					 left_right * (body_width / 2 + leg_w),
					 leg_length),
		     *_main_body, *l1));
	_servos.push_back(s1);

	Object::ptr_t l11
	  (new CappedCyl(env, pos + Vector3d(back_front * (leg_length + leg_dist / 2),
					     left_right * (body_width / 2 + leg_w),
					     leg_length  / 2 + wheel_rad),
			 leg_mass, leg_w, leg_length - wheel_rad * 2));
	_bodies.push_back(l11);
	Servo::ptr_t s2
	  (new Servo(env, pos + Vector3d(back_front * (leg_length + leg_dist / 2),
					 left_right * (body_width / 2 + leg_w),
					 leg_length),
		     *l1, *l11));
	_servos.push_back(s2);

	Object::ptr_t wheel
	  (new Sphere(env, pos + Vector3d(back_front * (leg_length + leg_dist / 2),
					  left_right * (body_width / 2 + leg_w),
					  wheel_rad),
		      wheel_mass, wheel_rad));
	_bodies.push_back(wheel);
	Motor::ptr_t s3
	  (new Motor(env, pos + Vector3d(back_front * (leg_length + leg_dist / 2),
					 left_right * (body_width / 2 + leg_w),
					 wheel_rad),
		     Vector3d(0, 1, 0),
		     *l11, *wheel));
	_motors.push_back(s3);
      }
    for (size_t i = 0; i < _servos.size(); ++i)
      for (size_t j = 0; j < 3; ++j)
	_servos[i]->set_lim(j, -M_PI / 3, M_PI / 3);
  }
}


int main()
{
  dInitODE();

  renderer::OsgVisitor v;
  ode::Environment env(0, 0.25, 0);
  robot::Rescue hyb(env, Eigen::Vector3d(2, 0, 0.05));

  float x = 0;
  float step = 0.001;
  hyb.accept(v);
  while(!v.done())
    {
      v.update();
     env.next_step(step);

      hyb.next_step(step);
      for (size_t i = 0; i < 4; ++i)
      	hyb.motors()[i]->set_vel(cos(x) * 2.5);
    }

  return 0;
}

