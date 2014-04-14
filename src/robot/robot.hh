#ifndef ROBOT_H_
#define ROBOT_H_
#include <vector>
#include <map>
#include <boost/foreach.hpp>

#include "ode/servo.hh"
#include "ode/motor.hh"
#include "ode/object.hh"
#include "ode/visitor.hh"

namespace robot
{
  class Robot
  {
  public:
    typedef boost::shared_ptr<Robot> ptr_t;
    Robot() {}
    Robot(const Robot& o, ode::Environment& env)
    {
      using namespace ode;
      std::map<const Object*, Object::ptr_t> old_to_new;
      BOOST_FOREACH(Object::ptr_t b, o._bodies)
	{
	  ode::Object::ptr_t b2 = b->clone(env);
	  old_to_new[b.get()] = b2;
	  _bodies.push_back(b2);
	}
      BOOST_FOREACH(Servo::ptr_t s, o._servos)
	_servos.push_back(s->clone(env, 
				   *old_to_new[&s->get_o1()], 
				   *old_to_new[&s->get_o2()]));

      BOOST_FOREACH(Motor::ptr_t s, o._motors)
	_motors.push_back(s->clone(env, 
				   *old_to_new[&s->get_o1()], 
				   *old_to_new[&s->get_o2()]));
      _main_body = old_to_new[o._main_body.get()];
    }

    virtual ptr_t clone(ode::Environment& env) const { return ptr_t(new Robot(*this, env)); }

    const std::vector<ode::Object::ptr_t>& bodies() const { return _bodies; }
    std::vector<ode::Object::ptr_t>& bodies() { return _bodies; }

    const std::vector<ode::Servo::ptr_t>& servos() const { return _servos; }
    std::vector<ode::Servo::ptr_t>& servos() { return _servos; }

    const std::vector<ode::Motor::ptr_t>& motors() const { return _motors; }
    std::vector<ode::Motor::ptr_t>& motors() { return _motors; }

    Eigen::Vector3d pos() const { return _main_body->get_pos(); }
    Eigen::Vector3d rot() const { return _main_body->get_rot(); }
    Eigen::Vector3d vel() const { return _main_body->get_vel(); }
    virtual void accept (ode::ConstVisitor &v) const { v.visit(_bodies); }
 
    virtual void next_step(double dt = ode::Environment::time_step)
    {
      BOOST_FOREACH(ode::Object::ptr_t s, _bodies)
	s->set_in_contact(false);
      BOOST_FOREACH(ode::Servo::ptr_t s, _servos) 
	s->next_step(dt);
      BOOST_FOREACH(ode::Motor::ptr_t s, _motors) 
	s->next_step(dt);
    }
  protected:
    std::vector<ode::Object::ptr_t> _bodies;
    std::vector<ode::Servo::ptr_t> _servos;
    std::vector<ode::Motor::ptr_t> _motors;
    ode::Object::ptr_t _main_body;
  };
}

#endif
