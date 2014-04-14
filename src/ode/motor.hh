/*
** motor.hh
** 
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
** 
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef   	MOTOR_HH_
# define   	MOTOR_HH_
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include "object.hh"

namespace ode
{ 
  class Motor
  {
  public:
    typedef boost::shared_ptr<Motor> ptr_t;
    Motor(Environment& env,
	  const Eigen::Vector3d& anchor,
	  const Eigen::Vector3d& axis,
	  Object& o1, Object& o2) :
      _env(env),
      _anchor(anchor),
      _axis(axis),
      _o1(o1), _o2(o2),
      _vel(0),
      _passive(false),
      _power(0),
      _torque(0),
      _efficiency(1),
      _pos(0)
    { _init(); }

    ~Motor()
    {
      dJointDestroy(_hinge);
    }
    Motor(const Motor& s, Environment& env, Object& o1, Object& o2);
    ptr_t clone(Environment& env, Object& o1, Object &o2) const
    { return ptr_t(new Motor(*this, env, o1, o2)); }

    void next_step(float dt);

    /// desired angular velocity
    void set_vel(float v) { _vel = v; }

    /// real angular velocity
    float get_vel() const { return dJointGetHingeAngleRate(_hinge); }
    
    // effiency : real_velocity = efficiency * asked_vel
    float get_efficiency() const { return _efficiency; }
    void set_efficiency(float v) { _efficiency = v; }
    
    const Eigen::Vector3d& get_anchor()	const { return _anchor; }
    const Eigen::Vector3d& get_axis()	const { return _axis; }
    
    void set_anchor(Eigen::Vector3d anchor)
    { 
      _anchor = anchor;
      dJointSetHingeAnchor(_hinge, _anchor.x(), _anchor.y(), _anchor.z());
    }

    void set_axis(Eigen::Vector3d axis)
    { 
      _axis = axis;
      dJointSetHingeAxis(_hinge, _axis.x(), _axis.y(), _axis.z());
    }

    void set_passive()
    {
      _passive = false;
      dJointSetHingeParam (_hinge, dParamFMax, 0);
    }

    const Object& get_o1() const { return _o1; }
    const Object& get_o2() const { return _o2; }
    float get_power() const { return _power; }
    float get_torque() const { return _torque; }
    // emulate an encoder, takes into account the "slippage" (via
    // effiency)
    // -> pos != real pos if efficiency != 1
    double get_pos() const { return _pos; }

  protected:
    void _init();
    
    // attributes
    Environment& _env;
    Eigen::Vector3d _anchor;
    Eigen::Vector3d _axis;
    Object& _o1;
    Object& _o2;
    dJointID _hinge;
    float _vel;		//desired angular velocity
    bool _passive;
    float _power;
    float _torque;
    float _efficiency;
    double _pos;
  };
}


#endif	    /* !MOTOR_HH_ */
