/*
** motor.cc
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

#include <iostream>
#include "motor.hh"
namespace ode
{
  void Motor :: _init()
  {
      
    _hinge = dJointCreateHinge(_env.get_world(), 0);
    dJointAttach(_hinge, _o1.get_body(), _o2.get_body());
    dJointSetHingeAnchor(_hinge, _anchor.x(), _anchor.y(), _anchor.z());
    dJointSetHingeAxis(_hinge, _axis.x(), _axis.y(), _axis.z());
    // inifinite torque ? 
    dJointSetHingeParam (_hinge, dParamFMax, dInfinity);
    dJointSetHingeParam (_hinge, dParamLoStop, -dInfinity);
    dJointSetHingeParam (_hinge, dParamHiStop, dInfinity);
 
    _vel = 0;
      
    //_o2.add_servo(this);//TODO : add add_motor() function in object ?
    //_o1.add_servo2(this);
  }
  Motor :: Motor(const Motor& o, Environment& env, Object& o1, Object& o2) :
    _env(env),
    _anchor(o._anchor),
    _axis(o._axis),
    _o1(o1), _o2(o2),
    _vel(0),
    _passive(o._passive),
    _power(0),
    _torque(0),
    _efficiency(o._efficiency),
    _pos(0)
  {
    _hinge = dJointCreateHinge(_env.get_world(), 0);
    dJointAttach(_hinge, _o1.get_body(), _o2.get_body());
    dVector3 anchor;
    dJointGetHingeAnchor(o._hinge, anchor);
    _anchor = Eigen::Vector3d(anchor[0], anchor[1], anchor[2]);   
    dJointSetHingeAnchor(_hinge, _anchor.x(), _anchor.y(), _anchor.z());
    
    dVector3 axis;
    dJointGetHingeAxis(o._hinge, axis);
    _axis = Eigen::Vector3d(axis[0], axis[1], axis[2]);   
    dJointSetHingeAxis(_hinge, _axis.x(), _axis.y(), _axis.z());

    // inifinite torque ? 
    dJointSetHingeParam (_hinge, dParamFMax, dInfinity);
    dJointSetHingeParam (_hinge, dParamLoStop, -dInfinity);
    dJointSetHingeParam (_hinge, dParamHiStop, dInfinity);

 
    _vel = 0;
    
  }
    
  void Motor :: next_step(float dt)
  {
    dJointSetHingeParam (_hinge, dParamVel, _vel * _efficiency);
    _pos += _vel * dt;
  }

}
