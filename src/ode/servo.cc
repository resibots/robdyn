/*
** servo.cc
** Login : <mouret@mexico>
** Started on  Fri Sep  8 19:48:06 2006 Jeanbaptiste MOURET
** $Id$
**
** Copyright (C) 2006 Jeanbaptiste MOURET
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
#include <Eigen/Geometry>
#include "servo.hh"
namespace ode
{

  void Servo :: _init()
  {
    _ball = dJointCreateBall(_env.get_world(), 0);
    dJointAttach(_ball, _o1.get_body(), _o2.get_body());
    dJointSetBallAnchor(_ball, _anchor.x(), _anchor.y(), _anchor.z());

    _amotor = dJointCreateAMotor(_env.get_world(), 0);
    dJointAttach(_amotor, _o1.get_body(), _o2.get_body());
     //axis
    dJointSetAMotorAxis(_amotor, 0, 1, 1, 0, 0);
    dJointSetAMotorAxis(_amotor, 2, 2, 0, 1, 0);
     // euler
    dJointSetAMotorMode(_amotor, dAMotorEuler);

     //stops
    dJointSetAMotorParam(_amotor, dParamLoStop, _lim_min[0]);
    dJointSetAMotorParam(_amotor, dParamHiStop, _lim_max[0]);
    dJointSetAMotorParam(_amotor, dParamLoStop2, _lim_min[1]);
    dJointSetAMotorParam(_amotor, dParamHiStop2, _lim_max[1]);
    dJointSetAMotorParam(_amotor, dParamLoStop3, _lim_min[2]);
    dJointSetAMotorParam(_amotor, dParamHiStop3, _lim_max[2]);

     //fmax
    dJointSetAMotorParam(_amotor, dParamFMax, DEFAULT_FMAX);
    dJointSetAMotorParam(_amotor, dParamFMax2, DEFAULT_FMAX);
    dJointSetAMotorParam(_amotor, dParamFMax3, DEFAULT_FMAX);
     //vels
    dJointSetAMotorParam(_amotor, dParamVel, 0);
    dJointSetAMotorParam(_amotor, dParamVel2, 0);
    dJointSetAMotorParam(_amotor, dParamVel3, 0);

     //    _angles(maths::_(0, _angles.size() - 1)) = 0; ??
    _angles = Eigen::Vector3d::Zero();

    _o2.add_servo(this);
    _o1.add_servo2(this);

    dJointSetFeedback(_amotor, &_feedback);
  }

  Servo :: Servo(const Servo& o, Environment& env, Object& o1, Object& o2,bool init) :
    _env(env),
    _anchor(o._anchor),
    _o1(o1), _o2(o2),
    _angles(o._angles),
    _passive(o._passive),
    _power(0),
    _torque(0),
    _vrot(0),
    _mode(o._mode),
    _vel(o._vel),
    _lim_min(o._lim_min),
    _lim_max(o._lim_max),
    _blocked(o._blocked),
    _p(o._p),
    _offset(o._offset)
  {
    if(!init)
      return;
    _ball = dJointCreateBall(_env.get_world(), 0);
    dJointAttach(_ball, _o1.get_body(), _o2.get_body());

    
    dVector3 anchor;
    dJointGetBallAnchor(o._ball, anchor);    
    _anchor = Eigen::Vector3d(anchor[0], anchor[1], anchor[2]);
    dJointSetBallAnchor(_ball, _anchor.x(), _anchor.y(), _anchor.z());

    _amotor = dJointCreateAMotor(_env.get_world(), 0);
    dJointAttach(_amotor, _o1.get_body(), _o2.get_body());
     //axis
    dVector3 axis;
    dJointGetAMotorAxis(o._amotor, 0, axis);
    dJointSetAMotorAxis(_amotor, 0, 1, axis[0], axis[1], axis[2]);
    dJointGetAMotorAxis(o._amotor, 2, axis);
    dJointSetAMotorAxis(_amotor, 2, 2, axis[0], axis[1], axis[2]);
     // euler
    dJointSetAMotorMode(_amotor, dAMotorEuler);

     //stops
    dJointSetAMotorParam(_amotor, dParamLoStop, _lim_min[0]);
    dJointSetAMotorParam(_amotor, dParamHiStop, _lim_max[0]);
    dJointSetAMotorParam(_amotor, dParamLoStop2, _lim_min[1]);
    dJointSetAMotorParam(_amotor, dParamHiStop2, _lim_max[1]);
    dJointSetAMotorParam(_amotor, dParamLoStop3, _lim_min[2]);
    dJointSetAMotorParam(_amotor, dParamHiStop3, _lim_max[2]);

     //fmax
    dJointSetAMotorParam(_amotor, dParamFMax, DEFAULT_FMAX);
    dJointSetAMotorParam(_amotor, dParamFMax2, DEFAULT_FMAX);
    dJointSetAMotorParam(_amotor, dParamFMax3, DEFAULT_FMAX);
     //vels
    dJointSetAMotorParam(_amotor, dParamVel, 0);
    dJointSetAMotorParam(_amotor, dParamVel2, 0);
    dJointSetAMotorParam(_amotor, dParamVel3, 0);

    for (size_t i = 0; i < 3; ++i)
    {
      float a = dJointGetAMotorAngle(o._amotor, i);
      _offset[i] += a;
    }

     //    _angles(maths::_(0, _angles.size() - 1)) = 0; ??
    _angles = Eigen::Vector3d::Zero();

    _o2.add_servo(this);
    _o1.add_servo2(this);

    dJointSetFeedback(_amotor, &_feedback);
  }

  void Servo :: _asserv(unsigned i, float dt)
  {
     //std::cout<<"\nServo::_asserv   dt ="<<dt<<"\n";
    static const float gain = 1.0 / (M_PI * dt);
    float pos = dJointGetAMotorAngle(_amotor, i);
    float error = pos - _angles(i) - _offset[i];
    float vel = -error * gain * _p;
 //   if (vel > 1)//cap velocity
 //     vel = 1;
 //   if (vel < -1)
  //    vel = -1;
    dJointSetAMotorParam(_amotor, _vel_selector(i), vel);
  }

  void Servo :: next_step(float dt)
  {
    if (!_blocked)
    {
      if (_mode == M_POS)
        for (unsigned i = 0; i < _angles.size(); ++i)
          _asserv(i, dt);
      else
        for (unsigned i = 0; i < _vel.size(); ++i)
          if ((_vel(i) < 0 && get_angle(i) > _lim_min(i)) ||
              (_vel(i) > 0 && get_angle(i) < _lim_max(i)))
            dJointSetAMotorParam(_amotor, _vel_selector(i), _vel(i));
          else
            dJointSetAMotorParam(_amotor, _vel_selector(i), 0);
    }

     //probably useless
    dJointGetFeedback(_amotor);

    const dReal*r1 = dBodyGetAngularVel(_o1.get_body());
    Eigen::Vector3d v1(r1[0], r1[1], r1[2]);
    const dReal*r2 = dBodyGetAngularVel(_o2.get_body());
    Eigen::Vector3d v2(r2[0], r2[1], r2[2]);
    /*dVector3 p;
      dJointGetBallAnchor(_ball, p);
    Eigen::Vector3d pos(p[0], p[1], p[2]);
    Eigen::Vector3d v = v1 - v2;
    */

     //get torques
    Eigen::Vector3d t1(_feedback.t1[0], _feedback.t1[1], _feedback.t1[2]);
    Eigen::Vector3d t2(_feedback.t2[0], _feedback.t2[1], _feedback.t2[2]);
    _torque=t1.norm();

    /*
     //forces
    Eigen::Vector3d f1(_feedback.f1[0], _feedback.f1[1], _feedback.f1[2]);
    Eigen::Vector3d f2(_feedback.f2[0], _feedback.f2[1], _feedback.f2[2]);
    */

  }

}
