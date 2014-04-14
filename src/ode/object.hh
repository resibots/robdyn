/*
** object.hh
** Login : <mandor@yog-sothoth>
** Started on  Thu Nov 25 22:45:43 2004 mandor
** $Id$
**
** Copyright (C) 2004 mandor
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

#ifndef         OBJECT_HH_
# define        OBJECT_HH_

#include <iostream>
#include <assert.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include "environment.hh"
#include "visitor.hh"

#include <time.h>
#include <sys/time.h>



namespace ode
{
  static const float density = 1.0f;
  class Servo;

   /// abstract class to represent a rigid body
  class Object
  {
    public:
      typedef boost::shared_ptr<Object> ptr_t;
      Object(Environment & env, const Eigen::Vector3d & pos) :
        _body(0x0),
        _geom(0x0),
        _init_pos(pos),
        _env(env),
        _servo(0x0),
        _servo2(0x0),
        _fix(0x0),
        _bad_state(false),
        _in_contact(false)
      {
      }

      virtual ~Object()
      {
        if (_body)
          dBodyDestroy(_body);
        if (_geom)
          dGeomDestroy(_geom);
      }
      virtual ptr_t clone(Environment& env) const = 0;

      virtual dBodyID get_body()  const { return _body; }
      virtual dGeomID get_geom()  const { return _geom; }
      virtual float get_mass() const { return _m.mass; }
       /// ask the current position to ode
       /// Cf ode documentation for details
      Eigen::Vector3d get_pos()   const { return _get_ode_pos(); }
      Eigen::Vector3d get_rot() const;
      Eigen::Vector3d get_vel() const { return get_vground(); }
       /// set an absolute rotation (euler angles)
      void set_rotation(float phi, float theta, float psi)
      {
        dMatrix3 r;
        dRFromEulerAngles(r, phi, theta, psi);
        dBodySetRotation(get_body(), r);
      }
       /// set an absolute rotation (axis and angle)
      void set_rotation(float ax, float ay, float az, float angle)
      {
        dMatrix3 r;
        dRFromAxisAndAngle(r, ax, ay, az, angle);
        dBodySetRotation(get_body(), r);
      }
      void set_rotation(const Eigen::Vector3d& a1, const Eigen::Vector3d &a2)
      {
        dMatrix3 r;
	dRFrom2Axes (r, a1.x(), a1.y(), a1.z(), a2.x(), a2.y(), a2.z());
        dBodySetRotation(get_body(), r);
      }
       /// const visitor, useful for example for a 3d renderer
      virtual void accept(ConstVisitor& v) const = 0;

       /// connect a servo. Used by Panels (called by the constructor) to
       /// change their shape according to their sweep angle
      void add_servo(Servo*servo) { _servo = servo; }
      void add_servo2(Servo*servo2) { _servo2 = servo2; }

      const Servo& get_servo(void) const { assert(_servo); return *_servo; }
      const Servo& get_servo2(void) const { assert(_servo2); return *_servo2; }

       /// true in case of simulation errors (nan, ...)
      bool get_bad_state() const { return _bad_state;}


       /// ask to ode the speed of a given point in the object
       /// the given point must be in world
       /// the result is in world coordinate
      Eigen::Vector3d get_vground(const Eigen::Vector3d& v)       const
      {
        dReal res[3];
        dBodyGetPointVel(get_body(), v.x(), v.y(), v.z(), res);
        return ode_to_vectorf(res);
      }
       /// get_vground() at CG
      Eigen::Vector3d get_vground() const
      {
        dReal res[3];
        const dReal*pos = dBodyGetPosition(get_body());
        dBodyGetPointVel(get_body(), pos[0], pos[1], pos[2], res);
        return ode_to_vectorf(res);
      }


       ///attach the object to the static environment
      void fix()
      {
        if (_fix)
          return;
        fix_along_axis(Eigen::Vector3d(0, 0, 1));
        dJointSetSliderParam(_fix, dParamLoStop, 0);
        dJointSetSliderParam(_fix, dParamHiStop, 0);
      }
       /// detach the object
      void unfix()
      {
        if (!_fix)
          return;
        dJointDestroy(_fix);
        _fix = 0x0;
      }

       /// attach to the static environment along the axis 'axis'
       /// (the object will be able to move only along this axis)
      void fix_along_axis(const Eigen::Vector3d& axis)
      {
        if (_fix)
          return;
        _fix = dJointCreateSlider(_env.get_world(), 0);
        dJointAttach(_fix, _body, 0);
        dJointSetSliderAxis(_fix, axis[0], axis[1], axis[2]);
      }

       /// true if fixed
      bool get_fix() const { return _fix == 0; }
      const Environment& get_env() const { return _env; }
      void set_in_contact(bool b) { _in_contact = b; }
      bool get_in_contact() const { return _in_contact; }
    protected:
      void init()
      {
        _body = dBodyCreate(_env.get_world());
        dBodySetPosition(_body,
                         _init_pos.x(),
                         _init_pos.y(),
                         _init_pos.z());
        dBodySetData(_body, this);
      }
       // does not copy servos (they must be copied later)
      void _copy(const Object& o)
      {
        _body = dBodyCreate(_env.get_world());
         //
        dQuaternion quat;
        dBodyCopyQuaternion(o._body, quat);
        dVector3 pos;
        dBodyCopyPosition(o._body, pos);
         //
        dBodySetPosition(_body, pos[0], pos[1], pos[2]);
        dBodySetQuaternion(_body, quat);
         //
        dBodyGetMass(o._body, &_m);
        dBodySetMass(_body, &_m);
         //
        _fix = o._fix;
        _init_pos = o.get_pos();
         //
        dBodySetData(_body, this);
      }
      Eigen::Vector3d _get_ode_pos()      const
      {
        assert(get_body());
        const dReal*pos;
        pos = dBodyGetPosition(get_body());
        return Eigen::Vector3d(pos[0], pos[1], pos[2]);
      }
      dMass _m;
      dBodyID _body;
      dGeomID _geom;
      Eigen::Vector3d _init_pos;
      Environment& _env;
      Servo*_servo;
      Servo*_servo2;
      dJointID _fix;
      bool _bad_state;
      bool _in_contact;
  };
}

#endif      /* !OBJECT_HH_ */
