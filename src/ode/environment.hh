/*
** environment.hh
** Login : <mandor@yog-sothoth>
** Started on  Tue Aug  3 15:10:20 2004 mandor
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

#ifndef         ENVIRONMENT_HH_
# define        ENVIRONMENT_HH_

#include <iostream>
#include <boost/config.hpp>
#include <ode/ode.h>
#include <ode/common.h>
#include <set>
#include "misc.hh"

namespace ode
{
  class Object;
   //singleton : only one env
  class Environment
  {
    public:
      BOOST_STATIC_CONSTEXPR float time_step = 0.05;
       // constructor
    Environment() :
        _ground(0x0), _pitch(0), _roll(0), _z(0)
      {
        _init(true);
      }
    Environment(float angle) :
        _ground(0x0), _pitch(0), _roll(0), _z(0)
      {
        _init(true,angle);
      }
    Environment(float pitch, float roll, float z) :
        _ground(0x0), _pitch(pitch), _roll(roll), _z(z)
      {
        _init(true);
      }

    Environment(bool add_ground) :
        _ground(0x0), _pitch(0), _roll(0), _z(0)
      {
        _init(add_ground);
      }
     ~Environment()
      {

        if (_ground)
          dGeomDestroy(_ground);
        dSpaceDestroy(get_space());
        dWorldDestroy(get_world());
        dJointGroupDestroy(_contactgroup);
      }
       //interfaces
      dWorldID get_world()        const
      {
        return _world_id;
      }
      dSpaceID get_space()        const
      {
        return _space_id;
      }
      dGeomID get_ground()        const
      {
        return _ground;
      }
      dJointGroupID get_contactgroup() const
      {
        return _contactgroup;
      }
       //update sim
      void next_step(double dt = time_step)
      {
         //check collisions
        dSpaceCollide(_space_id, (void *)this, &_near_callback);
         //next step
        dWorldStep(_world_id, dt);
         //dWorldQuickStep(_world_id, dt);
         // remove all contact joints
        dJointGroupEmpty(_contactgroup);
      }
      void disable_gravity()
      {
        dWorldSetGravity(_world_id, 0, 0, 0);
      }
      void set_gravity(float x, float y, float z)
      {
        dWorldSetGravity(_world_id, x, y, z);
      }
      void add_to_ground(ode::Object& o);
      float get_pitch() const { return _pitch; }
      float get_roll() const { return _roll; }
      float get_z() const { return _z; }
    protected:
      std::set<dGeomID> _ground_objects;
    void _init(bool add_ground,float angle=0);
      static void _near_callback(void *data, dGeomID o1, dGeomID o2)
      {
        Environment*env = reinterpret_cast<Environment *>(data);
        env->_collision(o1, o2);
      }
      virtual void _collision(dGeomID o1, dGeomID o2);
    public: // ??
       // attributes
      dWorldID _world_id;
      dSpaceID _space_id;
      dGeomID _ground;
      dJointGroupID _contactgroup;
      float _pitch, _roll, _z;
    float angle;
  };
}


#endif      /* !ENVIRONMENT_HH_ */
