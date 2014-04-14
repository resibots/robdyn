/*
** environment.cc
** Login : <mandor@ithaqua>
** Started on  Mon Jun 14 16:04:28 2010 mandor
** $Id$
**
** Copyright (C) 2010 mandor
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

#include "environment.hh"
#include "object.hh"

namespace ode
{
  void Environment::add_to_ground(ode::Object& o)
  {
    _ground_objects.insert(o.get_geom());
  }
  void Environment::_init(bool add_ground, float _angle)
  {
     //create world
    _world_id = dWorldCreate();
     //init gravity
    dWorldSetGravity(_world_id, 0, 0, -cst::g);
     //space
    _space_id = dHashSpaceCreate(0);
     //ground
    if (add_ground)
    {
      angle=_angle;
      Eigen::Vector3f normal(0, sin(-angle), cos(-angle));
      Eigen::Quaternion<float> q1, q2;
      q1 = Eigen::AngleAxis<float>(_pitch, Eigen::Vector3f::UnitX());
      q2 = Eigen::AngleAxis<float>(_roll, Eigen::Vector3f::UnitY());
      normal = q2 * q1 * normal;
      _ground = dCreatePlane(_space_id, normal.x(), normal.y(), normal.z(), _z);
      _ground_objects.insert(_ground);
    }
     //contact group1
    _contactgroup = dJointGroupCreate(0);

     // TODO :
     // void  dWorldSetContactMaxCorrectingVel (dWorldID, dReal vel);
     // dReal dWorldGetContactMaxCorrectingVel (dWorldID);
    dWorldSetContactMaxCorrectingVel(_world_id, 100);
    // Set and get the maximum correcting velocity that contacts are allowed to generate. The default value is infinity (i.e. no limit). Reducing this value can help prevent "popping" of deeply embedded objects.

    // void  dWorldSetContactSurfaceLayer (dWorldID, dReal depth);
    // dReal dWorldGetContactSurfaceLayer (dWorldID);// 0.001
  }
  void Environment::_collision(dGeomID o1, dGeomID o2)
  {
    int g1 = (_ground_objects.find(o1) != _ground_objects.end());
    int g2 = (_ground_objects.find(o2) != _ground_objects.end());

    if (!(g1 ^ g2))
      return;

    static const int N = 10;
    int i, n;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

    dBodyID b = 0;
    if (g1 && o2)
      b = dGeomGetBody(o2);
    else
    if (o1)
      b = dGeomGetBody(o1);
    if (b)
    {
      Object*o = (Object *)dBodyGetData(b);
      if (dBodyGetData(b))
        o->set_in_contact(true);
    }
    if (n > 0)
    {
      for (i = 0; i < n; i++)
      {
        contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                                  dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.slip1 = 0.001;// 0.01;
        contact[i].surface.slip2 = 0.001;//0.01;
        contact[i].surface.soft_erp = 0.5;
        contact[i].surface.soft_cfm = 0.05; //penetration/softness

        dJointID c = dJointCreateContact(get_world(), get_contactgroup(),
                                         &contact[i]);
        dJointAttach(c,
                     dGeomGetBody(contact[i].geom.g1),
                     dGeomGetBody(contact[i].geom.g2));

        // grass
        // dBodyID obj = 0;
        // if (g1 && o1 == _ground) // g2 is our object
        //   obj = dGeomGetBody(o2);
        // else if (g2 && o2 == _ground)
        //   obj = dGeomGetBody(o1);
        // if (obj)
        //   {
        //      dVector3 vel;
        //      dBodyGetPointVel(obj,
        //                       contact[i].geom.pos[0],
        //                       contact[i].geom.pos[1],
        //                       contact[i].geom.pos[2],
        //                       vel);
        //      dBodyAddForce(obj, -vel[0]*200, -vel[1]*200, 0);
        //      std::cout<<"vel:"<<vel[0]<<std::endl;
        //   }
      }
    }
  }

} //namespace ode
