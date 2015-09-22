
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

#include "environment_hexa.hh"
#include "object.hh"

namespace ode
{
  void Environment_hexa::add_leg_object(int leg,ode::Object& o)
  {
      switch(leg)
	{
	case 0:
	  _leg0_objects.insert(o.get_geom());
	  break;
	case 1:
	  _leg1_objects.insert(o.get_geom());
	  break;
	case 2:
	  _leg2_objects.insert(o.get_geom());
	  break;
	case 3:
	  _leg3_objects.insert(o.get_geom());
	  break;
	case 4:
	  _leg4_objects.insert(o.get_geom());
	  break;
	case 5:
	  _leg5_objects.insert(o.get_geom());
	  break;
	}
    }

   void Environment_hexa::_collision(dGeomID o1, dGeomID o2)
  {


    int g1 ;
    if(_ground_objects.find(o1) != _ground_objects.end())
      g1=-1;
    else if(_leg0_objects.find(o1) != _leg0_objects.end())
      g1=0;
    else if(_leg1_objects.find(o1) != _leg1_objects.end())
      g1=1;
    else if(_leg2_objects.find(o1) != _leg2_objects.end())
      g1=2;
    else if(_leg3_objects.find(o1) != _leg3_objects.end())
      g1=3;
    else if(_leg4_objects.find(o1) != _leg4_objects.end())
      g1=4;
    else if(_leg5_objects.find(o1) != _leg5_objects.end())
      g1=5;
    else
      g1=6; //main body
    int g2 ;
    if(_ground_objects.find(o2) != _ground_objects.end()) //ground
      g2=-1;
    else if(_leg0_objects.find(o2) != _leg0_objects.end())
      g2=0;
    else if(_leg1_objects.find(o2) != _leg1_objects.end())
      g2=1;
    else if(_leg2_objects.find(o2) != _leg2_objects.end())
      g2=2;
    else if(_leg3_objects.find(o2) != _leg3_objects.end())
      g2=3;
    else if(_leg4_objects.find(o2) != _leg4_objects.end())
      g2=4;
    else if(_leg5_objects.find(o2) != _leg5_objects.end())
      g2=5;
    else
      g2=6; // main body

    //if (!(g1==-1 ^ g2==-1))
    // return;

    if(g1==g2 || (g1==6 && g2!=-1) || (g1!=-1 && g2==6)) //contact between same part of the robot (or world)
      return;



    static const int N = 10;
    int i, n;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

    /*dBodyID b = 0;  //don't work anymore with collision between leg detection
    if (g1 && o2)
      b = dGeomGetBody(o2);
    else if (o1)
      b = dGeomGetBody(o1);
    if (b)
      {
	Object*o = (Object *)dBodyGetData(b);
	if (dBodyGetData(b))
	  o->set_in_contact(true);
	  }*/
    if (n > 0)
      {

	if(g1!=-1 && g2!=-1) //colision between legs
	  {
	    //std::cout<<"colision "<<g1 <<" et "<<g2<<std::endl;
	    _colision_between_legs=true;
	    return;
	  }


	dBodyID b = 0;
	if(g1==-1) //so g2 is in contact with the ground
	  {
	    b = dGeomGetBody(o2);
	  }
	else //so g1 in in contact with the ground
	  {
	    b = dGeomGetBody(o1);
	  }
	Object*o = (Object *)dBodyGetData(b);
	if (dBodyGetData(b))
	  o->set_in_contact(true);

	for (i = 0; i < n; i++)
	  {
	    contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	      dContactSoftERP | dContactSoftCFM | dContactApprox1;
	    contact[i].surface.mu = 0.9;//0.8;//dInfinity;//0.4 for inria?
	      contact[i].surface.slip1 = 0.02;// 0.01;
	      contact[i].surface.slip2 = 0.02;//0.01;
	      contact[i].surface.soft_erp = 0.1;
	      contact[i].surface.soft_cfm = 0.001; //penetration/softness



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
