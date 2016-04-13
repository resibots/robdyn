
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

void Environment_hexa::add_leglastsubsegment_object(ode::Object& o)
{
    _leg_lastsubsegments_objects.push_back(o.get_geom());
}


void Environment_hexa::_collision(dGeomID o1, dGeomID o2)
{
    bool g1_thirdlegsegment, g2_thirdlegsegment;
    assert(_leg_lastsubsegments_objects.size()==6);
    
    int g1 ; g1_thirdlegsegment = false;
    if(_ground_objects.find(o1) != _ground_objects.end())
        g1=-1; // g1 object is the ground
    else if(_leg0_objects.find(o1) != _leg0_objects.end())
        g1=0; // g1 object is one of the three segments of the leg 0 (segments inserted into _leg0_objects in increasing order of distance to main body)
    else if(_leg1_objects.find(o1) != _leg1_objects.end())
        g1=1; // g1 object is one of the three segments of the leg 1 (segments inserted into _leg0_objects in increasing order of distance to main body)
    else if(_leg2_objects.find(o1) != _leg2_objects.end())
        g1=2; // g1 object is one of the three segments of the leg 2 (segments inserted into _leg0_objects in increasing order of distance to main body)
    else if(_leg3_objects.find(o1) != _leg3_objects.end())
        g1=3; // g1 object is one of the three segments of the leg 3 (segments inserted   in increasing order of distance to main body)
    else if(_leg4_objects.find(o1) != _leg4_objects.end())
        g1=4; // g1 object is one of the three segments of the leg 4 (segments inserted   in increasing order of distance to main body)
    else if(_leg5_objects.find(o1) != _leg5_objects.end())
        g1=5; // g1 object is one of the three segments of the leg 5 (segments inserted   in increasing order of distance to main body)
    else
        g1=6; //main body // the g1 object is the

    int g2 ; g2_thirdlegsegment = false;
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

    if(g1==g2 || (g1==6 && g2!=-1) || (g1!=-1 && g2==6)) //contact between same part of the robot (or world)
        return;


    static const int N = 10;
    int i, n;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact)); //N is the maximum number of contacts

    if (n > 0)
    {

        if(g1!=-1 && g2!=-1) //colision between legs
        {
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


        g1_thirdlegsegment = false; g2_thirdlegsegment = false;
        if(g1==-1 && g2>=0 && g2<=5 &&
                std::find(_leg_lastsubsegments_objects.begin(),_leg_lastsubsegments_objects.end(),o2) != _leg_lastsubsegments_objects.end())
            g2_thirdlegsegment = true;

        if(g2==-1 && g1>=0 && g1<=5 &&
                std::find(_leg_lastsubsegments_objects.begin(),_leg_lastsubsegments_objects.end(),o1) != _leg_lastsubsegments_objects.end())
            g1_thirdlegsegment = true;

        size_t GRF_on_leg;
        if(g1_thirdlegsegment || g2_thirdlegsegment)
            g1_thirdlegsegment ? GRF_on_leg = g1 : GRF_on_leg = g2;

        for (i = 0; i < n; i++)
        {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = 0.8;//dInfinity;
            contact[i].surface.slip1 = 0.01;// 0.01;
            contact[i].surface.slip2 = 0.01;//0.01;
            contact[i].surface.soft_erp = 0.1;
            contact[i].surface.soft_cfm = 0.001; //penetration/softness


            dJointID c = dJointCreateContact(get_world(), get_contactgroup(),
                                             &contact[i]);
            dJointAttach(c,
                         dGeomGetBody(contact[i].geom.g1),
                         dGeomGetBody(contact[i].geom.g2));

            if(g1_thirdlegsegment || g2_thirdlegsegment)
            {
                dJointSetFeedback(c, &_feedback_GRF);
                dJointGetFeedback(c);

                if (i == 0)
                    assert(_legs_GRF[GRF_on_leg][0] == 0.0f && _legs_GRF[GRF_on_leg][1] == 0.0f && _legs_GRF[GRF_on_leg][2] == 0.0f);

                if(g1_thirdlegsegment)
                {
                    _legs_GRF[GRF_on_leg][0] += _feedback_GRF.f1[0]; _legs_GRF[GRF_on_leg][1] += _feedback_GRF.f1[1]; _legs_GRF[GRF_on_leg][2] += _feedback_GRF.f1[2];
                }
                else
                {
                    _legs_GRF[GRF_on_leg][0] += _feedback_GRF.f2[0]; _legs_GRF[GRF_on_leg][1] += _feedback_GRF.f2[1]; _legs_GRF[GRF_on_leg][2] += _feedback_GRF.f2[2];
                }
            }
        }
        if(g1_thirdlegsegment || g2_thirdlegsegment)
        {
            _legs_GRF[GRF_on_leg][0] /= n; _legs_GRF[GRF_on_leg][1] /= n; _legs_GRF[GRF_on_leg][2] /= n;
        }
    }
}
} //namespace ode
