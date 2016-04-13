
/** environment.hh
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

#ifndef         ENVIRONMENT_HEXA_HH_
# define        ENVIRONMENT_HEXA_HH_

#include <iostream>
#include <ode/ode.h>
#include <ode/common.h>
#include <set>
#include <vector>
#include "misc.hh"
#include "environment.hh"

namespace ode
{


class Environment_hexa: public Environment
{
public:
    Environment_hexa() :
        _colision_between_legs(false)
    {
        _init(true);
        for (size_t legs=0; legs<6;++legs)
            _legs_GRF.push_back(std::vector <float> (3, 0.0));

        _feedback_GRF.f1[0] = 0.0; _feedback_GRF.f1[1] = 0.0; _feedback_GRF.f1[2] = 0.0;
        _feedback_GRF.f2[0] = 0.0; _feedback_GRF.f2[1] = 0.0; _feedback_GRF.f2[2] = 0.0;
    }

    Environment_hexa(float angle) :
        _colision_between_legs(false)
    {
        _init(true,angle);
        for (size_t legs=0; legs<6;++legs)
            _legs_GRF.push_back(std::vector <float> (3, 0.0));

        _feedback_GRF.f1[0] = 0.0; _feedback_GRF.f1[1] = 0.0; _feedback_GRF.f1[2] = 0.0;
        _feedback_GRF.f2[0] = 0.0; _feedback_GRF.f2[1] = 0.0; _feedback_GRF.f2[2] = 0.0;
    }

#ifdef GRND_OBSTACLES_ROBDYN
    Environment_hexa(float floorangle, std::vector <std::vector <float> > obstacle_pos_rad) :
        _colision_between_legs(false)
    {
        _init_with_obstacles(true, floorangle, obstacle_pos_rad);
        for (size_t legs=0; legs<6;++legs)
            _legs_GRF.push_back(std::vector <float> (3, 0.0));

        _feedback_GRF.f1[0] = 0.0; _feedback_GRF.f1[1] = 0.0; _feedback_GRF.f1[2] = 0.0;
        _feedback_GRF.f2[0] = 0.0; _feedback_GRF.f2[1] = 0.0; _feedback_GRF.f2[2] = 0.0;
    }
#endif

#ifndef GRND_OBSTACLES_ROBDYN
    Environment_hexa(const Environment_hexa& env) :
        _colision_between_legs(false)
    {
        _init(true,env.angle);
        _leg0_objects=env._leg0_objects;
        _leg1_objects=env._leg1_objects;
        _leg2_objects=env._leg2_objects;
        _leg3_objects=env._leg3_objects;
        _leg4_objects=env._leg4_objects;
        _leg5_objects=env._leg5_objects;

        _leg_lastsubsegments_objects=env._leg_lastsubsegments_objects;
        for (size_t legs=0; legs<6;++legs)
            _legs_GRF.push_back(std::vector <float> (3, 0.0));

        _feedback_GRF.f1[0] = 0.0; _feedback_GRF.f1[1] = 0.0; _feedback_GRF.f1[2] = 0.0;
        _feedback_GRF.f2[0] = 0.0; _feedback_GRF.f2[1] = 0.0; _feedback_GRF.f2[2] = 0.0;
    }
#else
    Environment_hexa(const Environment_hexa& env) :
        _colision_between_legs(false)
    {
        _init_with_obstacles(true, env.angle, env.obstacle_pos_rad);
        _leg0_objects=env._leg0_objects;
        _leg1_objects=env._leg1_objects;
        _leg2_objects=env._leg2_objects;
        _leg3_objects=env._leg3_objects;
        _leg4_objects=env._leg4_objects;
        _leg5_objects=env._leg5_objects;

        _leg_lastsubsegments_objects=env._leg_lastsubsegments_objects;
        for (size_t legs=0; legs<6;++legs)
            _legs_GRF.push_back(std::vector <float> (3, 0.0));

        _feedback_GRF.f1[0] = 0.0; _feedback_GRF.f1[1] = 0.0; _feedback_GRF.f1[2] = 0.0;
        _feedback_GRF.f2[0] = 0.0; _feedback_GRF.f2[1] = 0.0; _feedback_GRF.f2[2] = 0.0;
    }
#endif


    void add_leg_object(int leg,ode::Object& o);
    void add_leglastsubsegment_object(ode::Object& o);

    bool get_colision_between_legs(){return _colision_between_legs;}

    std::vector < std::vector <float> > _legs_GRF;
    void resetGRF()
    {
        for (size_t legs=0; legs<6;++legs)
            _legs_GRF[legs] = std::vector <float> (3, 0.0);
    }

    void printGRF()
    {
        for (size_t legs=0; legs<6;++legs)
	{
	    std::cout << std::endl << std::endl;
            for (size_t j=0; j<_legs_GRF[legs].size();++j)
            	std::cout << _legs_GRF[legs][j] << "   ";
	}
	std::cout << std::endl;
	std::cout << std::endl << std::endl;
    }

	

    std::vector <float> getGRF(size_t leg)
    {
        return _legs_GRF[leg];
    }

protected:
    void _collision(dGeomID o1, dGeomID o2);
    std::set<dGeomID> _leg0_objects;
    std::set<dGeomID> _leg1_objects;
    std::set<dGeomID> _leg2_objects;
    std::set<dGeomID> _leg3_objects;
    std::set<dGeomID> _leg4_objects;
    std::set<dGeomID> _leg5_objects;

    std::vector<dGeomID> _leg_lastsubsegments_objects;

    bool _colision_between_legs ;

    dJointFeedback _feedback_GRF;


};
}


#endif      /* !ENVIRONMENT_HEXA_HH_ */
