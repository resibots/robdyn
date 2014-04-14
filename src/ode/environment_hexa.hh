
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
      }
    Environment_hexa(float angle) :
      _colision_between_legs(false)
      {
        _init(true,angle);
      }

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
    
      }
    


    void add_leg_object(int leg,ode::Object& o);
    bool get_colision_between_legs(){return _colision_between_legs;}
  protected:
    void _collision(dGeomID o1, dGeomID o2);
    std::set<dGeomID> _leg0_objects;
    std::set<dGeomID> _leg1_objects;
    std::set<dGeomID> _leg2_objects;
    std::set<dGeomID> _leg3_objects;
    std::set<dGeomID> _leg4_objects;
    std::set<dGeomID> _leg5_objects;
    bool _colision_between_legs ;
  
 
    
    
  };
}


#endif      /* !ENVIRONMENT_HEXA_HH_ */
