/*
** cappedCyl.hh
** Login : <mouret@zia.lip6.fr>
** Started on  Wed Jan  4 16:54:53 2006 Jean-baptiste MOURET
** $Id$
**
** Copyright (C) 2006 Jean-baptiste MOURET
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

#ifndef   	CAPPEDCYL_HH_
# define   	CAPPEDCYL_HH_

#include <ode/ode.h>

#include "object.hh"


namespace ode
{
  /// this class encapsulates the CappedCylinder class in ODE
  /// WARNING: length include the cap!
  class CappedCyl : public Object
  {
  public:
    CappedCyl(Environment& env,
	      const Eigen::Vector3d& pos,
	      float mass,
	      float radius, float length) :
      Object(env, pos),
      _mass(mass),
      _radius(radius),
      _length(length)
    {
      init();
    }
    CappedCyl(const CappedCyl& o, Environment& env) :
      Object(env, o.get_pos()),
      _mass(o._mass),
      _radius(o._radius),
      _length(o._length)
    {
      _copy(o);
      _geom = dCreateCCylinder(_env.get_space(), _radius, _length - _radius * 2);
      dGeomSetBody(_geom, _body);
    }

    virtual Object::ptr_t clone(Environment& env) const
    { return Object::ptr_t(new CappedCyl(*this, env)); }

    float get_radius()	const
    {
      return _radius;
    }
    float get_length()	const
    {
      return _length;
    }
    /// const visitor
    virtual void accept (ConstVisitor &v) const
    {
      assert(_geom);
      assert(_body);
      v.visit(*this);
    }
  protected:
    void init()
    {
      Object::init();
      _geom = dCreateCCylinder(_env.get_space(), _radius, _length - _radius * 2);
      assert(_body);
      assert(_geom);
      /*dMassSetCylinderTotal(&_m, _mass,
				  3, // direction = 3 (along z)
				  _radius, _length - _radius * 2);*/
       dMassSetCapsuleTotal(&_m, _mass,
				  3, // direction = 3 (along z)
				  _radius, _length - _radius * 2);
      dBodySetMass(_body, &_m);
      dGeomSetBody(_geom, _body);

    }
    // atributes
    float _mass;
    float _radius;
    float _length;
  };
}



#endif	    /* !CAPPEDCYL_HH_ */
