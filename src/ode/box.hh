/*
** box.hh
** Login : <mandor@yog-sothoth>
** Started on  Thu Nov 25 22:50:53 2004 mandor
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

#ifndef   	BOX_HH_
# define   	BOX_HH_

#include <iostream>
#include "object.hh"

namespace ode
{
  /// a simple wrapper around the ODE's box class
  class Box : public Object
  {
  public:
    BOOST_STATIC_CONSTEXPR  float standard_mass = 1;
    Box(Environment& env, 
	const Eigen::Vector3d& pos, 
	float mass, 
	float l, float w, float h) :
      Object(env, pos),
      _w(w), _h(h), _l(l), _mass(mass)
    {
      init();
    }

    Box(const Box& o, Environment& env) :
      Object(env, o.get_pos()),
      _w(o._w), _h(o._h), _l(o._l),
      _mass(o._mass)
    {
      _copy(o);
      _geom = dCreateBox(_env.get_space(), _l , _w, _h);
      dGeomSetBody(_geom, _body);
    }

    virtual Object::ptr_t clone(Environment& env) const 
    { return Object::ptr_t(new Box(*this, env)); }

    float get_length()	const { return _l; }
    float get_width()	const { return _w; }
    float get_height()	const { return _h; }
 
    //set
    void set_length(float l) { _l = l; init_again(); }
    void set_width(float w) { _w = w; init_again(); }
    void set_height(float h) { _h = h; init_again(); }

    void init_again()
    {
      if (_body)
	dBodyDestroy(_body);
      if (_geom)
	dGeomDestroy(_geom);
      init();
    }
  
    /// const visitor
    virtual void accept (ConstVisitor &v) const
    {
      assert(_body); assert(_geom);
      v.visit(*this);
    }
  protected:
    void init()
    {
      Object::init();
      dMassSetBoxTotal (&_m, _mass, _l, _w, _h);
      dBodySetMass (_body, &_m);
      _geom = dCreateBox(_env.get_space(), _l , _w, _h);
      dGeomSetBody(_geom, _body);
    }
    float _w, _h, _l;
    float _mass;
  };
}


#endif	    /* !BOX_HH_ */
