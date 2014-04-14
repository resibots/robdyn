
/*
** object.cc
** Login : <mandor@ithaqua>
** Started on  Tue Jun 15 21:10:07 2010 mandor
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
#include "object.hh"

namespace ode
{

    //x-y-z convention
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // see also : http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler
  // ode : [ w, x, y, z ], where w is the real part and (x, y, z) form the vector part.
  Eigen::Vector3d Object::get_rot() const
  {
    const dReal* q = dBodyGetQuaternion(get_body());
    double
      phi = 0, // bank
      theta = 0, // attitude
      psi = 0; // heading
    double k = q[1] * q[2] + q[3] * q[0];

    if (fabs(k - 0.5) < 1e-5) // north pole
      {
	psi = 2 * atan2(q[1], q[0]);
	theta = M_PI / 2.0;
	phi = 0;
      }
    else if ((k + 0.5) < 1e-5)// south pole
      {
	psi = -2 * atan2(q[1], q[0]);
	theta = -M_PI / 2.0;
	phi = 0;
      }
    else
      {
	phi = atan2(2 * (q[0] * q[1] + q[2] * q[3]),
		    1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	theta = asin(2 * (q[0] * q[2] -q[3] * q[1]));
	psi = atan2(2 * (q[0] * q[3] + q[1] * q[2]),
		    1 - 2 * (q[2] * q[2] + q[3] * q[3]));
      }
    return Eigen::Vector3d(phi, theta, psi);
  }
}
