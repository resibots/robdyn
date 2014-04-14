/*
** misc.hh
** Login : <mouret@zia.lip6.fr>
** Started on  Fri Jan  6 13:49:55 2006 Jean-baptiste MOURET
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

#ifndef   	ODE_MISC_HH_
# define   	ODE_MISC_HH_
#include <Eigen/Core>
#include <ode/ode.h>

namespace cst
{
  static const float rho = 1.225;
  static const float g = 9.80665; 

}

namespace ode
{
  /// from http://www.ode.org/ode-latest-userguide.html
  /// ODE format :
  /// store the matrix by rows, and each row is rounded up to
  /// a multiple of 4 elements. The extra "padding" elements at the
  /// end of each row/column must be set to 0. 
  inline Eigen::Matrix3d ode_to_matrixf(const dReal* dm)
  {
    assert(dm);
    Eigen::Matrix3d mat(3, 3);
    mat(0, 0) = dm[0];
    mat(0, 1) = dm[1];
    mat(0, 2) = dm[2];
    mat(1, 0) = dm[4];
    mat(1, 1) = dm[5];
    mat(1, 2) = dm[6];
    mat(2, 0) = dm[8];
    mat(2, 1) = dm[9];
    mat(2, 2) = dm[10];
    return mat;
  }

  inline Eigen::Vector3d ode_to_vectorf(const dReal* v)
  {
    assert(v);
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }
}


#endif	    /* !ODE_MISC_HH_ */
