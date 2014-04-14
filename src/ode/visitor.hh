/*
** visitor.hh
** Login : <mouret@zia.lip6.fr>
** Started on  Wed Jan  4 14:53:53 2006 Jean-baptiste MOURET
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

#ifndef   	VISITOR_HH_
# define   	VISITOR_HH_
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace traits
{

  /// const selectors
  template<typename T>
  struct constify_traits
  {
    typedef const T type;
  };
 
  template<typename T>
  struct id_traits
  {
    typedef T type;
  };
  
  //select iterator
  template<typename T>
  struct select_iterator
  {
    typedef typename T::iterator type;
  };
 
  template<typename T>
  struct select_iterator<const T>
  {
    typedef typename T::const_iterator type;
  };
} //namespace traits


//forward declaration
namespace ode
{
  class Box;
  class Object;
  class CappedCyl;
  class Sphere;
}

/// abstract visitor
/// check the book "design patterns" from Gamma et al for general
/// explanation about the visitor pattern
/// The template trick allows to declare const and non-const visitor
/// at the same time.
namespace ode
{
  template < template<typename> class Const >
  class GenVisitor
  {
  public:
    virtual void visit(typename Const<Box>::type& e) {};
    virtual void visit(typename Const<CappedCyl>::type& e) {}
    virtual void visit(typename Const<Sphere>::type& e) {}
    typedef boost::shared_ptr<Object> obj_ptr_t;
    virtual void visit(typename Const<std::vector<obj_ptr_t> >::type& l) {}
    virtual ~GenVisitor () {}
  };
  
  typedef GenVisitor<traits::constify_traits> ConstVisitor;
  typedef GenVisitor<traits::id_traits> Visitor;
}
#endif	    /* !VISITOR_HH_ */
