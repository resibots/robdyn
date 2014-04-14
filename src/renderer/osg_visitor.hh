/*
** osgVisitor.hh
** Login : <mouret@zia.lip6.fr>
** Started on  Wed Jan  4 15:14:31 2006 Jean-baptiste MOURET
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

#ifndef   	OSGVISITOR_HH_
# define   	OSGVISITOR_HH_

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/StateSet>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/TrackballManipulator>

#include "ode/servo.hh"
#include "keyboard.hh"
#include "osg_hud.hh"

namespace renderer
{
  class OsgVisitor : public ode::ConstVisitor
  {
  public:
    typedef enum { FREE, FOLLOW, FIXED} camera_t;
    OsgVisitor(camera_t cam = FOLLOW) :
      _root(new osg::Group()),
      _tm(new osgGA::NodeTrackerManipulator),
      _keh(new KeyboardEventHandler()),
      _first(true),
      _camera_type(cam),
      _node_ref(NULL)
    {
      _viewer.realize();
      osgViewer::WindowSizeHandler *wsh=new osgViewer::WindowSizeHandler;
      _viewer.addEventHandler(wsh);
      //wsh->setToggleFullscreen(false); // ne marche pas...
      //std::cout<<"ToggleFullscreen="<<wsh->getToggleFullscreen()<<std::endl;
      _tm->setRotationMode(osgGA::NodeTrackerManipulator::TRACKBALL);
      _tm->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);

      //      _create_ground();
      if (_camera_type != FIXED)
	_viewer.setCameraManipulator(new osgGA::TrackballManipulator());
      _viewer.addEventHandler(new osgViewer::StatsHandler);
      _viewer.addEventHandler(_keh.get());

      //      _hud = new OsgHud(_shadowed_scene.get());
    }
    void init() {}
    // update must be called after the first visit
    void update()
    {
      _first = false;
      if (_camera_type == FIXED)
	_viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3d(0, -2, 1.25), 
						   osg::Vec3d(0, 0, 0), 
						   osg::Vec3d(0, 0, 1));
      _viewer.frame();
      _update_traj();
    }
    bool done() { return _viewer.done(); }
    virtual void visit(const std::vector<ode::Object::ptr_t>& v);
     virtual void visit(const ode::Box&);
    virtual void visit(const ode::CappedCyl&);
    virtual void visit(const ode::Sphere&);

    void enable_dump(const std::string& prefix);
    
    KeyboardEventHandler* get_keh() { return _keh.get(); }
    OsgHud* get_hud() { return _hud; }
  protected:
    osgViewer::Viewer _viewer;
    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osg::Group> _shadowed_scene;
    osgGA::NodeTrackerManipulator* _tm;
    osg::ref_ptr<KeyboardEventHandler> _keh;
    OsgHud* _hud;
    bool _first;
    camera_t _camera_type;
    void _update_traj();
    void _set_tm(osg::Node* g)
    {
      if (!_node_ref)
	_node_ref = g;
      if (!_tm->getTrackNode() && _camera_type == FOLLOW)
	{
	  _tm->setTrackNode(g);
	  _tm->setNode(g);

	  _tm->setHomePosition(osg::Vec3(-4, -3, 2),
			       osg::Vec3(0, 0, 0),
			       osg::Vec3(0, 0, 1));
	  _viewer.setCameraManipulator(_tm);
	}
    }
    void _create_ground(const ode::Environment& env);
    osg::ref_ptr<osg::Geode> _create_sqr(float width, float length);
    osg::Texture2D* _load_texture(const std::string& fname);
    osg::ref_ptr<osg::Group> _ground;
    osg::Vec3 _prev_pos;
    osg::Node* _node_ref;
  };
}



#endif	    /* !OSGVISITOR_HH_ */
