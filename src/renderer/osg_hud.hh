#ifndef _OSGHUD_H
#define _OSGHUD_H

#include <osg/PositionAttitudeTransform>
#include <osg/Group>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>

#include <osgText/Font>
#include <osgText/Text>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osg/ShapeDrawable>

#include <osg/Geometry>
#include <osg/Texture2D>

#include <string>


namespace renderer 
{
  
  class OsgHud
  {
  public:
    OsgHud(osg::Group*);
  
    void set_text(const std::string& text)
    {	
      //_text->setText(text);
    }
  protected:    
    // Text instance for HUD:
    osgText::Text* _text;

    // points2 
    osg::Vec3* _p2;
    
  };
}
#endif
