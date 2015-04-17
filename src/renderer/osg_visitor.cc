#include <osg/PositionAttitudeTransform>
#include <osg/Group>
#include <osg/Node>
#include <osg/Geometry>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/StandardShadowMap>
#include <osgShadow/ShadowTexture>
#include <osgShadow/MinimalDrawBoundsShadowMap>
#include <osgShadow/MinimalCullBoundsShadowMap>
#include <osgShadow/ParallelSplitShadowMap>
#include <osgShadow/LightSpacePerspectiveShadowMap>
//#include <osgShadow/ViewDependentShadowMap>

#include <osgDB/WriteFile>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include "osg_visitor.hh"
#include "ode/capped_cyl.hh"
#include "ode/box.hh"
#include "ode/sphere.hh"

#include <iostream>
#include <iomanip>

#include "shadows.hh"

using namespace osg;
namespace renderer
{

  const int ReceivesShadowTraversalMask = 0x2;
  const int CastsShadowTraversalMask = 0x1;

  inline Quat _osg_quat(const ode::Object& b)
  {
    const dReal*q = dBodyGetQuaternion(b.get_body());
    return Quat(q[1], q[2], q[3], q[0]);
  }

  inline Vec3 _osg_pos(const ode::Object& b)
  {
    Eigen::Vector3d p = b.get_pos();
    return Vec3(p.x(), p.y(), p.z());
  }


   // apply ode transformation
  class UpdateCallback : public NodeCallback
  {
    public:
      UpdateCallback(const ode::Object & obj) : _obj(obj){
      }

      virtual void operator() (Node * node, NodeVisitor * nv)
      {
        PositionAttitudeTransform *pat =
          dynamic_cast<PositionAttitudeTransform *>(node);
        pat->setPosition(_osg_pos(_obj));
        pat->setAttitude(_osg_quat(_obj));
        traverse(node, nv);
      }
    protected:
      const ode::Object& _obj;
  };

  struct ImgPostDrawCallback : public osg::Camera::DrawCallback
  {
    ImgPostDrawCallback(const std::string& prefix):
      _prefix(prefix),
      _image(new osg::Image),
      _k(0)
    {}

    virtual void operator () (const osg::Camera &camera) const
    {
      int x = (int) camera.getViewport()->x();
      int y = (int) camera.getViewport()->y();
      int width = (int) camera.getViewport()->width(); 
      int height = (int )camera.getViewport()->height();

      _image->readPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE);
      osgDB::writeImageFile(*_image, 
			    _prefix + boost::str(boost::format("%1%")%boost::io::group(std::setfill('0'),std::setw(3),_k++))+".png");

    }
  private:
    std::string _prefix;
    osg::ref_ptr<osg::Image> _image;
    mutable size_t _k;
  };



  void OsgVisitor :: enable_dump(const std::string& prefix)
  { 
    _viewer.getCamera()->setPostDrawCallback(new ImgPostDrawCallback(prefix));
  }


  Texture2D *OsgVisitor :: _load_texture(const std::string& fname)
  {
    Texture2D*texture = new Texture2D();
    Image*image = osgDB::readImageFile(fname);
    assert(image);
    texture->setImage(image);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    texture->setDataVariance(osg::Object::DYNAMIC);
    return texture;
  }

  void OsgVisitor :: visit(const ode::Box& b)
  {
    if (!_first)
      return;
    ref_ptr<Geode> geode(new Geode);
    geode->
    addDrawable(new ShapeDrawable
                  (new Box(Vec3f(),
                           b.get_length(),
                           b.get_width(),
                           b.get_height())));
    ref_ptr<PositionAttitudeTransform>  pat(new PositionAttitudeTransform());
    pat->addChild(geode.get());
    ref_ptr<NodeCallback> cb(new UpdateCallback(b));
    pat->setUpdateCallback(cb.get());
    _root->addChild(pat.get());
    _set_tm(geode.get());
  }

  void OsgVisitor :: visit(const ode::Sphere& b)
  {
    float radius = b.get_rad();
    ref_ptr<Geode> geode(new Geode);
    geode->
    addDrawable(new ShapeDrawable
                  (new Sphere(Vec3f(),
                              radius)));

    ref_ptr<PositionAttitudeTransform>  pat(new PositionAttitudeTransform());
    pat->addChild(geode.get());

    ref_ptr<NodeCallback> cb(new UpdateCallback(b));
    pat->setUpdateCallback(cb.get());
    _root->addChild(pat.get());
  }



  ref_ptr<Geode> OsgVisitor :: _create_sqr(float width, float length)
  {
    ref_ptr<Geometry> sqr(new Geometry);
    ref_ptr<Geode> geode_sqr(new Geode);
    ref_ptr<Vec3Array> sqr_v(new Vec3Array);

    geode_sqr->addDrawable(sqr.get());
    float x = width;
    float y = length;
    sqr_v->push_back(Vec3(0, 0, 0));
    sqr_v->push_back(Vec3(0, y, 0));
    sqr_v->push_back(Vec3(x, y, 0));
    sqr_v->push_back(Vec3(x, 0, 0));
    sqr->setVertexArray(sqr_v.get());

    ref_ptr<DrawElementsUInt>
    quad(new DrawElementsUInt(PrimitiveSet::QUADS, 0));
    quad->push_back(3);
    quad->push_back(2);
    quad->push_back(1);
    quad->push_back(0);

    sqr->addPrimitiveSet(quad.get());

    ref_ptr<Vec2Array> texcoords(new Vec2Array(4));
    float rep = 5;
    (*texcoords)[0].set(0.0f, 0.0f);
    (*texcoords)[1].set(0.0f, rep);
    (*texcoords)[2].set(rep, rep);
    (*texcoords)[3].set(rep, 0.0f);
    sqr->setTexCoordArray(0, texcoords.get());

    ref_ptr<Vec3Array> normals(new Vec3Array);
    normals->push_back(osg::Vec3(0, 0, 1));
    sqr->setNormalArray(normals.get());
    sqr->setNormalBinding(Geometry::BIND_OVERALL);

    return geode_sqr;
  }

  void OsgVisitor :: _create_ground(const ode::Environment &env)
  {
    static const float ground_x = 20;
    static const float ground_y = ground_x;
    static const float ground_z = 0;

    ref_ptr<PositionAttitudeTransform>  patg(new PositionAttitudeTransform());
    patg->setAttitude(osg::Quat(env.get_roll(), osg::Vec3(0, 1, 0),
                                env.get_pitch(), osg::Vec3(1, 0, 0),
				0, osg::Vec3(0, 0, 1)));
    patg->setPosition(osg::Vec3(0, 0, env.get_z()));

    _ground = new Group;
    patg->addChild(_ground);

    static const float nb = 1;
    for (size_t i = 0; i < nb; ++i)
      for (size_t j = 0; j < nb; ++j)
      {
        ref_ptr<Geode> geode_sqr = _create_sqr(ground_x, ground_y);
	geode_sqr->setNodeMask(ReceivesShadowTraversalMask);

        ref_ptr<PositionAttitudeTransform>  pat(new PositionAttitudeTransform());
        pat->setPosition(Vec3((i - nb / 2) * ground_x,
                              (j - nb / 2) * ground_y, 0));
        pat->addChild(geode_sqr.get());

        ref_ptr<StateSet> ss_checker(new StateSet());
        ss_checker->setTextureAttributeAndModes(0, _load_texture("data/checker.tga"));
        geode_sqr->setStateSet(ss_checker.get());
        _ground->addChild(pat.get());
      }

    Vec3 center(0.0f, 0.0f, 0.0f);
    float radius = 500;
    Vec3 light_position(center + Vec3(0.0f, 0.0f, 10));
  


    osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;
    shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
    
     //    osgShadow::MinimalDrawBoundsShadowMap* sm = new
     //    osgShadow::MinimalDrawBoundsShadowMap;
    // osg::ref_ptr<osgShadow::MinimalCullBoundsShadowMap> sm = new
    //  osgShadow::MinimalCullBoundsShadowMap;
    //osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
    

    // OK  osg::ref_ptr<osgShadow::ParallelSplitShadowMap> sm =
    // new osgShadow::ParallelSplitShadowMap;
    //osg::ref_ptr<osgShadow::LightSpacePerspectiveShadowMap> sm =
    // new osgShadow::LightSpacePerspectiveShadowMap;
    
    //    osg::ref_ptr<osgShadow::StandardShadowMap> sm =
    //  new osgShadow::StandardShadowMap;
    osg::ref_ptr<osgShadow::ShadowTexture> sm =
      new osgShadow::ShadowTexture;

    //             osg::ref_ptr<osgShadow::SoftShadowMap> sm = new osgShadow::SoftShadowMap;
    //sm->setTextureSize(osg::Vec2s(1024*2, 1024*2));
    // sm->setAmbientBias(osg::Vec2(0.5, 0.4));
     //      sm->setMaxFarDistance(20);
    // sm->setPolygonOffset(osg::Vec2(0, 0));
     //     sm->enableShadowGLSLFiltering(false);
    shadowedScene->setShadowTechnique(sm.get());
    LightSource*ls = new LightSource;
    ls->getLight()->setPosition(osg::Vec4(light_position, 1));
    ls->getLight()->setAmbient(Vec4(1, 1, 1, 1.0));
    ls->getLight()->setDiffuse(Vec4(0.9, 0.9, 0.9, 1.0));

    if (env.get_ground() != 0x0)
      _root->addChild(patg);
    shadowedScene->addChild(_root.get());
    shadowedScene->addChild(ls);

    ref_ptr<Material> matirial = new Material;
    matirial->setColorMode(Material::DIFFUSE);
    matirial->setAmbient(Material::FRONT_AND_BACK, Vec4(0.2, 0.2, 0.2, 1));
    matirial->setSpecular(Material::FRONT_AND_BACK, Vec4(0.2, 0.2, 0.2, 1));
    matirial->setShininess(Material::FRONT_AND_BACK, 64);
    _root->getOrCreateStateSet()->setAttributeAndModes(matirial.get(), StateAttribute::ON);

     // _shadowed_scene =
     //    create_shadowed_scene(_root.get(), ground.get(),
     //			  light_position, radius / 100.0f, 1);
    _shadowed_scene = shadowedScene;

    _viewer.setSceneData(_shadowed_scene.get());
    //_viewer.setSceneData(_root);//_shadowed_scene.get());
  }

  void OsgVisitor::visit(const std::vector<ode::Object::ptr_t>& v)
  {
    assert(v.size());//hack..
    if (!_viewer.getSceneData())
      _create_ground(v[0]->get_env());
    BOOST_FOREACH(ode::Object::ptr_t o, v)
      o->accept(*this);
  }
  void OsgVisitor::_update_traj()
  {
    osg::Vec3 pos = _node_ref->getBound().center() * osg::computeLocalToWorld(_node_ref->getParentalNodePaths()[0]);
    if ((pos - _prev_pos).length() > 0.02)
    {
      _prev_pos = pos;
      ref_ptr<Geode> geode(new Geode);
      osg::ref_ptr<osg::ShapeDrawable> sphere = 
	new ShapeDrawable(new Sphere(osg::Vec3(pos.x(), pos.y(), 0), 0.01));
      sphere->setColor(Vec4(1, 0, 0, 0.5)); 
      sphere->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
      geode->addDrawable(sphere.get());

      _shadowed_scene->addChild(geode.get());
    }
  }
  

  void OsgVisitor :: visit(const ode::CappedCyl& e)
  {
    if (_first)
    {
      ref_ptr<Geode> geode(new Geode);
      geode->
	addDrawable(new ShapeDrawable
                    (new Capsule(Vec3f(),
                                 e.get_radius(),
                                 e.get_length() - e.get_radius() * 2)));
      ref_ptr<PositionAttitudeTransform>  pat(new PositionAttitudeTransform());
      pat->addChild(geode.get());
      ref_ptr<NodeCallback> cb(new UpdateCallback(e));
      pat->setUpdateCallback(cb.get());
      _root->addChild(pat.get());
      //      geode->setNodeMask(CastsShadowTraversalMask);
      _set_tm(geode.get());
    }
  }

   // apply ode transformation position only
  class PositionCallback : public NodeCallback
  {
    public:
      PositionCallback(const ode::Object & obj) : _obj(obj){
      }

      virtual void operator() (Node * node, NodeVisitor * nv)
      {
        PositionAttitudeTransform *pat =
          dynamic_cast<PositionAttitudeTransform *>(node);
        pat->setPosition(_osg_pos(_obj));
        traverse(node, nv);
      }
    protected:
      const ode::Object& _obj;
  };
}
