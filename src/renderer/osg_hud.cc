#include "osg_hud.hh"

using namespace osg;

namespace renderer 
{  
  OsgHud :: OsgHud(Group* groupe)
  {

    Geode *HUDGeode = new Geode();
    // Text instance for HUD:
    _text = new osgText::Text();
    // Projection node for defining view frustrum for HUD:
    Projection* HUDProjectionMatrix(new Projection);

    HUDProjectionMatrix->setMatrix(Matrix::ortho2D(0,1024,0,768));

    // For the HUD model view matrix use an identity matrix:
    MatrixTransform* HUDModelViewMatrix = new MatrixTransform;
    HUDModelViewMatrix->setMatrix(Matrix::identity());

    // Make sure the model view matrix is not affected by any transforms
    // above it in the scene graph:
    HUDModelViewMatrix->setReferenceFrame(Transform::ABSOLUTE_RF);

    // Add the HUD projection matrix as a child of the root node
    // and the HUD model view matrix as a child of the projection matrix
    // Anything under this node will be view using this projection matrix
    // and positioned with this model view matrix.
    groupe->addChild(HUDProjectionMatrix);
    HUDProjectionMatrix->addChild(HUDModelViewMatrix);

    // Add the Geometry node to contain HUD geometry as a child of the
    // HUD model view matrix.
    // (See figure "n")
    HUDModelViewMatrix->addChild(HUDGeode);

    // Add the text (Text class is derived from drawable) to the geode:
    HUDGeode->addDrawable(_text);

    // Set up the parameters for the text we'll add to the HUD:
    _text->setCharacterSize(15);
    
    _text->setAxisAlignment(osgText::Text::SCREEN);
    _text->setPosition(Vec3(10,10,-1.5) );
    _text->setColor(Vec4(199, 77, 15, 1) );
    _text->setFont("fonts/arial.ttf");

    // Set up geometry for the HUD and add it to the HUD
    // geometry node, see tutorial "n" for details:
    Geometry* HUDBackgroundGeometry = new Geometry();

    Vec3Array* HUDBackgroundVertices = new Vec3Array;

    HUDBackgroundVertices->push_back(Vec3(0, 0,-1) );
    HUDBackgroundVertices->push_back(Vec3(1024, 0,-1) );
    HUDBackgroundVertices->push_back(Vec3(1024,30,-1) );
    HUDBackgroundVertices->push_back(Vec3(0,30,-1) );

    DrawElementsUInt* HUDBackgroundIndices =
      new DrawElementsUInt(PrimitiveSet::POLYGON, 0);
    HUDBackgroundIndices->push_back(0);
    HUDBackgroundIndices->push_back(1);
    HUDBackgroundIndices->push_back(2);
    HUDBackgroundIndices->push_back(3);

    Vec4Array* HUDcolors = new Vec4Array;
    HUDcolors->push_back(Vec4(0.8f,0.8f,0.8f,0.8f));

    Vec2Array* texcoords = new Vec2Array(4);
    (*texcoords)[0].set(0.0f,0.0f);
    (*texcoords)[1].set(1.0f,0.0f);
    (*texcoords)[2].set(1.0f,1.0f);
    (*texcoords)[3].set(0.0f,1.0f);
    HUDBackgroundGeometry->setTexCoordArray(0,texcoords);
    Texture2D* HUDTexture = new Texture2D;
    HUDTexture->setDataVariance(Object::DYNAMIC);
    Image* hudImage;
    hudImage = osgDB::readImageFile("HUD.png");
    HUDTexture->setImage(hudImage);
    Vec3Array* HUDnormals = new Vec3Array;
    HUDnormals->push_back(Vec3(0.0f,0.0f,1.0f));
    HUDBackgroundGeometry->setNormalArray(HUDnormals);
    HUDBackgroundGeometry->setNormalBinding(Geometry::BIND_OVERALL);

    HUDBackgroundGeometry->addPrimitiveSet(HUDBackgroundIndices);
    HUDBackgroundGeometry->setVertexArray(HUDBackgroundVertices);
    HUDBackgroundGeometry->setColorArray(HUDcolors);
    HUDBackgroundGeometry->setColorBinding(Geometry::BIND_OVERALL);

    HUDGeode->addDrawable(HUDBackgroundGeometry);

    // Create and set up a state set using the texture from above:
    StateSet* HUDStateSet = new StateSet();
    HUDGeode->setStateSet(HUDStateSet);
    HUDStateSet->
      setTextureAttributeAndModes(0,HUDTexture,StateAttribute::ON);

    // For this state set, turn blending on (so alpha texture looks right)
    HUDStateSet->setMode(GL_BLEND,StateAttribute::ON);

    // Disable depth testing so geometry is draw regardless of depth values
    // of geometry already draw.
    HUDStateSet->setMode(GL_DEPTH_TEST,StateAttribute::OFF);
    HUDStateSet->setRenderingHint(StateSet::TRANSPARENT_BIN );

    // Need to make sure this geometry is draw last. RenderBins are handled
    // in numerical order so set bin number to 11
    HUDStateSet->setRenderBinDetails(99, "RenderBin");

  }

}
