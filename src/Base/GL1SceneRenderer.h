/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL1_SCENE_RENDERER_H
#define CNOID_BASE_GL1_SCENE_RENDERER_H

#include <cnoid/GLSceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class GL1SceneRendererImpl;
    
class CNOID_EXPORT GL1SceneRenderer : public GLSceneRenderer
{
public:
    GL1SceneRenderer();
    GL1SceneRenderer(SgGroup* root);
    virtual ~GL1SceneRenderer();

    virtual void setOutputStream(std::ostream& os) override;
    
    virtual NodeFunctionSet* renderingFunctions() override;
    virtual void renderCustomGroup(SgGroup* transform, std::function<void()> traverseFunction) override;
    virtual void renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction) override;

    virtual void renderNode(SgNode* node) override;

    virtual const Affine3& currentModelTransform() const override;
    virtual const Matrix4& projectionMatrix() const override;
        
    virtual bool initializeGL() override;
    virtual void flush() override;
    virtual const Vector3& pickedPoint() const override;
    virtual const SgNodePath& pickedNodePath() const override;

    virtual void setDefaultLighting(bool on) override;
    void setHeadLightLightingFromBackEnabled(bool on);
    virtual void setDefaultSmoothShading(bool on) override;
    virtual SgMaterial* defaultMaterial() override;
    virtual void enableTexture(bool on) override;
    virtual void setDefaultPointSize(double size) override;
    virtual void setDefaultLineWidth(double width) override;

    void setNewDisplayListDoubleRenderingEnabled(bool on);

    virtual void showNormalVectors(double length) override;

    virtual void requestToClearResources() override;

    /**
       If this is enabled, OpenGL resources such as display lists, vertex buffer objects
       are checked if they are still used or not, and the unused resources are released
       when finalizeRendering() is called. The default value is true.
    */
    void enableUnusedResourceCheck(bool on);

    virtual bool isPicking() const override;
    virtual void setColor(const Vector3f& color) override;
    void enableColorMaterial(bool on);
    void setDiffuseColor(const Vector4f& color);
    void setAmbientColor(const Vector4f& color);
    void setEmissionColor(const Vector4f& color);
    void setSpecularColor(const Vector4f& color);
    void setShininess(float shininess);
    void enableCullFace(bool on);
    void setFrontCCW(bool on);
    void enableLighting(bool on);
    void setLightModelTwoSide(bool on);
    void enableBlend(bool on);
    void enableDepthMask(bool on);
    void setPointSize(float size);
    void setLineWidth(float width);

  protected:
    virtual void doRender() override;
    virtual bool doPick(int x, int y) override;
    virtual void onImageUpdated(SgImage* image) override;
    
  private:
    GL1SceneRendererImpl* impl;
    friend class GL1SceneRendererImpl;
};

}

#endif
