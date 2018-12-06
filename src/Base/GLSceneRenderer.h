/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL_SCENE_RENDERER_H
#define CNOID_BASE_GL_SCENE_RENDERER_H

#include <cnoid/SceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class GLSceneRendererImpl;
    
class CNOID_EXPORT GLSceneRenderer : public SceneRenderer
{
public:
    GLSceneRenderer();
    GLSceneRenderer(SgGroup* root);
    virtual ~GLSceneRenderer();

    virtual void setOutputStream(std::ostream& os) = 0;

    virtual SgGroup* sceneRoot() override;
    virtual SgGroup* scene() override;

    virtual bool initializeGL();
    virtual void flush() = 0;

    // The following functions cannot be called bofore calling the initializeGL() function.
    bool setSwapInterval(int interval);
    int getSwapInterval() const;

    virtual void setViewport(int x, int y, int width, int height) override;
    virtual Array4i viewport() const override;
    void getViewport(int& out_x, int& out_y, int& out_width, int& out_height) const;
    virtual double aspectRatio() const override; // width / height;

    void getPerspectiveProjectionMatrix(
        double fovy, double aspect, double zNear, double zFar, Matrix4& out_matrix);
    void getOrthographicProjectionMatrix(
        double left,  double right,  double bottom,  double top,  double nearVal,  double farVal, Matrix4& out_matrix);
    
    void getViewFrustum(const SgPerspectiveCamera* camera, double& left, double& right, double& bottom, double& top) const;
    void getViewVolume(const SgOrthographicCamera* camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const;
    
    bool unproject(double x, double y, double z, Vector3& out_projected) const;    

    const Vector3f& backgroundColor() const;
    void setBackgroundColor(const Vector3f& color);

    const Vector3f& defaultColor() const;
    void setDefaultColor(const Vector3f& color);

    virtual void setDefaultLighting(bool on) = 0;
    virtual void clearShadows();
    virtual void enableShadowOfLight(int index, bool on = true);
    virtual void enableShadowAntiAliasing(bool on);
    virtual void setDefaultSmoothShading(bool on) = 0;
    virtual SgMaterial* defaultMaterial() = 0;
    virtual void enableTexture(bool on) = 0;
    virtual void setDefaultPointSize(double size) = 0;
    virtual void setDefaultLineWidth(double width) = 0;

    enum PolygonMode { FILL_MODE, LINE_MODE, POINT_MODE };
    void setPolygonMode(PolygonMode mode);
    PolygonMode polygonMode() const;

    virtual void showNormalVectors(double length) = 0;

    virtual void requestToClearResources() = 0;
    virtual void enableUnusedResourceCheck(bool on) = 0;
    
    virtual const Vector3& pickedPoint() const = 0;
    virtual const SgNodePath& pickedNodePath() const = 0;
    virtual bool isPicking() const = 0;

    virtual void setColor(const Vector3f& color) = 0;

    virtual void setUpsideDown(bool on);

protected:
    virtual void onSceneGraphUpdated(const SgUpdate& update) override;
    virtual void onImageUpdated(SgImage* image) = 0;

private:
    GLSceneRendererImpl* impl;
    friend class GLSceneRendererImpl;
};

}

#endif
