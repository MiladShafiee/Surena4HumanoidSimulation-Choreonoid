/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_SCENE_RAIN_SNOW_H
#define CNOID_PHENOMENON_PLUGIN_SCENE_RAIN_SNOW_H

#include "SceneParticles.h"

namespace cnoid {

class SceneRainSnowBase : public SceneParticles
{
protected:
    SceneRainSnowBase(int polymorphicId);
    SceneRainSnowBase(const SceneRainSnowBase& org);

public:
    const Vector3f& velocity() const { return velocity_; }
    float radius() const { return radius_; }
    float top() const { return top_; }
    float bottom() const { return bottom_; }

    void setVelocity(const Vector3f& v) { velocity_ = v; }
    void setRadius(float r) { radius_ = r; }
    void setTop(float t) { top_ = t; }
    void setBottom(float b) { bottom_ = b; }

private:
    float radius_;
    float top_;
    float bottom_;
    Vector3f velocity_;
};


class SceneRain : public SceneRainSnowBase
{
public:
    SceneRain();
    SceneRain(const SceneRain& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const override;
};


class SceneSnow : public SceneRainSnowBase
{
public:
    SceneSnow();
    SceneSnow(const SceneSnow& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const override;
};

}

#endif
