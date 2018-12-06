/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_AIST_COLLISION_DETECTOR_AIST_COLLISION_DETECTOR_H
#define CNOID_AIST_COLLISION_DETECTOR_AIST_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class AISTCollisionDetectorImpl;

class CNOID_EXPORT AISTCollisionDetector : public CollisionDetector
{
public:
    AISTCollisionDetector();
    virtual ~AISTCollisionDetector();
    virtual const char* name() const;
    virtual CollisionDetectorPtr clone() const;
    virtual void clearGeometries();
    virtual int numGeometries() const;
    virtual int addGeometry(SgNodePtr geometry);
    virtual void setGeometryStatic(int geometryId, bool isStatic = true);
    virtual bool enableGeometryCache(bool on);
    virtual void clearGeometryCache(SgNodePtr geometry);
    virtual void clearAllGeometryCaches();
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2);
    virtual bool makeReady();
    virtual void updatePosition(int geometryId, const Position& position);
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback);

    // experimental API
    int geometryPairId(int geometryId1, int geometryId2) const;
    double findClosestPoints(int geometryPairId, Vector3& out_point1, Vector3& out_point2);

    void setNumThreads(int n);

private:
    AISTCollisionDetectorImpl* impl;
};

typedef std::shared_ptr<AISTCollisionDetector> AISTCollisionDetectorPtr;

}

#endif
