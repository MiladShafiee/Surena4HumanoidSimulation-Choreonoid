/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_EXTRACTOR_H
#define CNOID_UTIL_MESH_EXTRACTOR_H

#include "EigenTypes.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class SgNode;
class SgMesh;
class MeshExtractorImpl;

class CNOID_EXPORT MeshExtractor
{
public:
    MeshExtractor();
    bool extract(SgNode* node, std::function<void()> callback);
    SgMesh* integrate(SgNode* node);

    SgMesh* currentMesh() const;
    const Affine3& currentTransform() const;
    const Affine3& currentTransformWithoutScaling() const;
    bool isCurrentScaled() const;

private:
    MeshExtractorImpl* impl;
};

}

#endif
