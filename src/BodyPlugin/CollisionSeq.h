/**
   @file
   @author Shizuko Hattori
*/

#ifndef CNOID_BODY_COLLISION_SEQ_H
#define CNOID_BODY_COLLISION_SEQ_H

#include <cnoid/CollisionLinkPair>
#include <cnoid/MultiSeq>
#include <cnoid/YAMLWriter>
#include "exportdecl.h"

namespace cnoid {

class YAMLWriter;
class CollisionSeqItem;

typedef std::vector<CollisionLinkPairPtr> CollisionLinkPairList;
typedef std::shared_ptr<CollisionLinkPairList> CollisionLinkPairListPtr;

class CNOID_EXPORT CollisionSeq : public MultiSeq<CollisionLinkPairListPtr>
{
    typedef MultiSeq<CollisionLinkPairListPtr> BaseSeqType;

public:
    CollisionSeqItem* collisionSeqItem_;
    CollisionSeq(CollisionSeqItem* collisionSeqItem);

    using BaseSeqType::operator=;

    virtual bool doWriteSeq(YAMLWriter& writer);
    virtual bool doReadSeq(const Mapping& archive);

    bool loadStandardYAMLformat(const std::string& filename);
    bool saveAsStandardYAMLformat(const std::string& filename);
    void writeCollsionData(YAMLWriter& writer, const CollisionLinkPairListPtr ptr);
    void readCollisionData(int nFrames, const Listing& values);

};

typedef std::shared_ptr<CollisionSeq> CollisionSeqPtr;

}

#endif
