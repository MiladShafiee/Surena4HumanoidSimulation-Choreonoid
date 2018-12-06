/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_ZMP_SEQ_H
#define CNOID_BODY_ZMP_SEQ_H

#include <cnoid/Vector3Seq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ZMPSeq : public Vector3Seq
{
public:
    static const std::string& key();

    ZMPSeq(int nFrames = 0);
    ZMPSeq(const ZMPSeq& org);
    ZMPSeq(const Vector3Seq& org);

    virtual AbstractSeq& operator=(const AbstractSeq& rhs) override;
    ZMPSeq& operator=(const ZMPSeq& rhs);
    virtual AbstractSeqPtr cloneSeq() const override;

    bool isRootRelative() const { return isRootRelative_; }
    void setRootRelative(bool on);

protected:
    virtual bool doWriteSeq(YAMLWriter& writer) override;
    virtual bool doReadSeq(const Mapping& archive) override;

private:
    bool isRootRelative_;
};

typedef std::shared_ptr<ZMPSeq> ZMPSeqPtr;
        
class BodyMotion;

CNOID_EXPORT ZMPSeqPtr getZMPSeq(const BodyMotion& motion);
CNOID_EXPORT ZMPSeqPtr getOrCreateZMPSeq(BodyMotion& motion);
CNOID_EXPORT void clearZMPSeq(BodyMotion& motion);
CNOID_EXPORT bool makeRootRelative(ZMPSeq& zmpseq, BodyMotion& motion, bool on);

}

#endif
