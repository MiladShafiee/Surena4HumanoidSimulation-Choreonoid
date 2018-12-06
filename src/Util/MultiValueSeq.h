/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_VALUE_SEQ_H
#define CNOID_UTIL_MULTI_VALUE_SEQ_H

#include "MultiSeq.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiValueSeq : public MultiSeq<double>
{
    typedef MultiSeq<double> BaseSeqType;
            
public:
    typedef std::shared_ptr<MultiValueSeq> Ptr;
    typedef std::shared_ptr<const MultiValueSeq> ConstPtr;

    MultiValueSeq();
    MultiValueSeq(int numFrames, int numParts = 1);
    MultiValueSeq(const MultiValueSeq& org);
    virtual ~MultiValueSeq();

    using BaseSeqType::operator=;
    virtual AbstractSeqPtr cloneSeq() const;
        
    virtual bool loadPlainFormat(const std::string& filename);
    virtual bool saveAsPlainFormat(const std::string& filename);

protected:
    virtual bool doWriteSeq(YAMLWriter& writer);
    virtual bool doReadSeq(const Mapping& archive);
};

typedef MultiValueSeq::Ptr MultiValueSeqPtr;        
typedef MultiValueSeq::ConstPtr ConstMultiValueSeqPtr;

}

#endif
