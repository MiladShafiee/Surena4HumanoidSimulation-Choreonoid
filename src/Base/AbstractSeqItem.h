/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ABSTRACT_SEQ_ITEM_H
#define CNOID_BASE_ABSTRACT_SEQ_ITEM_H

#include "Item.h"
#include <cnoid/AbstractSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractSeqItem : public Item
{
public:
    AbstractSeqItem();
    AbstractSeqItem(const AbstractSeqItem& org);
    virtual ~AbstractSeqItem();

    virtual AbstractSeqPtr abstractSeq() = 0;

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
};

typedef ref_ptr<AbstractSeqItem> AbstractSeqItemPtr;


class CNOID_EXPORT AbstractMultiSeqItem : public AbstractSeqItem
{
public:
    AbstractMultiSeqItem();
    AbstractMultiSeqItem(const AbstractMultiSeqItem& org);
    virtual ~AbstractMultiSeqItem();

    virtual AbstractSeqPtr abstractSeq();
    virtual AbstractMultiSeqPtr abstractMultiSeq() = 0;

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty);
};

typedef ref_ptr<AbstractMultiSeqItem> AbstractMultiSeqItemPtr;

}

#endif
