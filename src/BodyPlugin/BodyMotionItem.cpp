/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionItem.h"
#include "BodyItem.h"
#include <cnoid/MultiSeqItemCreationPanel>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/ZMPSeq>
#include <cnoid/LazyCaller>
#include <cnoid/MessageView>
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using boost::format;

namespace {

typedef std::function<AbstractSeqItem*(AbstractSeqPtr seq)> ExtraSeqItemFactory;
typedef map<string, ExtraSeqItemFactory> ExtraSeqItemFactoryMap;
ExtraSeqItemFactoryMap extraSeqItemFactories;

struct ExtraSeqItemInfo : public Referenced
{
    string key;
    AbstractSeqItemPtr item;
    Connection sigUpdateConnection;

    ExtraSeqItemInfo(const string& key, AbstractSeqItemPtr& item) : key(key), item(item) { }
    ~ExtraSeqItemInfo() {
        sigUpdateConnection.disconnect();
        item->detachFromParentItem();
    }
};

typedef ref_ptr<ExtraSeqItemInfo> ExtraSeqItemInfoPtr;
    
typedef std::map<std::string, ExtraSeqItemInfoPtr> ExtraSeqItemInfoMap;
}


namespace cnoid {

class BodyMotionItemImpl
{
public:
    BodyMotionItem* self;
        
    Connection jointPosSeqUpdateConnection;
    Connection linkPosSeqUpdateConnection;

    ExtraSeqItemInfoMap extraSeqItemInfoMap;
    vector<ExtraSeqItemInfoPtr> extraSeqItemInfos;
    Signal<void()> sigExtraSeqItemsChanged;
    Connection extraSeqsChangedConnection;

    BodyMotionItemImpl(BodyMotionItem* self);
    ~BodyMotionItemImpl();
    void initialize();
    void onSubItemUpdated();
    void onExtraSeqItemSetChanged();
    void updateExtraSeqItems();
};
}


static bool fileIoSub(BodyMotionItem* item, std::ostream& os, bool loaded, bool isLoading)
{
    if(!loaded){
        os << item->motion()->seqMessage();
    }
    return loaded;
}
                

static bool loadStandardYamlFormat(BodyMotionItem* item, const std::string& filename, std::ostream& os)
{
    return fileIoSub(item, os, item->motion()->loadStandardYAMLformat(filename), true);
}
    

static bool saveAsStandardYamlFormat(BodyMotionItem* item, const std::string& filename, std::ostream& os)
{
    return fileIoSub(item, os, item->motion()->saveAsStandardYAMLformat(filename), false);
}

static bool bodyMotionItemPreFilter(BodyMotionItem* protoItem, Item* parentItem)
{
    BodyItemPtr bodyItem = dynamic_cast<BodyItem*>(parentItem);
    if(!bodyItem){
        bodyItem = parentItem->findOwnerItem<BodyItem>();
    }
    if(bodyItem){
        MultiValueSeqPtr jointPosSeq = protoItem->motion()->jointPosSeq();
        int prevNumJoints = jointPosSeq->numParts();
        int numJoints = bodyItem->body()->numJoints();
        if(numJoints != prevNumJoints){
            jointPosSeq->setNumParts(numJoints, true);
        }
    }
    return true;
}


/*
  static bool bodyMotionItemPostFilter(BodyMotionItem* protoItem, Item* parentItem)
  {
  BodyItemPtr bodyItem = dynamic_cast<BodyItem*>(parentItem);
  if(!bodyItem){
  bodyItem = parentItem->findOwnerItem<BodyItem>();
  }
  if(bodyItem){
  BodyPtr body = bodyItem->body();
  MultiValueSeqPtr qseq = protoItem->jointPosSeq();
  int n = std::min(body->numJoints(), qseq->numParts());
  for(int i=0; i < n; ++i){
  Link* joint = body->joint(i);
  if(joint->defaultJointValue != 0.0){
  MultiValueSeq::Part part = qseq->part(i);
  std::fill(part.begin(), part.end(), joint->defaultJointValue);
  }
  }
  }
    
  return true;
  }
*/


void BodyMotionItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    
    if(initialized){
        return;
    }
    
    ItemManager& im = ext->itemManager();
    
    im.registerClass<BodyMotionItem>(N_("BodyMotionItem"));

    im.addCreationPanel<BodyMotionItem>(new MultiSeqItemCreationPanel(_("Number of joints")));
    im.addCreationPanelPreFilter<BodyMotionItem>(bodyMotionItemPreFilter);
    //im.addCreationPanelPostFilter<BodyMotionItem>(bodyMotionItemPostFilter);

    im.addLoaderAndSaver<BodyMotionItem>(
        _("Body Motion"), "BODY-MOTION-YAML", "yaml",
        std::bind(loadStandardYamlFormat, _1, _2, _3),  std::bind(saveAsStandardYamlFormat, _1, _2, _3));

    initialized = true;
}


void BodyMotionItem::addExtraSeqItemFactory
(const std::string& key, std::function<AbstractSeqItem*(AbstractSeqPtr seq)> factory)
{
    extraSeqItemFactories[key] = factory;
}


BodyMotionItem::BodyMotionItem()
    : bodyMotion_(new BodyMotion())
{
    impl = new BodyMotionItemImpl(this);
}


BodyMotionItem::BodyMotionItem(BodyMotionPtr bodyMotion)
    : bodyMotion_(bodyMotion)
{
    impl = new BodyMotionItemImpl(this);
}


BodyMotionItem::BodyMotionItem(const BodyMotionItem& org)
    : AbstractMultiSeqItem(org),
      bodyMotion_(new BodyMotion(*org.bodyMotion_))
{
    impl = new BodyMotionItemImpl(this);
}


BodyMotionItemImpl::BodyMotionItemImpl(BodyMotionItem* self)
    : self(self)
{
    initialize();
}


void BodyMotionItemImpl::initialize()
{
    self->jointPosSeqItem_ = new MultiValueSeqItem(self->bodyMotion_->jointPosSeq());
    self->jointPosSeqItem_->setName("Joint");
    self->addSubItem(self->jointPosSeqItem_);

    jointPosSeqUpdateConnection =
        self->jointPosSeqItem_->sigUpdated().connect(
            std::bind(&BodyMotionItemImpl::onSubItemUpdated, this));

    self->linkPosSeqItem_ = new MultiSE3SeqItem(self->bodyMotion_->linkPosSeq());
    self->linkPosSeqItem_->setName("Cartesian");
    self->addSubItem(self->linkPosSeqItem_);

    linkPosSeqUpdateConnection = 
        self->linkPosSeqItem_->sigUpdated().connect(
            std::bind(&BodyMotionItemImpl::onSubItemUpdated, this));

    extraSeqsChangedConnection =
        self->bodyMotion_->sigExtraSeqsChanged().connect(
            std::bind(&BodyMotionItemImpl::onExtraSeqItemSetChanged, this));

    updateExtraSeqItems();
}


BodyMotionItem::~BodyMotionItem()
{
    delete impl;
}


BodyMotionItemImpl::~BodyMotionItemImpl()
{
    extraSeqsChangedConnection.disconnect();
    jointPosSeqUpdateConnection.disconnect();
    linkPosSeqUpdateConnection.disconnect();
}


AbstractMultiSeqPtr BodyMotionItem::abstractMultiSeq()
{
    return bodyMotion_;
}


void BodyMotionItem::notifyUpdate()
{
    impl->jointPosSeqUpdateConnection.block();
    jointPosSeqItem_->notifyUpdate();
    impl->jointPosSeqUpdateConnection.unblock();

    impl->linkPosSeqUpdateConnection.block();
    linkPosSeqItem_->notifyUpdate();
    impl->linkPosSeqUpdateConnection.unblock();

    vector<ExtraSeqItemInfoPtr>& extraSeqItemInfos = impl->extraSeqItemInfos;
    for(size_t i=0; i < extraSeqItemInfos.size(); ++i){
        ExtraSeqItemInfo* info = extraSeqItemInfos[i];
        info->sigUpdateConnection.block();
        info->item->notifyUpdate();
        info->sigUpdateConnection.unblock();
    }

    Item::notifyUpdate();
}


void BodyMotionItemImpl::onSubItemUpdated()
{
    self->suggestFileUpdate();
    self->Item::notifyUpdate();
}


int BodyMotionItem::numExtraSeqItems() const
{
    return impl->extraSeqItemInfos.size();
}


const std::string& BodyMotionItem::extraSeqKey(int index) const
{
    return impl->extraSeqItemInfos[index]->key;
}


AbstractSeqItem* BodyMotionItem::extraSeqItem(int index)
{
    return impl->extraSeqItemInfos[index]->item;
}


const AbstractSeqItem* BodyMotionItem::extraSeqItem(int index) const
{
    return impl->extraSeqItemInfos[index]->item;
}


SignalProxy<void()> BodyMotionItem::sigExtraSeqItemsChanged()
{
    return impl->sigExtraSeqItemsChanged;
}


void BodyMotionItemImpl::onExtraSeqItemSetChanged()
{
    callLater(std::bind(&BodyMotionItemImpl::updateExtraSeqItems, this));
}


void BodyMotionItem::updateExtraSeqItems()
{
    impl->updateExtraSeqItems();
}


void BodyMotionItemImpl::updateExtraSeqItems()
{
    extraSeqItemInfos.clear();

    BodyMotion& bodyMotion = *self->bodyMotion_;
    BodyMotion::ConstSeqIterator p;
    for(p = bodyMotion.extraSeqBegin(); p != bodyMotion.extraSeqEnd(); ++p){
        const string& key = p->first;
        const AbstractSeqPtr& newSeq = p->second;
        AbstractSeqItemPtr newItem;
        ExtraSeqItemInfoMap::iterator p = extraSeqItemInfoMap.find(key);
        if(p != extraSeqItemInfoMap.end()){
            ExtraSeqItemInfo* info = p->second;
            AbstractSeqItemPtr& prevItem = info->item;
            if(typeid(prevItem->abstractSeq()) == typeid(newSeq)){
                extraSeqItemInfos.push_back(info);
                newItem = prevItem;
            }
        }
        if(!newItem){
            ExtraSeqItemFactoryMap::iterator q = extraSeqItemFactories.find(key);
            if(q != extraSeqItemFactories.end()){
                ExtraSeqItemFactory& factory = q->second;
                newItem = factory(newSeq);
                if(newItem){
                    self->addSubItem(newItem);
                    ExtraSeqItemInfo* info = new ExtraSeqItemInfo(key, newItem);
                    info->sigUpdateConnection =
                        newItem->sigUpdated().connect(
                            std::bind(&BodyMotionItemImpl::onSubItemUpdated, this));
                    extraSeqItemInfos.push_back(info);
                }
            }
        }
    }
    extraSeqItemInfoMap.clear();
    for(size_t i=0; i < extraSeqItemInfos.size(); ++i){
        ExtraSeqItemInfo* info = extraSeqItemInfos[i];
        extraSeqItemInfoMap.insert(make_pair(info->key, info));
    }

    sigExtraSeqItemsChanged();
}


bool BodyMotionItem::onChildItemAboutToBeAdded(Item* childItem_, bool isManualOperation)
{
    if(isManualOperation){
        AbstractSeqItem* seqItem = dynamic_cast<AbstractSeqItem*>(childItem_);
        if(seqItem){
            if(!dynamic_cast<BodyMotionItem*>(seqItem)){
                bool existingFound = false;
                for(Item* item = childItem(); item; item = item->nextItem()){
                    if(item->isSubItem() && item->name() == seqItem->name()){
                        if(AbstractSeqItem* orgSeqItem = dynamic_cast<AbstractSeqItem*>(item)){
                            existingFound = true;
                            if(showConfirmDialog(
                                   _("Confirm"),
                                   str(format(_("Do you want to replace the data of %1%?")) % item->name()))){
                                *orgSeqItem->abstractSeq() = *seqItem->abstractSeq();
                                return false;
                            }
                        }
                    }
                }
                if(!existingFound){
                    if(showConfirmDialog(
                           _("Confirm"),
                           str(format(_("Do you want to set %1% as a sequence data of %2%?"))
                               % childItem_->name() % this->name()))){
                        motion()->setExtraSeq(seqItem->abstractSeq());
                        return false;
                    }
                }
            }
        }
    }
    return true;
}


Item* BodyMotionItem::doDuplicate() const
{
    return new BodyMotionItem(*this);
}


bool BodyMotionItem::store(Archive& archive)
{
    if(overwrite() || !filePath().empty()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        return true;
    }
    return false;
}


bool BodyMotionItem::restore(const Archive& archive)
{
    std::string filename, format;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", format)){
        if(load(filename, format)){
            return true;
        }
    }
    return false;
}
