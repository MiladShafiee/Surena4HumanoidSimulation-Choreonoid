/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyIoRTCItem.h"
#include <cnoid/BodyIoRTC>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace cnoid {

class BodyIoRTCItemImpl : public ControllerIO
{
public:
    BodyIoRTCItem* self;
    BodyItem* bodyItem = 0;
    BodyIoRTC* bodyIoRTC = 0;
    MessageView* mv;
    
    BodyIoRTCItemImpl(BodyIoRTCItem* self);
    BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org);
    void setBodyItem(BodyItem* newBodyItem, bool forceReset);
    bool createBodyIoRTC();

    // Virtual functions of ControllerIO
    virtual Body* body();
    virtual std::string optionString() const;
    virtual std::ostream& os() const;
};

}


void BodyIoRTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<BodyIoRTCItem>(N_("BodyIoRTCItem"));
        ext->itemManager().addCreationPanel<BodyIoRTCItem>();
        initialized = true;
    }
}


BodyIoRTCItem::BodyIoRTCItem()
{
    impl = new BodyIoRTCItemImpl(this);
    useOnlyChoreonoidExecutionContext();
}


BodyIoRTCItemImpl::BodyIoRTCItemImpl(BodyIoRTCItem* self)
    : self(self),
      mv(MessageView::instance())
{

}


BodyIoRTCItem::BodyIoRTCItem(const BodyIoRTCItem& org)
    : ControllerRTCItem(org)
{
    impl = new BodyIoRTCItemImpl(this, *org.impl);
    useOnlyChoreonoidExecutionContext();
}


BodyIoRTCItemImpl::BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org)
    : BodyIoRTCItemImpl(self)
{

}


BodyIoRTCItem::~BodyIoRTCItem()
{
    delete impl;
}


Item* BodyIoRTCItem::doDuplicate() const
{
    return new BodyIoRTCItem(*this);
}


void BodyIoRTCItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyIoRTCItemImpl::setBodyItem(BodyItem* newBodyItem, bool forceReset)
{
    if(newBodyItem != bodyItem || forceReset){
        bodyItem = newBodyItem;
        self->createRTC();
    }
}


std::string BodyIoRTCItem::getDefaultRTCInstanceName() const
{
    return impl->bodyItem->name() + "-" + rtcModuleName();
}


Body* BodyIoRTCItemImpl::body()
{
    if(bodyItem){
        return bodyItem->body();
    }
    return nullptr;
}


std::string BodyIoRTCItemImpl::optionString() const
{
    return self->optionString();
}


std::ostream& BodyIoRTCItemImpl::os() const
{
    return mv->cout();
}


bool BodyIoRTCItem::createRTC()
{
    return impl->createBodyIoRTC();
}


bool BodyIoRTCItemImpl::createBodyIoRTC()
{
    if(!bodyItem){
        self->deleteRTC(true);
        return false;
    }
    
    if(self->createRTCmain()){

        bodyIoRTC = dynamic_cast<BodyIoRTC*>(self->rtc());
        if(!bodyIoRTC){
            mv->putln(MessageView::ERROR,
                      format(_("RTC \"%1%\" of %2% cannot be used as a BodyIoRTC because it is not derived from it."))
                      % self->rtcModuleName() % self->name());
            self->deleteRTC(false);
            return false;
        }
        bool initialized = false;
        if(bodyIoRTC->onInitialize(bodyItem->body()) == RTC::RTC_OK){ // old API
            initialized = true;
        } else {
            initialized = bodyIoRTC->initializeIO(this);
        }

        if(!initialized){
            mv->putln(MessageView::ERROR,
                      format(_("RTC \"%1%\" of %2% failed to initialize."))
                      % self->rtcModuleName() % self->name());
            self->deleteRTC(true);
            return false;
        }
        
        mv->putln(fmt(_("BodyIoRTC \"%1%\" has been created.")) % self->rtcInstanceName());
        
        return true;
    }

    return false;
}


void BodyIoRTCItem::deleteRTC(bool waitToBeDeleted)
{
    ControllerRTCItem::deleteRTC(waitToBeDeleted);
    impl->bodyIoRTC = 0;
}


bool BodyIoRTCItem::initialize(ControllerIO* io)
{
    if(impl->bodyIoRTC){
        return impl->bodyIoRTC->initializeSimulation(io);
    }
    return false;
}


bool BodyIoRTCItem::start()
{
    if(impl->bodyIoRTC){
        if(impl->bodyIoRTC->startSimulation()){
            if(ControllerRTCItem::start()){
                impl->bodyIoRTC->inputFromSimulator();
                return true;
            }
        }
    }
    return false;
}


void BodyIoRTCItem::input()
{
    impl->bodyIoRTC->inputFromSimulator();
}


void BodyIoRTCItem::output()
{
    impl->bodyIoRTC->outputToSimulator();
}


void BodyIoRTCItem::stop()
{
    impl->bodyIoRTC->stopSimulation();
    ControllerRTCItem::stop();
}
