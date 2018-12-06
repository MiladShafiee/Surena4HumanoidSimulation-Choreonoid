/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_TRACKING_CAMERA_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_TRACKING_CAMERA_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class BodyTrackingCameraItemImpl;
  
class CNOID_EXPORT BodyTrackingCameraItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyTrackingCameraItem();
    BodyTrackingCameraItem(const BodyTrackingCameraItem& org);

    virtual void setName(const std::string& name);
    virtual SgNode* getScene();

protected:
    virtual Item* doDuplicate() const;
    virtual void onPositionChanged();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    
private:
    BodyTrackingCameraItemImpl* impl;
};
  
typedef ref_ptr<BodyTrackingCameraItem> BodyTrackingCameraItemPtr;

}

#endif
