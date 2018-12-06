/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_JOINT_GRAPH_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_JOINT_GRAPH_VIEW_H_INCLUDED

#include "BodyItem.h"
#include "LinkSelectionView.h"
#include <cnoid/MultiValueSeqItem>
#include <cnoid/View>
#include <cnoid/GraphWidget>
#include <cnoid/ItemList>
#include <cnoid/ConnectionSet>

namespace cnoid {

class Archive;

class JointGraphView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    JointGraphView();
    ~JointGraphView();
            
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
protected:

    virtual QWidget* indicatorOnInfoBar();
            
private:
            
    GraphWidget graph;
    LinkSelectionView* linkSelection;

    struct ItemInfo
    {
        ~ItemInfo(){
            connections.disconnect();
        }
        MultiValueSeqItemPtr item;
        MultiValueSeqPtr seq;
        BodyItemPtr bodyItem;
        ConnectionSet connections;
        std::vector<GraphDataHandlerPtr> handlers;
    };

    std::list<ItemInfo> itemInfos;

    std::set<BodyItemPtr> bodyItems;
    ConnectionSet bodyItemConnections;
    Connection itemTreeViewConnection;

    void onItemSelectionChanged(const ItemList<MultiValueSeqItem>& items);
    void onDataItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter);
    void updateBodyItems();
    void onBodyItemDetachedFromRoot(BodyItemPtr bodyItem);
    void setupGraphWidget();
    void addJointTrajectory(std::list<ItemInfo>::iterator itemInfoIter, Link* joint, MultiValueSeqPtr seq);
    void onDataItemUpdated(std::list<ItemInfo>::iterator itemInfoIter);
    void onDataRequest(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* out_values);
    void onDataModified(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* values);

};
}

#endif
