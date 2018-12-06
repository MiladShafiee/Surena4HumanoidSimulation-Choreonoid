/**
   @author Shin'ichiro NAKAOKA
*/

#include "KinematicFaultChecker.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "WorldItem.h"
#include "LinkSelectionView.h"
#include <cnoid/BodyState>
#include <cnoid/Archive>
#include <cnoid/MainWindow>
#include <cnoid/MenuManager>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/SpinBox>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/Dialog>
#include <cnoid/Separator>
#include <cnoid/EigenUtil>
#include <cnoid/BodyCollisionDetectorUtil>
#include <cnoid/IdPair>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QFrame>
#include <QLabel>
#include <map>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using boost::dynamic_bitset;
using boost::format;

namespace {

bool USE_DUPLICATED_BODY = false;

KinematicFaultChecker* checkerInstance = 0;

#if defined(_MSC_VER) && _MSC_VER < 1800
inline long lround(double x) {
    return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
}
#endif
}

namespace cnoid {

class KinematicFaultCheckerImpl : public Dialog
{
public:
    MessageView& mes;
    std::ostream& os;
        
    CheckBox positionCheck;
    DoubleSpinBox angleMarginSpin;
    DoubleSpinBox translationMarginSpin;
    CheckBox velocityCheck;

    QButtonGroup radioGroup;
    RadioButton allJointsRadio;
    RadioButton selectedJointsRadio;
    RadioButton nonSelectedJointsRadio;
        
    DoubleSpinBox velocityLimitRatioSpin;
    CheckBox collisionCheck;

    CheckBox onlyTimeBarRangeCheck;

    int numFaults;
    vector<int> lastPosFaultFrames;
    vector<int> lastVelFaultFrames;
    typedef std::map<IdPair<int>, int> LastCollisionFrameMap;
    LastCollisionFrameMap lastCollisionFrames;

    double frameRate;
    double angleMargin;
    double translationMargin;
    double velocityLimitRatio;

    KinematicFaultCheckerImpl();
    bool store(Archive& archive);
    void restore(const Archive& archive);
    void apply();
    int checkFaults(
        BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os,
        bool checkPosition, bool checkVelocity, bool checkCollision,
        dynamic_bitset<> linkSelection, double beginningTime, double endingTime);
    void putJointPositionFault(int frame, Link* joint, std::ostream& os);
    void putJointVelocityFault(int frame, Link* joint, std::ostream& os);
    void putSelfCollision(Body* body, int frame, const CollisionPair& collisionPair, std::ostream& os);
};
}


void KinematicFaultChecker::initialize(ExtensionManager* ext)
{
    if(!checkerInstance){
        checkerInstance = ext->manage(new KinematicFaultChecker());

        MenuManager& mm = ext->menuManager();
        mm.setPath("/Tools");
        mm.addItem(_("Kinematic Fault Checker"))
            ->sigTriggered().connect(std::bind(&KinematicFaultCheckerImpl::show, checkerInstance->impl));
        
        ext->setProjectArchiver(
            "KinematicFaultChecker",
            std::bind(&KinematicFaultCheckerImpl::store, checkerInstance->impl, _1),
            std::bind(&KinematicFaultCheckerImpl::restore, checkerInstance->impl, _1));
    }
}


KinematicFaultChecker* KinematicFaultChecker::instance()
{
    return checkerInstance;
}


KinematicFaultChecker::KinematicFaultChecker()
{
    impl = new KinematicFaultCheckerImpl();
}


KinematicFaultChecker::~KinematicFaultChecker()
{
    delete impl;
}


KinematicFaultCheckerImpl::KinematicFaultCheckerImpl()
    : mes(*MessageView::mainInstance()),
      os(mes.cout())
{
    setWindowTitle(_("Kinematic Fault Checker"));
    
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);
    
    QHBoxLayout* hbox = new QHBoxLayout();
    
    positionCheck.setText(_("Joint position check"));
    positionCheck.setChecked(true);
    hbox->addWidget(&positionCheck);
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Angle margin")));
    angleMarginSpin.setDecimals(2);
    angleMarginSpin.setRange(-99.99, 99.99);
    angleMarginSpin.setSingleStep(0.01);
    hbox->addWidget(&angleMarginSpin);
    hbox->addWidget(new QLabel("[deg]"));
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Translation margin")));
    translationMarginSpin.setDecimals(4);
    translationMarginSpin.setRange(-9.9999, 9.9999);
    translationMarginSpin.setSingleStep(0.0001);
    hbox->addWidget(&translationMarginSpin);
    hbox->addWidget(new QLabel("[m]"));

    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout();
    
    velocityCheck.setText(_("Joint velocity check"));
    velocityCheck.setChecked(true);
    hbox->addWidget(&velocityCheck);
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Limit ratio")));
    velocityLimitRatioSpin.setDecimals(0);
    velocityLimitRatioSpin.setRange(1.0, 100.0);
    velocityLimitRatioSpin.setValue(100.0);
    hbox->addWidget(&velocityLimitRatioSpin);
    hbox->addWidget(new QLabel("%"));

    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout();

    radioGroup.addButton(&allJointsRadio);
    radioGroup.addButton(&selectedJointsRadio);
    radioGroup.addButton(&nonSelectedJointsRadio);
    
    allJointsRadio.setText(_("All joints"));
    allJointsRadio.setChecked(true);
    hbox->addWidget(&allJointsRadio);

    selectedJointsRadio.setText(_("Selected joints"));
    hbox->addWidget(&selectedJointsRadio);

    nonSelectedJointsRadio.setText(_("Non-selected joints"));
    hbox->addWidget(&nonSelectedJointsRadio);

    hbox->addStretch();
    vbox->addLayout(hbox);
    vbox->addWidget(new HSeparator);
    hbox = new QHBoxLayout();
    
    collisionCheck.setText(_("Self-collision check"));
    collisionCheck.setChecked(true);
    hbox->addWidget(&collisionCheck);

    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);
    
    hbox = new QHBoxLayout();
    onlyTimeBarRangeCheck.setText(_("Time bar's range only"));
    onlyTimeBarRangeCheck.setChecked(false);
    hbox->addWidget(&onlyTimeBarRangeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);

    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect(std::bind(&KinematicFaultCheckerImpl::apply, this));
    
    vbox->addWidget(buttonBox);
}


bool KinematicFaultCheckerImpl::store(Archive& archive)
{
    archive.write("checkJointPositions", positionCheck.isChecked());
    archive.write("angleMargin", angleMarginSpin.value());
    archive.write("translationMargin", translationMarginSpin.value());
    archive.write("checkJointVelocities", velocityCheck.isChecked());
    archive.write("velocityLimitRatio", velocityLimitRatioSpin.value());
    archive.write("targetJoints",
                  (allJointsRadio.isChecked() ? "all" :
                   (selectedJointsRadio.isChecked() ? "selected" : "non-selected")));
    archive.write("checkSelfCollisions", collisionCheck.isChecked());
    archive.write("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked());
    return true;
}


void KinematicFaultCheckerImpl::restore(const Archive& archive)
{
    positionCheck.setChecked(archive.get("checkJointPositions", positionCheck.isChecked()));
    angleMarginSpin.setValue(archive.get("angleMargin", angleMarginSpin.value()));
    translationMarginSpin.setValue(archive.get("translationMargin", translationMarginSpin.value()));
    velocityCheck.setChecked(archive.get("checkJointVelocities", velocityCheck.isChecked()));
    velocityLimitRatioSpin.setValue(archive.get("velocityLimitRatio", velocityLimitRatioSpin.value()));
    string target;
    if(archive.read("targetJoints", target)){
        if(target == "all"){
            allJointsRadio.setChecked(true);
        } else if(target == "selected"){
            selectedJointsRadio.setChecked(true);
        } else if(target == "non-selected"){
            nonSelectedJointsRadio.setChecked(true);
        }
    }
    collisionCheck.setChecked(archive.get("checkSelfCollisions", collisionCheck.isChecked()));
    onlyTimeBarRangeCheck.setChecked(archive.get("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked()));
}


void KinematicFaultCheckerImpl::apply()
{
    ItemList<BodyMotionItem> items = ItemTreeView::mainInstance()->selectedItems<BodyMotionItem>();
    if(items.empty()){
        mes.notify(_("No BodyMotionItems are selected."));
    } else {
        for(size_t i=0; i < items.size(); ++i){
            BodyMotionItem* motionItem = items.get(i);
            BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>();
            if(!bodyItem){
                mes.notify(str(fmt(_("%1% is not owned by any BodyItem. Check skiped.")) % motionItem->name()));
            } else {
                mes.putln();
                mes.notify(str(fmt(_("Applying the Kinematic Fault Checker to %1% ..."))
                               % motionItem->headItem()->name()));
                
                dynamic_bitset<> linkSelection;
                if(selectedJointsRadio.isChecked()){
                    linkSelection = LinkSelectionView::mainInstance()->linkSelection(bodyItem);
                } else if(nonSelectedJointsRadio.isChecked()){
                    linkSelection = LinkSelectionView::mainInstance()->linkSelection(bodyItem);
                    linkSelection.flip();
                } else {
                    linkSelection.resize(bodyItem->body()->numLinks(), true);
                }
                
                double beginningTime = 0.0;
                double endingTime = motionItem->motion()->getTimeLength();
                std::numeric_limits<double>::max();
                if(onlyTimeBarRangeCheck.isChecked()){
                    TimeBar* timeBar = TimeBar::instance();
                    beginningTime = timeBar->minTime();
                    endingTime = timeBar->maxTime();
                }
                
                int n = checkFaults(bodyItem, motionItem, mes.cout(),
                                    positionCheck.isChecked(),
                                    velocityCheck.isChecked(),
                                    collisionCheck.isChecked(),
                                    linkSelection,
                                    beginningTime, endingTime);
                
                if(n > 0){
                    if(n == 1){
                        mes.notify(_("A fault has been detected."));
                    } else {
                        mes.notify(str(fmt(_("%1% faults have been detected.")) % n));
                    }
                } else {
                    mes.notify(_("No faults have been detected."));
                }
            }
        }
    }
}


/**
   @return Number of detected faults
*/
int KinematicFaultChecker::checkFaults
(BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os, double beginningTime, double endingTime)
{
    dynamic_bitset<> linkSelection(bodyItem->body()->numLinks());
    linkSelection.set();
    return impl->checkFaults(
        bodyItem, motionItem, os, true, true, true, linkSelection, beginningTime, endingTime);
}


int KinematicFaultCheckerImpl::checkFaults
(BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os,
 bool checkPosition, bool checkVelocity, bool checkCollision, dynamic_bitset<> linkSelection,
 double beginningTime, double endingTime)
{
    numFaults = 0;

    BodyPtr body = bodyItem->body();
    BodyMotionPtr motion = motionItem->motion();
    MultiValueSeqPtr qseq = motion->jointPosSeq();;
    MultiSE3SeqPtr pseq = motion->linkPosSeq();
    
    if((!checkPosition && !checkVelocity && !checkCollision) || body->isStaticModel() || !qseq->getNumFrames()){
        return numFaults;
    }

    BodyState orgKinematicState;
    
    if(USE_DUPLICATED_BODY){
        body = body->clone();
    } else {
        bodyItem->storeKinematicState(orgKinematicState);
    }

    CollisionDetectorPtr collisionDetector;
    WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();
    if(worldItem){
        collisionDetector = worldItem->collisionDetector()->clone();
    } else {
        int index = CollisionDetector::factoryIndex("AISTCollisionDetector");
        if(index >= 0){
            collisionDetector = CollisionDetector::create(index);
        } else {
            collisionDetector = CollisionDetector::create(0);
            os << _("A collision detector is not found. Collisions cannot be detected this time.") << endl;
        }
    }

    addBodyToCollisionDetector(*body, *collisionDetector);
    collisionDetector->makeReady();

    const int numJoints = std::min(body->numJoints(), qseq->numParts());
    const int numLinks = std::min(body->numLinks(), pseq->numParts());

    frameRate = motion->frameRate();
    double stepRatio2 = 2.0 / frameRate;
    angleMargin = radian(angleMarginSpin.value());
    translationMargin = translationMarginSpin.value();
    velocityLimitRatio = velocityLimitRatioSpin.value() / 100.0;

    int beginningFrame = std::max(0, (int)(beginningTime * frameRate));
    int endingFrame = std::min((motion->numFrames() - 1), (int)lround(endingTime * frameRate));

    lastPosFaultFrames.clear();
    lastPosFaultFrames.resize(numJoints, std::numeric_limits<int>::min());
    lastVelFaultFrames.clear();
    lastVelFaultFrames.resize(numJoints, std::numeric_limits<int>::min());
    lastCollisionFrames.clear();

    if(checkCollision){
        Link* root = body->rootLink();
        root->p().setZero();
        root->R().setIdentity();
    }
        
    for(int frame = beginningFrame; frame <= endingFrame; ++frame){

        int prevFrame = (frame == beginningFrame) ? beginningFrame : frame - 1;
        int nextFrame = (frame == endingFrame) ? endingFrame : frame + 1;

        for(int i=0; i < numJoints; ++i){
            Link* joint = body->joint(i);
            double q = qseq->at(frame, i);
            joint->q() = q;
            if(joint->index() >= 0 && linkSelection[joint->index()]){
                if(checkPosition){
                    bool fault = false;
                    if(joint->isRotationalJoint()){
                        fault = (q > (joint->q_upper() - angleMargin) || q < (joint->q_lower() + angleMargin));
                    } else if(joint->isSlideJoint()){
                        fault = (q > (joint->q_upper() - translationMargin) || q < (joint->q_lower() + translationMargin));
                    }
                    if(fault){
                        putJointPositionFault(frame, joint, os);
                    }
                }
                if(checkVelocity){
                    double dq = (qseq->at(nextFrame, i) - qseq->at(prevFrame, i)) / stepRatio2;
                    joint->dq() = dq;
                    if(dq > (joint->dq_upper() * velocityLimitRatio) || dq < (joint->dq_lower() * velocityLimitRatio)){
                        putJointVelocityFault(frame, joint, os);
                    }
                }
            }
        }

        if(checkCollision){

            Link* link = body->link(0);
            if(!pseq->empty())
            {
                const SE3& p = pseq->at(frame, 0);
                link->p() = p.translation();
                link->R() = p.rotation().toRotationMatrix();
            }
            else
            {
                link->p() = Vector3d(0., 0., 0.);
                link->R() = Matrix3d::Identity();
            }

            body->calcForwardKinematics();

            for(int i=1; i < numLinks; ++i){
                link = body->link(i);
                if(!pseq->empty())
                {
                    const SE3& p = pseq->at(frame, i);
                    link->p() = p.translation();
                    link->R() = p.rotation().toRotationMatrix();
                }
            }

            for(int i=0; i < numLinks; ++i){
                link = body->link(i);
                collisionDetector->updatePosition(i, link->position());
            }
            collisionDetector->detectCollisions(
                std::bind(&KinematicFaultCheckerImpl::putSelfCollision, this, body.get(), frame, _1, std::ref(os)));
        }
    }

    if(!USE_DUPLICATED_BODY){
        bodyItem->restoreKinematicState(orgKinematicState);
    }

    return numFaults;
}


void KinematicFaultCheckerImpl::putJointPositionFault(int frame, Link* joint, std::ostream& os)
{
    static format f1(fmt(_("%1$7.3f [s]: Position limit over of %2% (%3% is beyond the range (%4% , %5%) with margin %6%.)")));
    static format f2(fmt(_("%1$7.3f [s]: Position limit over of %2% (%3% is beyond the range (%4% , %5%).)")));
    
    if(frame > lastPosFaultFrames[joint->jointId()] + 1){
        double q, l, u, m;
        if(joint->isRotationalJoint()){
            q = degree(joint->q());
            l = degree(joint->q_lower());
            u = degree(joint->q_upper());
            m = degree(angleMargin);
        } else {
            q = joint->q();
            l = joint->q_lower();
            u = joint->q_upper();
            m = translationMargin;
        }

        if(m != 0.0){
            os << (f1 % (frame / frameRate) % joint->name() % q % l % u % m) << endl;
        } else {
            os << (f2 % (frame / frameRate) % joint->name() % q % l % u) << endl;
        }

        numFaults++;
    }
    lastPosFaultFrames[joint->jointId()] = frame;
}


void KinematicFaultCheckerImpl::putJointVelocityFault(int frame, Link* joint, std::ostream& os)
{
    static format f(fmt(_("%1$7.3f [s]: Velocity limit over of %2% (%3% is %4$.0f %% of the range (%5% , %6%).)")));
    
    if(frame > lastVelFaultFrames[joint->jointId()] + 1){
        double dq, l, u;
        if(joint->isRotationalJoint()){
            dq = degree(joint->dq());
            l = degree(joint->dq_lower());
            u = degree(joint->dq_upper());
        } else {
            dq = joint->dq();
            l = joint->dq_lower();
            u = joint->dq_upper();
        }

        double r = (dq < 0.0) ? (dq / l) : (dq / u);
        r *= 100.0;

        os << (f % (frame / frameRate) % joint->name() % dq % r % l % u) << endl;

        numFaults++;
    }
    lastVelFaultFrames[joint->jointId()] = frame;
}


void KinematicFaultCheckerImpl::putSelfCollision(Body* body, int frame, const CollisionPair& collisionPair, std::ostream& os)
{
    static format f(fmt(_("%1$7.3f [s]: Collision between %2% and %3%")));
    
    bool putMessage = false;
    IdPair<int> idPair(collisionPair.geometryId);
    LastCollisionFrameMap::iterator p = lastCollisionFrames.find(idPair);
    if(p == lastCollisionFrames.end()){
        putMessage = true;
        lastCollisionFrames[idPair] = frame;
    } else {
        if(frame > p->second + 1){
            putMessage = true;
        }
        p->second = frame;
    }

    if(putMessage){
        Link* link0 = body->link(collisionPair.geometryId[0]);
        Link* link1 = body->link(collisionPair.geometryId[1]);
        os << (f % (frame / frameRate) % link0->name() % link1->name()) << endl;
        numFaults++;
    }
}
