/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/ProjectManager>
#include <cnoid/ItemManager>
#include <QLibrary>
#include <boost/format.hpp>
#include <boost/dynamic_bitset.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;
namespace filesystem = boost::filesystem;

namespace {

enum {
    INPUT_JOINT_DISPLACEMENT = 0,
    INPUT_JOINT_FORCE = 1,
    INPUT_JOINT_VELOCITY = 2,
    INPUT_JOINT_ACCELERATION = 3,
    INPUT_LINK_POSITION = 4,
    INPUT_NONE = 5
};


struct SharedInfo : public Referenced
{
    BodyPtr ioBody;
    ScopedConnectionSet inputDeviceStateConnections;
    boost::dynamic_bitset<> inputEnabledDeviceFlag;
    boost::dynamic_bitset<> inputDeviceStateChangeFlag;
};

typedef ref_ptr<SharedInfo> SharedInfoPtr;

}

namespace cnoid {

class SimpleControllerItemImpl : public SimulationSimpleControllerIO
{
public:
    SimpleControllerItem* self;
    SimpleController* controller;
    Body* simulationBody;
    Body* ioBody;
    ControllerIO* io;
    SharedInfoPtr sharedInfo;

    vector<unsigned short> inputLinkIndices;
    vector<char> inputStateTypes;

    vector<bool> outputLinkFlags;
    vector<unsigned short> outputLinkIndices;

    ConnectionSet outputDeviceStateConnections;
    boost::dynamic_bitset<> outputDeviceStateChangeFlag;

    vector<SimpleControllerItemPtr> childControllerItems;

    vector<char> linkIndexToInputStateTypeMap;
        
    MessageView* mv;

    std::string controllerModuleName;
    std::string controllerModuleFilename;
    filesystem::path controllerDirectory;
    QLibrary controllerModule;
    bool doReloading;
    Selection baseDirectoryType;

    enum BaseDirectoryType {
        NO_BASE_DIRECTORY = 0,
        CONTROLLER_DIRECTORY,
        PROJECT_DIRECTORY,
        N_BASE_DIRECTORY_TYPES
    };

    Signal<void()> sigControllerChanged;

    SimpleControllerItemImpl(SimpleControllerItem* self);
    SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org);
    ~SimpleControllerItemImpl();
    void setController(const std::string& name);
    void unloadController(bool doNotify);
    void initializeIoBody();
    void updateInputEnabledDevices();
    SimpleController* initialize(ControllerIO* io, SharedInfo* info);
    void updateIOStateTypes();
    void input();
    void onInputDeviceStateChanged(int deviceIndex);
    void onOutputDeviceStateChanged(int deviceIndex);
    void output();
    bool onReloadingChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);

    // virtual functions of ControllerIO
    virtual std::string optionString() const override;
    virtual Body* body() override;
    virtual std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual bool isNoDelayMode() const;
    virtual bool setNoDelayMode(bool on);

    // virtual functions of SimpleControllerIO
    virtual void enableIO(Link* link) override;
    virtual void enableInput(Link* link) override;
    virtual void enableInput(Link* link, int stateTypes) override;
    virtual void enableInput(Device* device) override;
    virtual void enableOutput(Link* link) override;

    // deprecated virtual functions
    virtual void setLinkInput(Link* link, int stateTypes) override;
    virtual void setJointInput(int stateTypes) override;
    virtual void setLinkOutput(Link* link, int stateTypes) override;
    virtual void setJointOutput(int stateTypes) override;
    virtual bool isImmediateMode() const override;
    virtual void setImmediateMode(bool on) override;
};

}


void SimpleControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<SimpleControllerItem>(N_("SimpleControllerItem"));
    itemManager.addCreationPanel<SimpleControllerItem>();
}


SimpleControllerItem::SimpleControllerItem()
{
    setName("SimpleController");
    impl = new SimpleControllerItemImpl(this);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self)
    : self(self),
      baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    controller = 0;
    io = 0;
    mv = MessageView::instance();
    doReloading = true;

    controllerDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "simplecontroller";

    baseDirectoryType.setSymbol(NO_BASE_DIRECTORY, N_("None"));
    baseDirectoryType.setSymbol(CONTROLLER_DIRECTORY, N_("Controller directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(CONTROLLER_DIRECTORY);
}


SimpleControllerItem::SimpleControllerItem(const SimpleControllerItem& org)
    : ControllerItem(org)
{
    impl = new SimpleControllerItemImpl(this, *org.impl);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org)
    : self(self),
      controllerModuleName(org.controllerModuleName),
      controllerDirectory(org.controllerDirectory),
      baseDirectoryType(org.baseDirectoryType)
{
    controller = 0;
    io = 0;
    mv = MessageView::instance();
    doReloading = org.doReloading;
}


SimpleControllerItem::~SimpleControllerItem()
{
    delete impl;
}


SimpleControllerItemImpl::~SimpleControllerItemImpl()
{
    unloadController(false);
    outputDeviceStateConnections.disconnect();
}


void SimpleControllerItem::onDisconnectedFromRoot()
{
    if(!isActive()){
        impl->unloadController(false);
    }
    impl->childControllerItems.clear();
}


Item* SimpleControllerItem::doDuplicate() const
{
    return new SimpleControllerItem(*this);
}


void SimpleControllerItem::setController(const std::string& name)
{
    impl->setController(name);
}


void SimpleControllerItemImpl::setController(const std::string& name)
{
    unloadController(true);

    filesystem::path modulePath(name);
    if(modulePath.is_absolute()){
        baseDirectoryType.select(NO_BASE_DIRECTORY);
        if(modulePath.parent_path() == controllerDirectory){
            baseDirectoryType.select(CONTROLLER_DIRECTORY);
            modulePath = modulePath.filename();
        } else {
            filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());
            if(!projectDir.empty() && (modulePath.parent_path() == projectDir)){
                baseDirectoryType.select(PROJECT_DIRECTORY);
                modulePath = modulePath.filename();
            }
        }
    }
    controllerModuleName = modulePath.string();
    controllerModuleFilename.clear();
}


SimpleController* SimpleControllerItem::controller()
{
    return impl->controller;
}


void SimpleControllerItemImpl::unloadController(bool doNotify)
{
    if(controller){
        delete controller;
        controller = 0;

        if(doNotify){
            sigControllerChanged();
        }
    }

    if(controllerModule.unload()){
        mv->putln(fmt(_("The controller module \"%2%\" of %1% has been unloaded."))
                  % self->name() % controllerModuleFilename);
    }
}


SignalProxy<void()> SimpleControllerItem::sigControllerChanged()
{
    return impl->sigControllerChanged;
}


void SimpleControllerItemImpl::initializeIoBody()
{
    ioBody = simulationBody->clone();

    outputDeviceStateConnections.disconnect();
    const DeviceList<>& ioDevices = ioBody->devices();
    outputDeviceStateChangeFlag.resize(ioDevices.size());
    outputDeviceStateChangeFlag.reset();
    for(size_t i=0; i < ioDevices.size(); ++i){
        outputDeviceStateConnections.add(
            ioDevices[i]->sigStateChanged().connect(
                [this, i](){ onOutputDeviceStateChanged(i); }));
    }

    sharedInfo->inputEnabledDeviceFlag.resize(simulationBody->numDevices());
    sharedInfo->inputEnabledDeviceFlag.reset();

    sharedInfo->ioBody = ioBody;
}


void SimpleControllerItemImpl::updateInputEnabledDevices()
{
    const DeviceList<>& devices = simulationBody->devices();
    sharedInfo->inputDeviceStateChangeFlag.resize(devices.size());
    sharedInfo->inputDeviceStateChangeFlag.reset();
    sharedInfo->inputDeviceStateConnections.disconnect();

    const boost::dynamic_bitset<>& flag = sharedInfo->inputEnabledDeviceFlag;
    for(size_t i=0; i < devices.size(); ++i){
        if(flag[i]){
            sharedInfo->inputDeviceStateConnections.add(
                devices[i]->sigStateChanged().connect(
                    [this, i](){ onInputDeviceStateChanged(i); }));
        } else {
            sharedInfo->inputDeviceStateConnections.add(Connection()); // null connection
        }
    }
}


bool SimpleControllerItem::initialize(ControllerIO* io)
{
    if(impl->initialize(io, new SharedInfo)){
        impl->updateInputEnabledDevices();
        return true;
    }
    return false;
}


SimpleController* SimpleControllerItemImpl::initialize(ControllerIO* io, SharedInfo* info)
{
    this->io = io;
    simulationBody = io->body();
    sharedInfo = info;
    
    bool result = false;

    if(!controller){

        filesystem::path modulePath(controllerModuleName);
        if(!modulePath.is_absolute()){
            if(baseDirectoryType.is(CONTROLLER_DIRECTORY)){
                modulePath = controllerDirectory / modulePath;
            } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
                string projectDir = ProjectManager::instance()->currentProjectDirectory();
                if(!projectDir.empty()){
                    modulePath = filesystem::path(projectDir) / modulePath;
                } else {
                    mv->putln(MessageView::ERROR,
                              format(_("Controller module \"%1%\" of %2% is specified as a relative "
                                       "path from the project directory, but the project directory "
                                       "has not been determined yet."))
                              % controllerModuleName % self->name());
                    return 0;
                }
            }
        }
        controllerModuleFilename = modulePath.make_preferred().string();
        controllerModule.setFileName(controllerModuleFilename.c_str());
        
        if(controllerModule.isLoaded()){
            mv->putln(fmt(_("The controller module of %1% has already been loaded.")) % self->name());
            
            // This should be called to make the reference to the DLL.
            // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
            controllerModule.load();
            
        } else {
            mv->put(fmt(_("Loading the controller module \"%2%\" of %1% ... "))
                    % self->name() % controllerModuleFilename);
            if(!controllerModule.load()){
                mv->put(_("Failed.\n"));
                mv->putln(MessageView::ERROR, controllerModule.errorString());
            } else {                
                mv->putln(_("OK!"));
            }
        }
        
        if(controllerModule.isLoaded()){
            SimpleController::Factory factory =
                (SimpleController::Factory)controllerModule.resolve("createSimpleController");
            if(!factory){
                mv->putln(MessageView::ERROR,
                          _("The factory function \"createSimpleController()\" is not found in the controller module."));
            } else {
                controller = factory();
                if(!controller){
                    mv->putln(MessageView::ERROR, _("The factory failed to create a controller instance."));
                } else {
                    mv->putln(_("A controller instance has successfully been created."));
                    sigControllerChanged();
                }
            }
        }
    }

    childControllerItems.clear();

    if(controller){
        if(!sharedInfo->ioBody){
            initializeIoBody();
        } else {
            ioBody = sharedInfo->ioBody;
        }
        
        inputLinkIndices.clear();
        inputStateTypes.clear();
        outputLinkFlags.clear();
        
        result = controller->initialize(this);

        if(!result){
            mv->putln(MessageView::ERROR, fmt(_("%1%'s initialize method failed.")) % self->name());
            if(doReloading){
                self->stop();
            }
        } else {
            for(Item* child = self->childItem(); child; child = child->nextItem()){
                SimpleControllerItem* childControllerItem = dynamic_cast<SimpleControllerItem*>(child);
                if(childControllerItem){
                    SimpleController* childController = childControllerItem->impl->initialize(io, sharedInfo);
                    if(childController){
                        childControllerItems.push_back(childControllerItem);
                    }
                }
            }
            updateIOStateTypes();
        }
    }

    return controller;
}


static int getInputStateTypeIndex(int type)
{
    switch(type){
    case SimpleControllerIO::JOINT_DISPLACEMENT: return INPUT_JOINT_DISPLACEMENT;
    case SimpleControllerIO::JOINT_VELOCITY:     return INPUT_JOINT_VELOCITY;
    case SimpleControllerIO::JOINT_ACCELERATION: return INPUT_JOINT_ACCELERATION;
    case SimpleControllerIO::JOINT_FORCE:        return INPUT_JOINT_FORCE;
    case SimpleControllerIO::LINK_POSITION:      return INPUT_LINK_POSITION;
    default:
        return INPUT_NONE;
    }
}


void SimpleControllerItemImpl::updateIOStateTypes()
{
    // Input
    inputLinkIndices.clear();
    inputStateTypes.clear();
    for(size_t i=0; i < linkIndexToInputStateTypeMap.size(); ++i){
        bitset<5> types(linkIndexToInputStateTypeMap[i]);
        if(types.any()){
            const int n = types.count();
            inputLinkIndices.push_back(i);
            inputStateTypes.push_back(n);
            for(int j=0; j < 5; ++j){
                if(types.test(j)){
                    inputStateTypes.push_back(getInputStateTypeIndex(1 << j));
                }
            }
        }
    }

    // Output
    outputLinkIndices.clear();
    for(size_t i=0; i < outputLinkFlags.size(); ++i){
        if(outputLinkFlags[i]){
            outputLinkIndices.push_back(i);
            simulationBody->link(i)->setActuationMode(ioBody->link(i)->actuationMode());
        }
    }
}


std::string SimpleControllerItemImpl::optionString() const
{
    if(io){
        const std::string& opt1 = io->optionString();
        const std::string& opt2 = self->optionString();
        if(!opt1.empty()){
            if(opt2.empty()){
                return opt1;
            } else {
                return opt1 + " " + opt2;
            }
        }
    }
    return self->optionString();
}


Body* SimpleControllerItemImpl::body()
{
    return ioBody;
}


double SimpleControllerItem::timeStep() const
{
    return impl->io ? impl->io->timeStep() : 0.0;
}


double SimpleControllerItemImpl::timeStep() const
{
    return io->timeStep();
}


double SimpleControllerItemImpl::currentTime() const
{
    return io->currentTime();
}


std::ostream& SimpleControllerItemImpl::os() const
{
    return mv->cout();
}


void SimpleControllerItemImpl::enableIO(Link* link)
{
    enableInput(link);
    enableOutput(link);
}
        

void SimpleControllerItemImpl::enableInput(Link* link)
{
    int defaultInputStateTypes = 0;

    switch(link->actuationMode()){

    case Link::JOINT_EFFORT:
    case Link::JOINT_SURFACE_VELOCITY:
        defaultInputStateTypes = SimpleControllerIO::JOINT_ANGLE;
        break;

    case Link::JOINT_DISPLACEMENT:
    case Link::JOINT_VELOCITY:
        defaultInputStateTypes = SimpleControllerIO::JOINT_ANGLE | SimpleControllerIO::JOINT_TORQUE;
        break;

    case Link::LINK_POSITION:
        defaultInputStateTypes = SimpleControllerIO::LINK_POSITION;
        break;

    default:
        break;
    }

    enableInput(link, defaultInputStateTypes);
}        


void SimpleControllerItemImpl::enableInput(Link* link, int stateTypes)
{
    if(link->index() >= static_cast<int>(linkIndexToInputStateTypeMap.size())){
        linkIndexToInputStateTypeMap.resize(link->index() + 1, 0);
    }
    linkIndexToInputStateTypeMap[link->index()] |= stateTypes;
}        


void SimpleControllerItemImpl::setLinkInput(Link* link, int stateTypes)
{
    enableInput(link, stateTypes);
}


void SimpleControllerItemImpl::setJointInput(int stateTypes)
{
    for(Link* joint : ioBody->joints()){
        setLinkInput(joint, stateTypes);
    }
}


void SimpleControllerItemImpl::enableOutput(Link* link)
{
    int index = link->index();
    if(static_cast<int>(outputLinkFlags.size()) <= index){
        outputLinkFlags.resize(index + 1, false);
    }
    outputLinkFlags[index] = true;
}


void SimpleControllerItemImpl::setLinkOutput(Link* link, int stateTypes)
{
    Link::ActuationMode mode = Link::NO_ACTUATION;

    if(stateTypes & SimpleControllerIO::LINK_POSITION){
        mode = Link::LINK_POSITION;
    } else if(stateTypes & SimpleControllerIO::JOINT_DISPLACEMENT){
        mode = Link::JOINT_DISPLACEMENT;
    } else if(stateTypes & SimpleControllerIO::JOINT_VELOCITY){
        mode = Link::JOINT_VELOCITY;
    } else if(stateTypes & SimpleControllerIO::JOINT_EFFORT){
        mode = Link::JOINT_EFFORT;
    }

    if(mode != Link::NO_ACTUATION){
        link->setActuationMode(mode);
        enableOutput(link);
    }
}


void SimpleControllerItemImpl::setJointOutput(int stateTypes)
{
    const int nj = ioBody->numJoints();
    for(int i=0; i < nj; ++i){
        setLinkOutput(ioBody->joint(i), stateTypes);
    }
}
    

void SimpleControllerItemImpl::enableInput(Device* device)
{
    sharedInfo->inputEnabledDeviceFlag.set(device->index());
}


bool SimpleControllerItemImpl::isNoDelayMode() const
{
    return self->isNoDelayMode();
}


bool SimpleControllerItemImpl::isImmediateMode() const
{
    return isNoDelayMode();
}


bool SimpleControllerItemImpl::setNoDelayMode(bool on)
{
    self->setNoDelayMode(on);
    return on;
}


void SimpleControllerItemImpl::setImmediateMode(bool on)
{
    setNoDelayMode(on);
}


bool SimpleControllerItem::start()
{
    if(impl->controller->start()){
        for(size_t i=0; i < impl->childControllerItems.size(); ++i){
            if(!impl->childControllerItems[i]->start()){
                return false;
            }
        }
    }
    return true;
}


void SimpleControllerItem::input()
{
    impl->input();
    
    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->impl->input();
    }
}


void SimpleControllerItemImpl::input()
{
    int typeArrayIndex = 0;
    for(size_t i=0; i < inputLinkIndices.size(); ++i){
        const int linkIndex = inputLinkIndices[i];
        const Link* simLink = simulationBody->link(linkIndex);
        Link* ioLink = ioBody->link(linkIndex);
        const int n = inputStateTypes[typeArrayIndex++];
        for(int j=0; j < n; ++j){
            switch(inputStateTypes[typeArrayIndex++]){
            case INPUT_JOINT_DISPLACEMENT:
                ioLink->q() = simLink->q();
                break;
            case INPUT_JOINT_FORCE:
                ioLink->u() = simLink->u();
                break;
            case INPUT_LINK_POSITION:
                ioLink->T() = simLink->T();
                break;
            case INPUT_JOINT_VELOCITY:
                ioLink->dq() = simLink->dq();
                break;
            case INPUT_JOINT_ACCELERATION:
                ioLink->ddq() = simLink->ddq();
                break;
            default:
                break;
            }
        }
    }

    boost::dynamic_bitset<>& inputDeviceStateChangeFlag = sharedInfo->inputDeviceStateChangeFlag;
    if(inputDeviceStateChangeFlag.any()){
        const DeviceList<>& devices = simulationBody->devices();
        const DeviceList<>& ioDevices = ioBody->devices();
        boost::dynamic_bitset<>::size_type i = inputDeviceStateChangeFlag.find_first();
        while(i != inputDeviceStateChangeFlag.npos){
            Device* ioDevice = ioDevices[i];
            ioDevice->copyStateFrom(*devices[i]);
            outputDeviceStateConnections.block(i);
            ioDevice->notifyStateChange();
            outputDeviceStateConnections.unblock(i);
            i = inputDeviceStateChangeFlag.find_next(i);
        }
        inputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItemImpl::onInputDeviceStateChanged(int deviceIndex)
{
    sharedInfo->inputDeviceStateChangeFlag.set(deviceIndex);
}


bool SimpleControllerItem::control()
{
    bool result = impl->controller->control();

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        if(impl->childControllerItems[i]->impl->controller->control()){
            result = true;
        }
    }
        
    return result;
}


void SimpleControllerItemImpl::onOutputDeviceStateChanged(int deviceIndex)
{
    outputDeviceStateChangeFlag.set(deviceIndex);
}


void SimpleControllerItem::output()
{
    impl->output();
    
    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->impl->output();
    }
}


void SimpleControllerItemImpl::output()
{
    for(size_t i=0; i < outputLinkIndices.size(); ++i){
        const int index = outputLinkIndices[i];
        const Link* ioLink = ioBody->link(index);
        Link* simLink = simulationBody->link(index);
        switch(ioLink->actuationMode()){
        case Link::JOINT_EFFORT:
            simLink->u() = ioLink->u();
            break;
        case Link::JOINT_DISPLACEMENT:
            simLink->q() = ioLink->q();
            break;
        case Link::JOINT_VELOCITY:
        case Link::JOINT_SURFACE_VELOCITY:
            simLink->dq() = ioLink->dq();
            simLink->dq() = ioLink->dq();
            break;
        case Link::LINK_POSITION:
            simLink->T() = ioLink->T();
            break;
        default:
            break;
        }
    }
        
    if(outputDeviceStateChangeFlag.any()){
        const DeviceList<>& devices = simulationBody->devices();
        const DeviceList<>& ioDevices = ioBody->devices();
        boost::dynamic_bitset<>::size_type i = outputDeviceStateChangeFlag.find_first();
        while(i != outputDeviceStateChangeFlag.npos){
            Device* device = devices[i];
            device->copyStateFrom(*ioDevices[i]);
            sharedInfo->inputDeviceStateConnections.block(i);
            device->notifyStateChange();
            sharedInfo->inputDeviceStateConnections.unblock(i);
            i = outputDeviceStateChangeFlag.find_next(i);
        }
        outputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItem::stop()
{
    if(impl->doReloading || !findRootItem()){
        impl->unloadController(true);
    }

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->stop();
    }
    impl->childControllerItems.clear();

    impl->io = 0;
}


bool SimpleControllerItemImpl::onReloadingChanged(bool on)
{
    doReloading = on;
    return true;
}


void SimpleControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void SimpleControllerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty moduleProperty(
        controllerModuleName,
        { str(format(_("Simple Controller Module (*.%1%)")) % DLL_EXTENSION) });

    if(baseDirectoryType.is(CONTROLLER_DIRECTORY)){
        moduleProperty.setBaseDirectory(controllerDirectory.string());
    } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
    }

    putProperty(_("Controller module"), moduleProperty,
                [&](const string& name){ setController(name); return true; });
    putProperty(_("Base directory"), baseDirectoryType, changeProperty(baseDirectoryType));

    putProperty(_("Reloading"), doReloading, [&](bool on){ return onReloadingChanged(on); });
}


bool SimpleControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    return impl->store(archive);
}


bool SimpleControllerItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("controller", controllerModuleName);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("reloading", doReloading);
    return true;
}


bool SimpleControllerItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    return impl->restore(archive);
}


bool SimpleControllerItemImpl::restore(const Archive& archive)
{
    string value;
    if(archive.read("controller", value)){
        controllerModuleName = archive.expandPathVariables(value);
    }
    
    baseDirectoryType.select(CONTROLLER_DIRECTORY);
    if(archive.read("baseDirectory", value) ||
       archive.read("RelativePathBase", value) /* for the backward compatibility */){
        baseDirectoryType.select(value);
    }

    archive.read("reloading", doReloading);

    return true;
}
