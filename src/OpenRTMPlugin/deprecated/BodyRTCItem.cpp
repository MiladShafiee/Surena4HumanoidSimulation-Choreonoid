/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "BodyRTCItem.h"
#include "VirtualRobotRTC.h"
#include "../OpenRTMUtil.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/BasicSensors>
#include <cnoid/ControllerIO>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/MessageView>
#include <cnoid/Sleep>
#include <cnoid/ProjectManager>
#include <rtm/CorbaNaming.h>
#include <boost/regex.hpp>
#include "../gettext.h"

using namespace std;
using namespace cnoid;
using namespace RTC;
using boost::format;
namespace filesystem = boost::filesystem;

namespace {
const bool TRACE_FUNCTIONS = false;
}

void BodyRTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<BodyRTCItem>(N_("BodyRTCItem"));
        ext->itemManager().addCreationPanel<BodyRTCItem>();
        initialized = true;
    }
}


BodyRTCItem::BodyRTCItem()
    : os(MessageView::instance()->cout()),
      configMode(N_CONFIG_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    setName("BodyRTC");
    
    io = 0;
    virtualRobotRTC = 0;
    rtcomp = 0;
    bridgeConf = 0;
    moduleName.clear();
    bodyName.clear();
    instanceName.clear();
    mv = MessageView::instance();

    configMode.setSymbol(CONF_FILE_MODE,  N_("Use Configuration File"));
    configMode.setSymbol(CONF_ALL_MODE,  N_("Create Default Port"));
    configMode.select(CONF_ALL_MODE);
    autoConnect = false;

    baseDirectoryType.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(RTC_DIRECTORY);
    rtcDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc";

    executionCycleProperty = 0.0;
}


BodyRTCItem::BodyRTCItem(const BodyRTCItem& org)
    : ControllerItem(org),
      os(MessageView::instance()->cout()),
      configMode(org.configMode),
      baseDirectoryType(org.baseDirectoryType)
{
    io = 0;
    virtualRobotRTC = org.virtualRobotRTC;
    rtcomp = org.rtcomp;
    bridgeConf = org.bridgeConf;
    moduleName = org.moduleName;
    instanceName = org.instanceName;
    bodyName = org.bodyName;
    autoConnect = org.autoConnect;
    mv = MessageView::instance();
    executionCycleProperty = org.executionCycleProperty;
    rtcDirectory = org.rtcDirectory;
}


BodyRTCItem::~BodyRTCItem()
{
    
}


void BodyRTCItem::createRTC(BodyPtr body)
{
    bridgeConf = new BridgeConf();

    filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());

    if(configMode.is(CONF_ALL_MODE)){
        setdefaultPort(body);
    } else if(configMode.is(CONF_FILE_MODE)){
        filesystem::path confPath;
        if(!confFileName.empty()){
            confPath = confFileName;
        } else if(!moduleName.empty()){
            confPath = moduleName+".conf";
        }

        if(!confPath.empty()){
            if(!confPath.is_absolute()){
                if(baseDirectoryType.is(RTC_DIRECTORY)){
                    confPath = rtcDirectory / confPath;
                } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
                    if(projectDir.empty()){
                        mv->putln(_("Please save the project."));
                        return;
                    } else {
                        confPath = projectDir / confPath;
                   }
                }
            }
            std::string confFileName0 = getNativePathString(confPath);
            if(bridgeConf->loadConfigFile(confFileName0.c_str())){
                mv->putln(fmt(_("Config File \"%1%\" has been loaded.")) % confFileName0);
            } else {
                mv->putln(fmt(_("Cannot find or open \"%1%\".")) % confFileName0);
            }
        }
    }

    ModuleInfoList& moduleInfoList = bridgeConf->moduleInfoList;
    ModuleInfoList::iterator it;
    for(it=moduleInfoList.begin(); it != moduleInfoList.end(); ++it){
        mv->putln(fmt(_("Loading Module....  \"%1%\"")) % it->fileName);
    }
    bridgeConf->setupModules();

    if(!moduleName.empty()){
        ModuleInfoList& moduleInfoList = bridgeConf->moduleInfoList;
        ModuleInfoList::iterator it;
        for(it = moduleInfoList.begin(); it != moduleInfoList.end(); ++it){
            if(it->componentName == moduleName){
                break;
            }
        }
        if(it == moduleInfoList.end()){
            PropertyMap prop;
            prop["exec_cxt.periodic.type"] = "ChoreonoidExecutionContext";
            prop["exec_cxt.periodic.rate"] = "1000000";

            filesystem::path modulePath(moduleName);
            if(!modulePath.is_absolute()){
                if(baseDirectoryType.is(RTC_DIRECTORY)){
                    modulePath = rtcDirectory / modulePath;
                } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
                    if(projectDir.empty()){
                        mv->putln(_("Please save the project."));
                        return;
                    } else {
                        modulePath = projectDir / modulePath;
                    }
                }
            }
            rtcomp = new RTComponent(modulePath, prop);
        }
    }

    RTC::Manager& rtcManager = RTC::Manager::instance();
    string bodyName(body->name());
    if(instanceName.empty()){
        instanceName = bodyName;
    }
    int i=2;
    while(rtcManager.getComponent(instanceName.c_str()) != NULL){
        stringstream ss;
        ss << bodyName << "(" << i << ")";
        instanceName = ss.str();
        i++;
    }
    format param("VirtualRobot?instance_name=%1%&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000");
    RtcBase* rtc = createManagedRTC(str(param % instanceName).c_str());
    mv->putln(fmt(_("RTC \"%1%\" has been created.")) % instanceName);
    virtualRobotRTC = dynamic_cast<VirtualRobotRTC*>(rtc);
    virtualRobotRTC->createPorts(bridgeConf);

    virtualRobotEC = OpenRTM::ExtTrigExecutionContextService::_nil();
    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            virtualRobotEC = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            break;
        }
    }
    
}

void BodyRTCItem::setdefaultPort(BodyPtr body)
{
    PortInfoMap& outPortInfoMap = bridgeConf->outPortInfos;
    PortInfo portInfo;
    portInfo.dataOwnerNames.clear();
    portInfo.dataTypeId = JOINT_VALUE;
    portInfo.portName = "q";
    portInfo.stepTime = 0;
    outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
    portInfo.dataTypeId = JOINT_TORQUE;
    portInfo.portName = "u_out";
    portInfo.stepTime = 0;
    outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));

    for(size_t i=0; i < forceSensors_.size(); ++i){
        if(Device* sensor = forceSensors_[i]){
            portInfo.dataTypeId = FORCE_SENSOR;
            portInfo.dataOwnerNames.clear();
            portInfo.dataOwnerNames.push_back(sensor->name());
            portInfo.portName = sensor->name();
            portInfo.stepTime = 0;
            outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
        }
    }
    for(size_t i=0; i < gyroSensors_.size(); ++i){
        if(Device* sensor = gyroSensors_[i]){
            portInfo.dataTypeId = RATE_GYRO_SENSOR;
            portInfo.dataOwnerNames.clear();
            portInfo.dataOwnerNames.push_back(sensor->name());
            portInfo.portName = sensor->name();
            portInfo.stepTime = 0;
            outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
        }
    }
    for(size_t i=0; i < accelSensors_.size(); ++i){
        if(Device* sensor = accelSensors_[i]){
            portInfo.dataTypeId = ACCELERATION_SENSOR;
            portInfo.dataOwnerNames.clear();
            portInfo.dataOwnerNames.push_back(sensor->name());
            portInfo.portName = sensor->name();
            portInfo.stepTime = 0;
            outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
        }
    }

    PortInfoMap& inPortInfoMap = bridgeConf->inPortInfos;
    portInfo.dataOwnerNames.clear();
    portInfo.dataTypeId = JOINT_TORQUE;
    portInfo.portName = "u_in";
    portInfo.stepTime = 0;
    inPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
}

void BodyRTCItem::onPositionChanged()
{
    // create or recreate an RTC corresponding to the body
    // The target body can be detected like this:

    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
        Body* body = ownerBodyItem->body();
        if(bodyName != body->name()){
            forceSensors_ = body->devices<ForceSensor>().getSortedById();
            gyroSensors_ = body->devices<RateGyroSensor>().getSortedById();
            accelSensors_ = body->devices<AccelerationSensor>().getSortedById();
            bodyName = body->name();
            deleteModule(true);
            createRTC(body);
        }
    } else {
        deleteModule(false);
        bodyName.clear();
    }
}


void BodyRTCItem::onDisconnectedFromRoot()
{
    // This is not necessary because onPositionChanged() is also called
    // when the item is disconnected from the root
    deleteModule(false);
}


Item* BodyRTCItem::doDuplicate() const
{
    return new BodyRTCItem(*this);
}


bool BodyRTCItem::initialize(ControllerIO* io)
{
    this->io = io;
    simulationBody = io->body();
    timeStep_ = io->timeStep();
    controlTime_ = io->currentTime();

    for(auto joint : simulationBody->joints()){
        if(joint->isRevoluteJoint() || joint->isPrismaticJoint()){
            joint->setActuationMode(Link::JOINT_EFFORT);
        }
    }

    forceSensors_ = simulationBody->devices<ForceSensor>().getSortedById();
    gyroSensors_ = simulationBody->devices<RateGyroSensor>().getSortedById();
    accelSensors_ = simulationBody->devices<AccelerationSensor>().getSortedById();

    executionCycle = (executionCycleProperty > 0.0) ? executionCycleProperty : timeStep_;
    executionCycleCounter = executionCycle;

    return true;
}


bool BodyRTCItem::start()
{
    bool isReady = true;
    
    if(rtcomp && !rtcomp->isValid()){
        mv->putln(fmt(_("RTC \"%1%\" is not ready.")) % rtcomp->name());
        isReady = false;
    }

    if(virtualRobotRTC) {
        virtualRobotRTC->initialize(simulationBody);
        if(!virtualRobotRTC->checkOutPortStepTime(timeStep_)){
            mv->putln(fmt(_("Output interval must be longer than the control interval.")));
            isReady = false;
        }
    }

    if(isReady){
        rtcInfoVector.clear();
        detectRtcs();
        setupRtcConnections();
        activateComponents();
    }

#ifdef ENABLE_SIMULATION_PROFILING
    bodyRTCTime = 0.0;
#endif

    return isReady;
}


double BodyRTCItem::timeStep() const
{
    return timeStep_;
}


void BodyRTCItem::input()
{
    controlTime_ = io->currentTime();

    // write the state of simulationBody to out-ports
    virtualRobotRTC->inputDataFromSimulator(this);
}


bool BodyRTCItem::control()
{
    // tick the execution context of the connected RTCs
    virtualRobotRTC->writeDataToOutPorts(controlTime_, timeStep_);

    if(!CORBA::is_nil(virtualRobotEC)){
        executionCycleCounter += timeStep_;
        if(executionCycleCounter + timeStep_ / 2.0 > executionCycle){

#ifdef ENABLE_SIMULATION_PROFILING
    timer.begin();
#endif

            virtualRobotEC->tick();

#ifdef ENABLE_SIMULATION_PROFILING
    bodyRTCTime = timer.measure();
#endif

            executionCycleCounter -= executionCycle;
        }
    }

#ifdef ENABLE_SIMULATION_PROFILING
    timer.begin();
#endif

    for(RtcInfoVector::iterator p = rtcInfoVector.begin(); p != rtcInfoVector.end(); ++p){
        RtcInfoPtr& rtcInfo = *p;
        if(!CORBA::is_nil(rtcInfo->execContext)){
            rtcInfo->timeRateCounter += rtcInfo->timeRate;
            if(rtcInfo->timeRateCounter + rtcInfo->timeRate/2.0 > 1.0){
                rtcInfo->execContext->tick();
                rtcInfo->timeRateCounter -= 1.0;
            }
        }
    }

#ifdef ENABLE_SIMULATION_PROFILING
    controllerTime = timer.measure();
#endif

    virtualRobotRTC->readDataFromInPorts();

    return true;
}


void BodyRTCItem::output()
{
    // read in-ports and write the values to simulationBody
    virtualRobotRTC->outputDataToSimulator(simulationBody);
}

    
void BodyRTCItem::stop()
{
    // deactivate the RTC
    deactivateComponents();
}


void BodyRTCItem::setControllerModule(const std::string& name)
{
    if(name != moduleName){

        filesystem::path modulePath(name);
        if(modulePath.is_absolute()){
            baseDirectoryType.select(NO_BASE_DIRECTORY);
            if(modulePath.parent_path() == rtcDirectory){
                baseDirectoryType.select(RTC_DIRECTORY);
                modulePath = modulePath.filename();
            } else {
                filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());
                if(!projectDir.empty() && (modulePath.parent_path() == projectDir)){
                    baseDirectoryType.select(PROJECT_DIRECTORY);
                    modulePath = modulePath.filename();
                }
            }
        }
        moduleName = modulePath.string();

        BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setAutoConnectionMode(bool on)
{
    autoConnect = on;
}


void BodyRTCItem::setConfigFile(const std::string& name)
{
    if(name != confFileName){
        confFileName = name;
        if(configMode.is(CONF_ALL_MODE))
            return;
        BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setConfigMode(int mode)
{
    if(mode != configMode.which()){
        configMode.select(mode);
        BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setPeriodicRate(double freq)
{
    executionCycleProperty = 1.0 / freq;
}


void BodyRTCItem::setInstanceName(const std::string& name)
{
    if(instanceName!=name){
        instanceName = name;
        BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setBaseDirectoryType(int type)
{
    if(type != baseDirectoryType.which()){
        baseDirectoryType.select(type);
        BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}

void BodyRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    putProperty(_("Auto Connect"), autoConnect, changeProperty(autoConnect));
    putProperty(_("RTC Instance name"), instanceName,
                [&](const string& name){ setInstanceName(name); return true; });
    putProperty.decimals(3)(_("Periodic rate"), executionCycleProperty,
                            changeProperty(executionCycleProperty));

    FilePathProperty moduleProperty(
        moduleName,
        { str(format(_("RT-Component module (*%1%)")) % DLL_SUFFIX) });

    FilePathProperty confFileProperty(
        confFileName,
        { _("RTC cconfiguration file (*.conf)") });

    if(baseDirectoryType.is(RTC_DIRECTORY)){
        moduleProperty.setBaseDirectory(rtcDirectory.string());
        confFileProperty.setBaseDirectory(moduleProperty.baseDirectory());
    } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
        confFileProperty.setBaseDirectory(moduleProperty.baseDirectory());
    }
    putProperty(_("Base directory"), baseDirectoryType,
                [&](int which){ setBaseDirectoryType(which); return true; });

    putProperty(_("Controller module"), moduleProperty,
                [&](const string& name){ setControllerModule(name); return true; });

    putProperty(_("Configuration mode"), configMode,
                [&](int which){ setConfigMode(which); return true; });

    putProperty(_("Configuration file"), confFileProperty,
                [&](const string& name){ setConfigFile(name); return true; });
}


bool BodyRTCItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    archive.writeRelocatablePath("moduleName", moduleName);
    archive.writeRelocatablePath("confFileName", confFileName);
    archive.write("configurationMode", configMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("AutoConnect", autoConnect);
    archive.write("InstanceName", instanceName, DOUBLE_QUOTED);
    archive.write("bodyPeriodicRate", executionCycleProperty);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);

    return true;
}


bool BodyRTCItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    string value;
    if(archive.read("moduleName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        moduleName = getNativePathString(path);
    }
    if(archive.read("confFileName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        confFileName = getNativePathString(path);
    }
    if(archive.read("configurationMode", value)){
        configMode.select(value);
    }
    if(archive.read("baseDirectory", value) || archive.read("RelativePathBase", value)){
        baseDirectoryType.select(value);
    }
    archive.read("AutoConnect", autoConnect);
    archive.read("InstanceName", instanceName);
    archive.read("bodyPeriodicRate", executionCycleProperty);

    return true;
}


// Detects the RTC specified in the config file and the RTC already connected to the robot.
void BodyRTCItem::detectRtcs()
{
    RTC::Manager& rtcManager = RTC::Manager::instance();
    
    string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    naming = new RTC::CorbaNaming(rtcManager.getORB(), nameServer.c_str());

    for(TimeRateMap::iterator it = bridgeConf->timeRateMap.begin(); it != bridgeConf->timeRateMap.end(); ++it){
        RTC::RTObject_var rtcRef;
        string rtcName = it->first;
        if( rtcName != "") {
            string rtcNamingName = rtcName + ".rtc";
            CORBA::Object_var objRef;
            try {
                objRef = naming->resolve(rtcNamingName.c_str());
            } catch(const CosNaming::NamingContext::NotFound &ex) {

            }
            if(CORBA::is_nil(objRef)) {
                mv->putln(fmt(_("%1% is not found.")) % rtcName);
            } else {
                rtcRef = RTC::RTObject::_narrow(objRef);
                if(CORBA::is_nil(rtcRef)){
                    mv->putln(fmt(_("%1% is not an RTC object.")) % rtcName);
                }
            }
        }
        if (!CORBA::is_nil(rtcRef)) {
            addRtcVectorWithConnection(rtcRef);
        }
    }

    mv->putln(_("setup RT components"));
    rtcInfoMap.clear();
    for(size_t i=0; i < bridgeConf->portConnections.size(); ++i){

        PortConnection& connection = bridgeConf->portConnections[i];

        for(int i=0; i<2; i++){
            RTC::RTObject_var rtcRef = 0;
            string rtcName;

            if(!connection.InstanceName[i].empty()){
                rtcName = connection.InstanceName[i];
            } else {
                if(i==1){
                    if(rtcomp && rtcomp->rtc()){
                        RTC::RtcBase* rtcServant = rtcomp->rtc();
                        rtcName = rtcServant->getInstanceName();                                                 
                        rtcRef = rtcServant->getObjRef();
                    }
                } else {
                    continue;
                }
            }
            
            RtcInfoMap::iterator it = rtcInfoMap.find(rtcName);
            if(it == rtcInfoMap.end()){
                if(!rtcRef){
                    string rtcNamingName = rtcName + ".rtc";
                    CORBA::Object_var objRef;
                    try {
                        objRef = naming->resolve(rtcNamingName.c_str());
                    } catch(const CosNaming::NamingContext::NotFound &ex) {

                    }
                    if(CORBA::is_nil(objRef)){
                        mv->putln(fmt(_("%1% is not found.")) % rtcName);
                    } else {
                        rtcRef = RTC::RTObject::_narrow(objRef);
                        if(CORBA::is_nil(rtcRef)){
                            mv->putln(fmt(_("%1% is not an RTC object.")) % rtcName);
                        }
                    }
                }
                if(!CORBA::is_nil(rtcRef)){
                    RtcInfoPtr rtcInfo = addRtcVectorWithConnection(rtcRef);
                    rtcInfoMap.insert(make_pair(rtcName,rtcInfo));
                }
            }
        }
    }

    RTC::RTCList_var rtcList = virtualRobotRTC->getConnectedRtcs();
    for(CORBA::ULong i=0; i < rtcList->length(); ++i){
        addRtcVectorWithConnection(rtcList[i]);
    }
}

void BodyRTCItem::makePortMap(RtcInfoPtr& rtcInfo)
{
    RTC::PortServiceList_var ports = rtcInfo->rtcRef->get_ports();
    for(CORBA::ULong i=0; i < ports->length(); ++i){
        RTC::PortProfile_var profile = ports[i]->get_port_profile();
        std::string portName(profile->name);
        string::size_type index = portName.rfind(".");
        if (index != string::npos) portName = portName.substr(index+1);
        rtcInfo->portMap[portName] = ports[i];
    }
}

/// Create a port map of new_rtcRef and register it in rtcInfoVector.
BodyRTCItem::RtcInfoPtr BodyRTCItem::addRtcVectorWithConnection(RTC::RTObject_var new_rtcRef)
{
    RtcInfoVector::iterator it = rtcInfoVector.begin();
    for( ; it != rtcInfoVector.end(); ++it){
        if((*it)->rtcRef->_is_equivalent(new_rtcRef))
            return *it;
    }

    RtcInfoPtr rtcInfo(new RtcInfo());
    rtcInfo->rtcRef = new_rtcRef;
    makePortMap(rtcInfo);
    string rtcName = (string)rtcInfo->rtcRef->get_component_profile()->instance_name;

    if (bridgeConf->timeRateMap.size() == 0 ) {
        rtcInfo->timeRate = 1.0;
        rtcInfo->timeRateCounter = 0.0;
    } else {
        TimeRateMap::iterator p = bridgeConf->timeRateMap.find(rtcName);
        if ( p != bridgeConf->timeRateMap.end() ) {
            rtcInfo->timeRate = (double)p->second;
            rtcInfo->timeRateCounter = 1.0 - rtcInfo->timeRate;
        } else {
            rtcInfo->timeRate = 0.0;
            rtcInfo->timeRateCounter = 0.0;
        }
        mv->putln(fmt(_("periodic-rate (%1%) = %2% ")) % rtcName % rtcInfo->timeRate);
    }
    rtcInfoVector.push_back(rtcInfo);

    RTC::ExecutionContextList_var eclist = rtcInfo->rtcRef->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            rtcInfo->execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            SDOPackage::NVList& properties = rtcInfo->rtcRef->get_component_profile()->properties;
            const char* ec_type(0);
            NVUtil::find(properties, "exec_cxt.periodic.type") >>= ec_type;
            if(!CORBA::is_nil(rtcInfo->execContext)){
                mv->putln(_("detected the ExtTrigExecutionContext"));
            }
            break;
        }
    }
    return rtcInfo;
}

void BodyRTCItem::activateComponents()
{
    for(RtcInfoVector::iterator p = rtcInfoVector.begin(); p != rtcInfoVector.end(); ++p){
        RtcInfoPtr& rtcInfo = *p;
        if(!CORBA::is_nil(rtcInfo->execContext)){
            if( RTC::PRECONDITION_NOT_MET == rtcInfo->execContext->activate_component(rtcInfo->rtcRef) ){
                rtcInfo->execContext->reset_component(rtcInfo->rtcRef);
                rtcInfo->execContext->activate_component(rtcInfo->rtcRef);
            }
        }
    }

    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            if(!CORBA::is_nil(execContext)){
                if(RTC::PRECONDITION_NOT_MET == execContext->activate_component(virtualRobotRTC->getObjRef())){
                    execContext->reset_component(virtualRobotRTC->getObjRef());
                    execContext->tick();
                    execContext->activate_component(virtualRobotRTC->getObjRef());
                }
                execContext->tick();
            }
            break;
        }
    }
}


void BodyRTCItem::deactivateComponents()
{
    std::vector<OpenRTM::ExtTrigExecutionContextService_var> vecExecContext;

    for(RtcInfoVector::iterator p = rtcInfoVector.begin(); p != rtcInfoVector.end(); ++p){
        RtcInfoPtr& rtcInfo = *p;
        if(!CORBA::is_nil(rtcInfo->execContext)){
            rtcInfo->execContext->deactivate_component(rtcInfo->rtcRef);
            vecExecContext.push_back(rtcInfo->execContext);
        }
    }

    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            if(!CORBA::is_nil(execContext)){
                execContext->deactivate_component(virtualRobotRTC->getObjRef());
                vecExecContext.push_back(execContext);
            }
            break;
        }
    }

    for( std::vector<OpenRTM::ExtTrigExecutionContextService_var>::iterator ite = vecExecContext.begin();
         ite != vecExecContext.end();   ++ite   ){
        if(!CORBA::is_nil( *ite )){
            (*ite)->tick();
        }
    }
}


void BodyRTCItem::setupRtcConnections()
{
    for(size_t i=0; i < bridgeConf->portConnections.size(); ++i){

        const PortConnection& connection = bridgeConf->portConnections[i];

        string instanceName0 = connection.InstanceName[0];
        string instanceName1 = connection.InstanceName[1];
        if(instanceName1.empty()){
            if(rtcomp && rtcomp->rtc()){
                instanceName1 = rtcomp->rtc()->getInstanceName();
            }
        }

        bool instance0isRobot = false;
        RtcInfoPtr rtcInfo0;
        RtcInfoPtr rtcInfo1; 

        if(!instanceName0.empty()){
            RtcInfoMap::iterator p = rtcInfoMap.find(instanceName0);
            if(p != rtcInfoMap.end()){
                rtcInfo0 = p->second;
            }
        } else {
            instance0isRobot = true;
            instanceName0 = virtualRobotRTC->getInstanceName();
        }
        if(!instanceName0.empty()){
            RtcInfoMap::iterator p = rtcInfoMap.find(instanceName1);
            if(p != rtcInfoMap.end()){
                rtcInfo1 = p->second;
            }
        } else {
            continue;
        }

        if( ( rtcInfo0 || instance0isRobot ) && rtcInfo1){
            RTC::PortService_var portRef0;
            if(!instance0isRobot){
                PortMap::iterator q = rtcInfo0->portMap.find(connection.PortName[0]);
                if(q == rtcInfo0->portMap.end()){
                    mv->putln(fmt(_("%1% does not have a port %2%.")) % instanceName0 % connection.PortName[0]);
                    continue;
                }
                portRef0 = q->second;
            }

            RTC::PortService_var portRef1;
            PortMap::iterator q = rtcInfo1->portMap.find(connection.PortName[1]);
            if(q == rtcInfo1->portMap.end()){
                mv->putln(fmt(_("%1% does not have a port %2%.")) % instanceName1 % connection.PortName[1]);
                continue;
            }
            portRef1 = q->second;
            RTC::PortProfile_var profile = portRef1->get_port_profile();
            const char* type;
            if (!(NVUtil::find(profile->properties, "port.port_type") >>= type)) {
                continue;
            }
            bool port1isIn = false;
            if(!std::strcmp(type,"DataInPort")){
                port1isIn = true;
            }

            if(instance0isRobot){
                PortHandlerPtr robotPortHandler;
                if(port1isIn){
                    robotPortHandler = virtualRobotRTC->getOutPortHandler(connection.PortName[0]);
                } else {
                    robotPortHandler = virtualRobotRTC->getInPortHandler(connection.PortName[0]);
                }
                if(!robotPortHandler){
                    mv->putln(fmt(_("The robot does not have a port named %1%.")) % connection.PortName[0]);
                    continue;
                }
                portRef0 = robotPortHandler->portRef;
                if(CORBA::is_nil(portRef0)){
                    continue;
                }
            }

            int connected = false;
            if(port1isIn){
                mv->putln(fmt(_("connect %1%:%2% --> %3%:%4%")) 
                          % instanceName0 % connection.PortName[0] % instanceName1 % connection.PortName[1]);
                connected = connectPorts(portRef0, portRef1);
            }else{
                mv->putln(fmt(_("connect %1%:%2% <-- %3%:%4%")) 
                          % instanceName0 % connection.PortName[0] % instanceName1 % connection.PortName[1]);
                connected = connectPorts(portRef1, portRef0);
            }

            if(!connected){
                mv->putln( _("Connection was successful."));
            } else if(connected == -1){
                mv->putln(_("Connection failed."));
            } else if(connected == 1){
                mv->putln(_(" It has already been connected."));
            }
        }
    }

    if(configMode.is(CONF_ALL_MODE) && autoConnect){
        std::vector<RTC::RTObject_var> rtcRefs;
        if(!moduleName.empty()){
            if(rtcomp && rtcomp->rtc()){
                rtcRefs.push_back(rtcomp->rtc()->getObjRef());
            }
        }
        if(rtcRefs.empty()){
            CosNaming::BindingList_var bl;
            CosNaming::BindingIterator_var bi;
            naming->list(naming->getRootContext(), 100, bl, bi);
            CORBA::ULong len(bl->length());
            for(CORBA::ULong i = 0; i < len; ++i){
                string name(naming->toString(bl[i].binding_name));
                if(name != instanceName+".rtc" && name.find(".rtc") != string::npos){
                    RTC::RTObject_var rtcRef;
                    try {
                        rtcRef = RTC::RTObject::_narrow(naming->resolve(bl[i].binding_name));
                    } catch(const CosNaming::NamingContext::NotFound &ex) {
                        
                    }
                    rtcRefs.push_back(rtcRef);
                }
            }
        }

        PortInfoMap& outPortInfoMap = bridgeConf->outPortInfos;
        for(PortInfoMap::iterator it=outPortInfoMap.begin(); it!=outPortInfoMap.end(); it++){
            string robotPortName(it->first);

            for(std::vector<RTC::RTObject_var>::iterator it = rtcRefs.begin(); it != rtcRefs.end(); it++){
                RTC::PortServiceList_var ports = (*it)->get_ports();
                for(CORBA::ULong i=0; i < ports->length(); ++i){
                    RTC::PortProfile_var profile = ports[i]->get_port_profile();
                    const char* type;
                    if (!(NVUtil::find(profile->properties, "port.port_type") >>= type)){
                        break;
                    }
                    if(!std::strcmp(type,"DataInPort")){
                        std::string portName(profile->name);
                        string::size_type index = portName.rfind(".");
                        if (index != string::npos) portName = portName.substr(index+1);
                        if(portName==robotPortName || (robotPortName=="u_out" && portName=="u") ||
                           (robotPortName=="u_out" && portName=="u_in")){
                            RtcInfoPtr rtcInfo = addRtcVectorWithConnection(*it);
                            PortMap::iterator q = rtcInfo->portMap.find(portName);
                            RTC::PortService_var controllerPortRef = q->second;
                            PortHandlerPtr robotPortHandler = virtualRobotRTC->getOutPortHandler(robotPortName);
                            RTC::PortService_var  robotPortRef = robotPortHandler->portRef;
                            if(!CORBA::is_nil(robotPortRef)){
                                int connected = connectPorts(robotPortRef, controllerPortRef);
                                mv->putln(fmt(_("connect %1%:%2% --> %3%:%4%")) 
                                          % virtualRobotRTC->getInstanceName() % robotPortName % (*it)->get_component_profile()->instance_name % portName);
                                if(!connected){
                                    mv->putln(_("Connection was successful."));
                                } else if(connected == -1){
                                    mv->putln(_("Connection failed."));
                                } else if(connected == 1){
                                    mv->putln(_(" It has already been connected."));
                                }
                            }
                        }
                    }
                }
            }
        }

        PortInfoMap& inPortInfoMap = bridgeConf->inPortInfos;
        for(PortInfoMap::iterator it=inPortInfoMap.begin(); it!=inPortInfoMap.end(); it++){
            string robotPortName(it->first);

            for(std::vector<RTC::RTObject_var>::iterator it = rtcRefs.begin(); it != rtcRefs.end(); it++){
                RTC::PortServiceList_var ports = (*it)->get_ports();
                for(CORBA::ULong i=0; i < ports->length(); ++i){
                    RTC::PortProfile_var profile = ports[i]->get_port_profile();
                    const char* type;
                    if(!(NVUtil::find(profile->properties, "port.port_type") >>= type)){
                        break;
                    }
                    if(!std::strcmp(type,"DataOutPort")){
                        std::string portName(profile->name);
                        string::size_type index = portName.rfind(".");
                        if (index != string::npos) portName = portName.substr(index+1);
                        if(portName==robotPortName || (robotPortName=="u_in" && portName=="u") ||
                           (robotPortName=="u_in" && portName=="u_out")){
                            RtcInfoPtr rtcInfo = addRtcVectorWithConnection(*it);
                            PortMap::iterator q = rtcInfo->portMap.find(portName);
                            RTC::PortService_var controllerPortRef = q->second;
                            PortHandlerPtr robotPortHandler = virtualRobotRTC->getInPortHandler(robotPortName);
                            RTC::PortService_var  robotPortRef = robotPortHandler->portRef;
                            if(!CORBA::is_nil(robotPortRef)){
                                int connected = connectPorts(controllerPortRef, robotPortRef);
                                mv->putln(fmt(_("connect %1%:%2% <-- %3%:%4%"))
                                          % virtualRobotRTC->getInstanceName() % robotPortName % (*it)->get_component_profile()->instance_name % portName);
                                if(!connected){
                                    mv->putln(_("Connection was successful."));
                                } else if(connected == -1){
                                    mv->putln(_("Connection failed."));
                                } else if(connected == 1){
                                    mv->putln(_(" It has already been connected."));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

int BodyRTCItem::connectPorts(RTC::PortService_var outPort, RTC::PortService_var inPort)
{
    RTC::ConnectorProfileList_var connectorProfiles = inPort->get_connector_profiles();
    for(CORBA::ULong i=0; i < connectorProfiles->length(); ++i){
        RTC::ConnectorProfile& connectorProfile = connectorProfiles[i];
        RTC::PortServiceList& connectedPorts = connectorProfile.ports;

        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            RTC::PortService_ptr connectedPortRef = connectedPorts[j];
            if(connectedPortRef->_is_equivalent(outPort)){
                return 1;
            }
        }
    }
    // connect ports
    RTC::ConnectorProfile cprof;
    cprof.connector_id = "";
    cprof.name = CORBA::string_dup("connector0");
    cprof.ports.length(2);
    cprof.ports[0] = RTC::PortService::_duplicate(inPort);
    cprof.ports[1] = RTC::PortService::_duplicate(outPort);

    CORBA_SeqUtil::push_back(cprof.properties,
                             NVUtil::newNV("dataport.dataflow_type",
                                           "Push"));
    CORBA_SeqUtil::push_back(cprof.properties,
                             NVUtil::newNV("dataport.interface_type",
                                           "corba_cdr"));
    CORBA_SeqUtil::push_back(cprof.properties,
                             NVUtil::newNV("dataport.subscription_type",
                                           "flush"));
    RTC::ReturnCode_t result = inPort->connect(cprof);

    if(result == RTC::RTC_OK)
        return 0;
    else
        return -1;
}


void BodyRTCItem::deleteModule(bool waitToBeDeleted)
{
    RTC::Manager& rtcManager = RTC::Manager::instance();
    
    if(TRACE_FUNCTIONS){
        cout << "BodyRTCItem::deleteModule()" << endl;
    }
    
    std::vector<string> deleteList;
    if(bridgeConf){
        ModuleInfoList& moduleInfoList = bridgeConf->moduleInfoList;
        ModuleInfoList::iterator it;
        for(it=moduleInfoList.begin(); it != moduleInfoList.end(); ++it){
            if(it->isLoaded){
                if(it->rtcServant){
                    deleteList.push_back(it->rtcServant->getInstanceName());
                    it->rtcServant->exit();
                    mv->putln(fmt(_("delete %1%")) % it->rtcServant->getInstanceName());
                }
            }
        }
        delete bridgeConf;
        bridgeConf = 0;
    }
    
    if(rtcomp){
        rtcomp->deleteRTC();
        delete rtcomp;
        rtcomp = 0;
    }

    if(virtualRobotRTC){
        deleteList.push_back(virtualRobotRTC->getInstanceName());
        mv->putln(fmt(_("delete %1%")) % virtualRobotRTC->getInstanceName());
        cnoid::deleteRTC(virtualRobotRTC);
        virtualRobotRTC = 0;
    }

    if(waitToBeDeleted){
        std::vector<string> remainder;
        for(int i=0; i < 100; i++){
            remainder.clear();
            for(std::vector<string>::iterator it=deleteList.begin(); it!=deleteList.end(); it++){
                RTC::RtcBase* component = rtcManager.getComponent((*it).c_str());
                if(component){
                    remainder.push_back(*it);
                }
            }
            if(remainder.empty()){
                return;
            }
            msleep(20);
        }
        for(std::vector<string>::iterator it=remainder.begin(); it!=remainder.end(); it++){
            mv->putln(fmt(_("%1% cannot be deleted.")) % *it);
        }
    }

    if(TRACE_FUNCTIONS){
        cout << "End of BodyRTCItem::deleteModule()" << endl;
    }
}

#ifdef ENABLE_SIMULATION_PROFILING
void BodyRTCItem::getProfilingNames(vector<string>& profilingNames)
{
    profilingNames.push_back("    BodyRTC calculation time");
    profilingNames.push_back("    Controller calculation time");
}


void BodyRTCItem::getProfilingTimes(vector<double>& profilingToimes)
{
    profilingToimes.push_back(bodyRTCTime);
    profilingToimes.push_back(controllerTime);
}
#endif
