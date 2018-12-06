/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "RainSnowDevice.h"
#include "SceneRainSnow.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>

using namespace std;
using namespace cnoid;

namespace {

template <class DeviceType>
bool readDevice(YAMLBodyLoader& loader, Mapping& node)
{
    ref_ptr<DeviceType> device = new DeviceType;
    return loader.readDevice(device, node);
}


template <class SceneNodeType>
SceneDevice* createSceneDevice(Device* device)
{
    auto sceneNode = new SceneNodeType;
    auto sceneDevice = new SceneDevice(device, sceneNode);

    sceneDevice->setFunctionOnTimeChanged(
        [sceneNode](double time){
            sceneNode->setTime(time);
            sceneNode->notifyUpdate();
        });
            
    return sceneDevice;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("SnowDevice", readDevice<SnowDevice>);
        SceneDevice::registerSceneDeviceFactory<SnowDevice>(createSceneDevice<SceneSnow>);

        YAMLBodyLoader::addNodeType("RainDevice", readDevice<RainDevice>);
        SceneDevice::registerSceneDeviceFactory<RainDevice>(createSceneDevice<SceneRain>);
    }
} registration;

}


RainSnowDevice::RainSnowDevice()
{
    on_ = true;
}


RainSnowDevice::RainSnowDevice(const RainSnowDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


void RainSnowDevice::copyStateFrom(const RainSnowDevice& other)
{
    on_ = other.on_;
}


void RainSnowDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RainSnowDevice))){
        Device::forEachActualType(func);
    }
}


int RainSnowDevice::stateSize() const
{
    return 1;
}


const double* RainSnowDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* RainSnowDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}


RainDevice::RainDevice()
{

}


RainDevice::RainDevice(const RainDevice& org, bool copyStateOnly)
    : RainSnowDevice(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* RainDevice::typeName()
{
    return "RainDevice";
}


void RainDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RainDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    RainSnowDevice::copyStateFrom(static_cast<const RainDevice&>(other));
}


DeviceState* RainDevice::cloneState() const
{
    return new RainDevice(*this, false);
}


Device* RainDevice::clone() const
{
    return new RainDevice(*this);
}


void RainDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RainDevice))){
        RainSnowDevice::forEachActualType(func);
    }
}


SnowDevice::SnowDevice()
{

}


SnowDevice::SnowDevice(const SnowDevice& org, bool copyStateOnly)
    : RainSnowDevice(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* SnowDevice::typeName()
{
    return "SnowDevice";
}


void SnowDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SnowDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    RainSnowDevice::copyStateFrom(static_cast<const SnowDevice&>(other));
}


DeviceState* SnowDevice::cloneState() const
{
    return new SnowDevice(*this, false);
}


Device* SnowDevice::clone() const
{
    return new SnowDevice(*this);
}


void SnowDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SnowDevice))){
        RainSnowDevice::forEachActualType(func);
    }
}
