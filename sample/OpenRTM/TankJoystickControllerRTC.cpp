/**
   @author Shin'ichiro Nakaoka
*/

#include "TankJoystickControllerRTC.h"
#include <cmath>

using namespace std;

namespace {

const double timeStep = 0.001;
const int numTankJoints = 4;
const int cannonJointId = 2;
const int cannonAxis[] = { 2, 3 };

const char* spec[] =
{
    "implementation_id", "TankJoystickControllerRTC",
    "type_name",         "TankJoystickControllerRTC",
    "description",       "Tank Joystick Controller ",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

}


TankJoystickControllerRTC::TankJoystickControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      anglesIn("q", angles),
      axesIn("axes", axes),
      buttonsIn("buttons", buttons),
      velocitiesOut("dq", velocities),
      torquesOut("u", torques),
      lightSwitchOut("light", lightSwitch)
{

}


TankJoystickControllerRTC::~TankJoystickControllerRTC()
{

}


RTC::ReturnCode_t TankJoystickControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", anglesIn);
    addInPort("axes", axesIn);
    addInPort("buttons", buttonsIn);
    
    // Set OutPort buffer
    addOutPort("dq", velocitiesOut);
    addOutPort("u", torquesOut);
    addOutPort("light", lightSwitchOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    if(anglesIn.isNew()){
        anglesIn.read();
        if(angles.data.length() >= 2){
            for(int i=0; i < 2; ++i){
                qref[i] = qprev[i] = angles.data[i];
            }
        }
    }

    velocities.data.length(2);
    torques.data.length(2);
    for(int i=0; i < 2; ++i){
        velocities.data[i] = 0.0;
        torques.data[i] = 0.0;
    }

    lightSwitch.data.length(1);
    lastLightButtonState = false;
    isLightOn = true;

    lastAxes.clear();
    lastAxes.resize(5, 0.0f);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t TankJoystickControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(axesIn.isNew()){
        axesIn.read();
        int n = std::min((int)axes.data.length(), (int)lastAxes.size());
        for(int i=0; i < n; ++i){
            lastAxes[i] = axes.data[i];
        }
    }

    velocities.data[0] = -2.0 * lastAxes[1] + lastAxes[0];
    velocities.data[1] = -2.0 * lastAxes[1] - lastAxes[0];
    velocitiesOut.write();

    double q[2] = { qprev[0], qprev[1] };
    if(anglesIn.isNew()){
        anglesIn.read();
        if(angles.data.length() >= 2){
            for(int i=0; i < 2; ++i){
                q[i] = angles.data[i];
            }
        }
    }

    static const double P = 200.0;
    static const double D = 50.0;
    for(int i=0; i < 2; ++i){
        double dq = (q[i] - qprev[i]) / timeStep;
        double dqref = 0.0;
        double pos = lastAxes[cannonAxis[i]];
        if(fabs(pos) > 0.25){
            double deltaq = 0.002 * pos;
            qref[i] += deltaq;
            dqref = deltaq / timeStep;
        }
        torques.data[i] = P * (qref[i] - q[i]) + D * (dqref - dq);
        qprev[i] = q[i];
    }
    torquesOut.write();

    if(buttonsIn.isNew()){
        buttonsIn.read();
        bool lightButtonState = buttons.data[0];
        if(lightButtonState){
            if(!lastLightButtonState){
                isLightOn = !isLightOn;
                lightSwitch.data[0] = isLightOn;
                lightSwitchOut.write();
            }
        }
        lastLightButtonState = lightButtonState;
    }

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void TankJoystickControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile,
                                 RTC::Create<TankJoystickControllerRTC>,
                                 RTC::Delete<TankJoystickControllerRTC>);
    }
};
