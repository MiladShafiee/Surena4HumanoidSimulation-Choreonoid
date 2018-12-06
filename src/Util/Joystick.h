/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_JOYSTICK_H
#define CNOID_UTIL_JOYSTICK_H

#include "Signal.h"
#include "exportdecl.h"

namespace cnoid {

class JoystickImpl;

class CNOID_EXPORT Joystick
{
public:
    Joystick();
    Joystick(const char* device);
    virtual ~Joystick();

    int fileDescriptor() const;

    bool isReady() const;
    const char* errorMessage() const;

    int numAxes() const;
    void setAxisEnabled(int axis, bool on);
    int numButtons() const;
    bool readCurrentState();
    double getPosition(int axis) const;
    bool getButtonState(int button) const;
    double getNativePosition(int axis) const;
    bool getNativeButtonState(int button) const;
    bool isActive() const;
    SignalProxy<void(int id, bool isPressed)> sigButton();
    SignalProxy<void(int id, double position)> sigAxis();

private:
    JoystickImpl* impl;
    friend class JoystickImpl;
};

}

#endif
