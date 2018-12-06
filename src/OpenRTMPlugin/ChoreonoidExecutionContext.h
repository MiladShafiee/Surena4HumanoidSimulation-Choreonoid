/*!
  @file ChoreonoidExecutionContext.h
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_CHOREONOID_EXECUTION_CONTEXT_H
#define CNOID_OPENRTM_PLUGIN_CHOREONOID_EXECUTION_CONTEXT_H

#include <rtm/RTC.h>
#include <coil/Task.h>
#include <rtm/Manager.h>
#if defined(OPENRTM_VERSION110)
  #include <rtm/PeriodicExecutionContext.h>
#else
  #include <rtm/OpenHRPExecutionContext.h>
#endif

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace cnoid
{
/*!
  To call 'tick()' when the 'deactivate_component" function is called,
  OpenHRPExecutionContext is redefined as this class in the OpenRTM plugin.
  See post 02356 to the openrtm-users mailing list.
*/
#ifdef OPENRTM_VERSION110
  class ChoreonoidExecutionContext : public virtual RTC::PeriodicExecutionContext
#else
  class ChoreonoidExecutionContext : public RTC::OpenHRPExecutionContext
#endif
{
public:
    ChoreonoidExecutionContext();
    virtual ~ChoreonoidExecutionContext(void);
    virtual void tick(void) throw (CORBA::SystemException);
    virtual int svc(void);
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
};
};

#endif
