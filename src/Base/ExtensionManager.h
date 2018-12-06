/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_EXTENSION_MANAGER_H
#define CNOID_BASE_EXTENSION_MANAGER_H

#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Item;
class View;
class ToolBar;
class Archive;
class ItemManager;
class ViewManager;
class TimeSyncItemEngineManager;
class MenuManager;
class OptionManager;

class ExtensionManagerImpl;

class CNOID_EXPORT ExtensionManager
{
public:

    ExtensionManager(const std::string& moduleName, bool isPlugin);
    ExtensionManager(const std::string& moduleName, const std::string& version, bool isPlugin);
    virtual ~ExtensionManager();

    const std::string& name() const;
    const std::string& textDomain() const;

    ItemManager& itemManager();
    TimeSyncItemEngineManager& timeSyncItemEngineManger();
    ViewManager& viewManager();
    MenuManager& menuManager();
    OptionManager& optionManager();

private:

    struct CNOID_EXPORT PtrHolderBase {
        virtual ~PtrHolderBase();
    };

    // smart pointer version
    template <class PointerType> struct PtrHolder : public PtrHolderBase {
        PtrHolder(PointerType pointer) : pointer(pointer) { }
        virtual ~PtrHolder() { }
        PointerType pointer;
    };

    // raw pointer version
    template <class Object> struct PtrHolder<Object*> : public PtrHolderBase {
        PtrHolder(Object* pointer) : pointer(pointer) { }
        virtual ~PtrHolder() { delete pointer; }
        Object* pointer;
    };

    void manageSub(PtrHolderBase* holder);

public:

    void addToolBar(ToolBar* toolBar);

    template <class PointerType> PointerType manage(PointerType pointer) {
        manageSub(new PtrHolder<PointerType>(pointer));
        return pointer;
    }

    /**
       This signal is emitted when the states of the objects available in the system are updated,
       such as when the initial generation of the object is completed at startup,
       when a new plug-in is read or released.
    */
    SignalProxy<void()> sigSystemUpdated();
        
    /**
       This function emits the SystemUpdated signal.

       @note Emitting a signal is not done at the time of calling this function, but later
       in the event loop. As a result, the processing of the callback function becomes
       after the other events under processing are processed. Also, even if this function
       is called more than once at the same time, emission of signals will be brought together.
    */
    static void notifySystemUpdate();

    SignalProxy<void()> sigReleaseRequest();

    void setProjectArchiver(
        const std::string& name,
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);

    void setProjectArchiver(
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);
        
private:
    ExtensionManager(const ExtensionManager& org);
        
    ExtensionManagerImpl* impl;

    friend class ExtensionManagerImpl;
};

}

#endif
