/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LAZY_SIGNAL_H
#define CNOID_BASE_LAZY_SIGNAL_H

#include "LazyCaller.h"
#include <cnoid/Signal>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LazySignalBase : public LazyCaller
{
public:
    void request();
    void requestBlocking(Connection connection){
        connectionsToBlock.push_back(connection);
    }
protected:
    LazySignalBase();
    LazySignalBase(std::function<void()> emitFunction, int priority);
    std::function<void()> emitFunction;
    std::vector<Connection> connectionsToBlock;
    virtual void defaultEmitFunction() = 0;

private:
    bool doEmit();
};

template <class SignalType> class LazySignal : public LazySignalBase
{
public:
    LazySignal() { }

    LazySignal(std::function<void()> emitFunction, int priority = LazyCaller::PRIORITY_HIGH)
        : LazySignalBase(emitFunction, priority) {
    }

    SignalType& signal() { return signal_; }

protected:
    virtual void defaultEmitFunction() {
        signal_();
    }

private:
    SignalType signal_;
};

}
        
#endif
