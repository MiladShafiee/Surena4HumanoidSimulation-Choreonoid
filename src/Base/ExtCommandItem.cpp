/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ExtCommandItem.h"
#include "ItemManager.h"
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Sleep>
#include <boost/filesystem.hpp>
#include <functional>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using namespace std::placeholders;


void ExtCommandItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<ExtCommandItem>(N_("ExtCommandItem"));
        im.addCreationPanel<ExtCommandItem>();
        initialized = true;
    }
}


ExtCommandItem::ExtCommandItem()
{
    waitingTimeAfterStarted_ = 0.0;
    signalReadyStandardOutputConnected = false;
    doCheckExistingProcess = false;
    doExecuteOnLoading = true;

    process.sigReadyReadStandardOutput().connect(
        std::bind(&ExtCommandItem::onReadyReadServerProcessOutput, this));
}


ExtCommandItem::ExtCommandItem(const ExtCommandItem& org)
    : Item(org)
{
    command_ = org.command_;
    waitingTimeAfterStarted_ = org.waitingTimeAfterStarted_;
    signalReadyStandardOutputConnected = false;
    doCheckExistingProcess = org.doCheckExistingProcess;
    doExecuteOnLoading = org.doExecuteOnLoading;

    process.sigReadyReadStandardOutput().connect(
        std::bind(&ExtCommandItem::onReadyReadServerProcessOutput, this));
}


ExtCommandItem::~ExtCommandItem()
{

}


void ExtCommandItem::onDisconnectedFromRoot()
{
    terminate();
}


Item* ExtCommandItem::doDuplicate() const
{
    return new ExtCommandItem(*this);
}


void ExtCommandItem::setCommand(const std::string& command)
{
    terminate();
    command_ = command;
    if(name().empty()){
        setName(command);
    }
}


void ExtCommandItem::setWaitingTimeAfterStarted(double time)
{
    waitingTimeAfterStarted_ = time;
}


bool ExtCommandItem::execute()
{
    bool result = false;
    
    MessageView* mv = MessageView::instance();
    
    if(!command_.empty()){
        terminate();
        string actualCommand(command_);
#ifdef _WIN32
        if(filesystem::path(actualCommand).extension() != ".exe"){
            actualCommand += ".exe";
        }
        // quote the command string to support a path including spaces
        process.start(QString("\"") + actualCommand.c_str() + "\"");
#else
        process.start(actualCommand.c_str());
#endif

        if(process.waitForStarted()){
            mv->putln(fmt(_("External command \"%1%\" has been executed by item \"%2%\"."))
                      % actualCommand % name());

            if(waitingTimeAfterStarted_ > 0.0){
                msleep(waitingTimeAfterStarted_ * 1000.0);
            }
            
            result = true;

        } else {
            mv->put(fmt(_("External command \"%1%\" cannot be executed.")) % actualCommand);
            if(!boost::filesystem::exists(actualCommand)){
                mv->putln(_(" The command does not exist."));
            } else {
                mv->putln("");
            }
        }
    }
    return result;
}


bool ExtCommandItem::terminate()
{
    if(process.state() != QProcess::NotRunning){
        process.kill();
        return process.waitForFinished(100);
    }
    return false;
}


void ExtCommandItem::onReadyReadServerProcessOutput()
{
    MessageView::instance()->put(QString(process.readAll()));
}


void ExtCommandItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Command"), command_,
                std::bind(&ExtCommandItem::setCommand, this, _1), true);
    putProperty(_("Execute on loading"), doExecuteOnLoading,
                changeProperty(doExecuteOnLoading));
    putProperty(_("Waiting time after started"), waitingTimeAfterStarted_,
                changeProperty(waitingTimeAfterStarted_));
}


bool ExtCommandItem::store(Archive& archive)
{
    //archive.writeRelocatablePath("command", command_);
    archive.write("command", command_);
    archive.write("executeOnLoading", doExecuteOnLoading);
    archive.write("waitingTimeAfterStarted", waitingTimeAfterStarted_);
    return true;
}


bool ExtCommandItem::restore(const Archive& archive)
{
    //archive.readRelocatablePath("command", command_);
    archive.read("command", command_);
    archive.read("waitingTimeAfterStarted", waitingTimeAfterStarted_);

    if(archive.read("executeOnLoading", doExecuteOnLoading)){
        if(doExecuteOnLoading){
            execute();
        }
    }
    return true;
}
