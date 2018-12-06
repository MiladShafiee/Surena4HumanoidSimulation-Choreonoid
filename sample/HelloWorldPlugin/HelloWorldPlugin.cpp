/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <mainwindow1.h>
#include <QApplication>
#include <QWidget>
#include <QMainWindow>
    #include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

using namespace std;
using namespace cnoid;

class HelloWorldPlugin : public Plugin
{
public:
       QWidget window;
    HelloWorldPlugin() : Plugin("HelloWorld")
    {

    }
    
    virtual bool initialize()
    {
        Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
        menuItem->sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));
        return true;
    }

private:
    
    void onHelloWorldTriggered()
    {
        MessageView::instance()->putln("Hello World !");
         QApplication a();
    MainWindow1 mainwindow1;
   mainwindow1.show();

int argc;
   QApplication app();

       QLabel *label = new QLabel(QApplication::translate("windowlayout", "Name:"));
       QLineEdit *lineEdit = new QLineEdit();

       QHBoxLayout *layout = new QHBoxLayout();
       layout->addWidget(label);
       layout->addWidget(lineEdit);
       window.setLayout(layout);
       window.setWindowTitle(
           QApplication::translate("windowlayout", "Window layout"));
       window.show();


    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
