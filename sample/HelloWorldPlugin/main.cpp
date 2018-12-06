#include "mainwindow1.h"
#include <QApplication>
#include "Eigen/Dense"
//#include "robot.h"
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <qdebug.h>
#include <qmath.h>
#include <QMainWindow>
    #include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
//#include <taskspace.h>
//#include<taskspaceoffline.h>
using namespace Eigen;
using namespace std;
int main(int argc, char *argv[])
{
    //QApplication a(argc, argv);

// MainWindow w;




    // SQDTaskSpace Forwardwalking();
    //    MatrixXd PelvisTrajectory;
    // PelvisTrajectory.setLinSpaced(81,0,0.4);
    // MatrixXd AnkleTrajectory;
    //AnkleTrajectory.setLinSpaced(81,0,0.4);
    //Robot Milad;
//TaskSpaceOffline offline;
//   TaskSpace Pelvis;
//    QApplication a(argc, argv);
//    MainWindow w;
     //QList<Link> temp = Surena4.GetLinks();
    //w.show();
    //MatrixXd temp(2,3);
    //temp<< 1,2,3,4,5,6;
    //return a.exec();
//uncomment below
   // w.show();
    //w.Plot(Pelvis);
   // return a.exec();






    QApplication app(argc, argv);
    QWidget window;
    QLabel *label = new QLabel(QApplication::translate("windowlayout", "Name:"));
    QLineEdit *lineEdit = new QLineEdit();

    QHBoxLayout *layout = new QHBoxLayout();
    layout->addWidget(label);
    layout->addWidget(lineEdit);
    window.setLayout(layout);
    window.setWindowTitle(
        QApplication::translate("windowlayout", "Window layout"));
    window.show();
    return app.exec();





}
