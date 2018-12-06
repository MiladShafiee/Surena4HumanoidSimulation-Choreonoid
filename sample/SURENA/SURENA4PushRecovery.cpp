#include <cnoid/SimulatorItem>
#include <cnoid/SimpleController>
#include <vector>
#include <cnoid/RootItem>
#include <iostream>
#include <QString>
#include <QList>
#include "Headers/Robot.h"
#include"Headers/TaskSpace.h"
#include"Headers/taskspaceoffline.h"
#include <qmath.h>
#include <cstring>
#include <cnoid/BodyLoader>
#include "src/BodyPlugin//BodyItem.h"
#include <cnoid/Sensor>
#include <QApplication>
#include<Headers/mainwindow.h>
//#include <cnoid/MainWindow>
//#include <ros/node_handle.h>
//#include <sensor_msgs/JointState.h>
//#include <sensor_msgs/image_encodings.h>

using namespace cnoid;
using namespace std;

//const double pgain[] = {
//     600.0,  600.0,  1200.0,  1500.0,  1200.0,  1000.0,
//     600.0,  600.0,  1200.0,  1500.0,  1200.0,  1000.0,
//     600.0,  1300.0,  500.0, 500.0,  500.0,  500.0,  500.0,  500.0,  500.0,
//     500.0, 500.0,  500.0,  500.0,  500.0,  500.0,  500.0 };

//const double dgain[] = {
//    15.0, 15.0, 20.0, 20.0, 20.0, 15.0,
//    15.0, 15.0, 20.0, 20.0, 20.0, 15.0,
//    15.0, 30.0, 15.0, 15.0, 15.0, 15.0,
//    15.0, 15.0, 15.0, 15.0, 15.0, 15.0,
//    15.0, 15.0, 15.0, 15.0};
const double pgain[] = {
    3000.0, 3000.0, 3000.0, 5000.0, 5000.0, 5000.0,
    3000.0, 3000.0,3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0,3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0,3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0};
QApplication a();

    MainWindow miladplot;
class SURENA4PushRecovery : public SimpleController
{


    double dt;
    BodyPtr ioBody;
    Robot SURENA;
    TaskSpaceOffline SURENAOffilneTaskSpace;
    QList<LinkM> links;
    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;
    //BodyItemPtr SurenaItemBody;
    //BodyItem* mil;
    vector<double> qref;
    vector<double> qold;
    double footPitchOld;
    double footRollOld;
      //  milad();



     double waistGyroOffset = 0;
     double alpha=1;
     double K_shoulder = alpha*1*0.00750;
     double K_elbow = alpha*1*0.0069;
     double K_pelvis = alpha*1*0.00460;
     double K_knee = alpha*1*0.0040;
     double K_ankle = alpha*1*0.0011;

     ItemPtr currentBodyItem;
     BodyItem* currentBodyItem1;

     SimulatorItem* activeSimulator;
     SimpleControllerIO* ioG;
public:

    BodyLoader bodyloader;

    RateGyroSensorPtr waistGyro;
    RateGyroSensorPtr ankleRightGyro;
    RateGyroSensorPtr ankleLefttGyro;

    ForceSensorPtr ankleRightForce;
    ForceSensorPtr ankleLeftForce;

    AccelerationSensor* waistAccelSensor;
    AccelerationSensor* ankleLeftAccelSensor;
    AccelerationSensor* ankleRightAccelSensor;


    virtual bool initialize(SimpleControllerIO* io) override
    {

        ioBody = io->body();
               dt = io->timeStep();

               ioG = io;
               for(auto joint : ioBody->joints()){
                   joint->setActuationMode(Link::JOINT_TORQUE);
                   io->enableIO(joint);
                   qref.push_back(joint->q());
               }
               qold = qref;

               DeviceList<RateGyroSensor> sensors(ioBody->devices());

               if(!sensors.empty()){
                   waistGyro = sensors.front();
                   waistGyro->clearState();
               }
               io->enableInput(waistGyro);
               waistGyroOffset = waistGyro->w()(1);
               currentBodyItem = RootItem::instance()->findItem("World/SURENA4");//it should be eidted
               currentBodyItem1 = RootItem::instance()->findItem<BodyItem>("World/SURENA4");
               return true;
    }

    virtual bool control() override
    {
        Link* waistLink = ioBody->joint(0);
               Vector3 force;
               force << 130, 0, 0;
               Vector3 point;
               point <<0, 0, 0.00;
               if(ioG->currentTime() == 3){
                  activeSimulator = SimulatorItem::findActiveSimulatorItemFor(currentBodyItem);
                  activeSimulator->setExternalForce(currentBodyItem1, waistLink, point, force, 0.5);
               }
               for(int i=0; i < ioBody->numJoints(); ++i){
                   Link* joint = ioBody->joint(i);
                   double u;
                   double q = joint->q();
                   double dq = (q - qold[i]) / dt;
                   Vector3 waistGyroVal = waistGyro->w();

                   double waistFeedback = waistGyroVal(1);

                   if (ioG->currentTime()>=3 && ioG->currentTime()<=10) {


                   if((i == 2) || (i == 8)){
                       qref[i] -=(1*K_pelvis*(waistFeedback) - 0*(waistGyroOffset));
                   }

                   if((i == 3) || (i == 9)){
                       qref[i] +=(1*K_knee*(waistFeedback) - 0*(waistGyroOffset));
                   }

                   if((i == 4) || (i == 10)){
                       qref[i] -=(1*K_ankle*(waistFeedback) - 0*(waistGyroOffset));
                   }

                   if((i == 14) || (i == 21)){
                       qref[i] +=(1*K_shoulder*(waistFeedback) - 0*(waistGyroOffset));
                   }

                   if((i == 17) || (i == 24)){
                       qref[i] +=(1*K_elbow*(waistFeedback) - 0*(waistGyroOffset));
                   }
 }


//                   if(i==15){

//                       qref[i]=-0.174400;
//                   }

//                   else if (i==22) {
//                       qref[i]=0.174400;
//                   }
//                   else {

//                       qref[i]=0.000;
//                   }

                   u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
                   qold[i] = q;
                   joint->u() = u;
               }
               return true;

    }



};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SURENA4PushRecovery)
