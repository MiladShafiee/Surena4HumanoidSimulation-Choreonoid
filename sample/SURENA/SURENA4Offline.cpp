
#include <cnoid/SimpleController>
#include <vector>
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

const double pgain[] = {
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    8000.0, 8000.0, 3000.0,3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0,3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0};

QApplication a();

    MainWindow miladplot;
class SURENA4Offline : public SimpleController
{



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

    double dt;
    SimpleControllerIO* tempIO;
public:

    double StartTime=0;
    double RollTime=0;
    double WalkTime=0;
    double  DurationOfStartPhase=6;
    double  DurationOfendPhase=6;
    double DurationOfPDTest=30;
    double hipRoll=0;



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
       // milad();
miladplot.show();
        // std::string model_path("model/SR1.body");
        // BodyPtr robot = bodyloader.load(model_path.c_str());
        ioBody = io->body();
        io->setLinkInput(io->body()->link("LLeg_AnkleR_J6"), LINK_POSITION);
        //            cout << "dof: " <<ioBody->numJoints() << endl;
        //            cout << "base link name: " << ioBody->rootLink()->name() << endl;
        //            cout << "base link pos: \n" << ioBody->rootLink()->p() << endl;
        cout << "base link rot: \n" << ioBody->rootLink()->R() << endl;

        dt = io->timeStep();
        tempIO=io;


        waistAccelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(waistAccelSensor);


        for(auto joint : ioBody->joints()){
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;
        return true;
    }

    virtual bool control() override
    {
        bool startPhase=true;
        bool endPhase=true;
        PoseRoot.resize(6,1);
        PoseRFoot.resize(6,1);
        PoseLFoot.resize(6,1);
        bool controllerPD =false;
        if (controllerPD==true && StartTime<=DurationOfPDTest) {

            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0,0.9;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<0,DurationOfPDTest;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);



            double zStart=0;
            MatrixXd outputZStart= SURENAOffilneTaskSpace.GetAccVelPos(CoefZStart,StartTime,0,5);
            zStart=outputZStart(0,0);


            MatrixXi Route;

            Route= SURENA.GetLeftLegRoute();
            ioBody->link("LLeg_Foot_Link")->R();
            //            double u;
            //            double q = joint->q();
            //            double dq = (q - qold[i]) / dt;
            //            u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];//note:the joint velocity should not be zero
            //            qold[i] = q;
           //cout << "base link rot: \n" << tempIO->body()->link("LLeg_AnkleR_J6")->R() << endl;

            Matrix3d LeftAnkleAttitude=tempIO->body()->link("LLeg_AnkleR_J6")->R();
            double footPitch=acos( LeftAnkleAttitude(0,0));
            double footRoll=acos( LeftAnkleAttitude(1,1));
             //cout<<"pitch"<<footPitch<<endl;
            // cout << "base link rot: \n" << tempIO->currentTime();
            double dfootPitch=(footPitch-footPitchOld)/dt;
            double dfootRoll=(footRoll-footRollOld)/dt;
            MatrixXd LegAngle(1,6);
            if (StartTime<6) {
                LegAngle<<0 ,0 ,0*zStart,1*zStart, 0+1*(-0-footPitch)+0*(0-dfootPitch), 0*(0-footRoll)+0*(0-dfootRoll);
            }else {
               LegAngle<<0 ,0 ,0*zStart,1*zStart, 0+1*(-0-footPitch)+0.05*(0-dfootPitch), 0*(0-footRoll)+0*(0-dfootRoll);
            }


            SURENA.SetJointAngle(LegAngle,Route);
            footPitchOld=footPitch;
            footRollOld=footRoll;

            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

        }
        //*******************This part of code is for initialization of joints of the robot for walking**********************************
        if (startPhase==true && StartTime<=DurationOfStartPhase) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.95100,0.860;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<0,DurationOfStartPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpace.GetAccVelPos(CoefZStart,StartTime,0,5);
            zStart=outputZStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            PoseRFoot<<0,
                    -0.11500,
                    0.112000,
                    0,
                    0,
                    0;

            PoseLFoot<<0,
                    0.11500,
                    0.11200,
                    0,
                    0,
                    0;

            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }

        if (StartTime>DurationOfStartPhase && StartTime<(DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime)){

            bool walk=true;
            double m1;
            double m2;
            double m3;
            double m4;
            double m5;
            double m6;
            double m7;
            double m8;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;
            //qDebug()<<StartTime;
            MatrixXd P;
            if(walk==true){
                MatrixXd m=SURENAOffilneTaskSpace.AnkleTrajectory(SURENAOffilneTaskSpace.globalTime);
                m1=m(0,0);
                m2=m(1,0);
                m3=m(2,0);
                m4=m(3,0);
                m5=m(4,0);
                m6=m(5,0);
                m7=m(6,0);
                m8=m(7,0);

                P=SURENAOffilneTaskSpace.PelvisTrajectory (SURENAOffilneTaskSpace.globalTime);

                // just some samples for plotting!!!
                //                SURENAOffilneTaskSpace.CoMXVector.append(P(0,0));
                //                SURENAOffilneTaskSpace.timeVector.append(SURENAOffilneTaskSpace.globalTime);
                //                SURENAOffilneTaskSpace.LeftFootXTrajectory.append(m1);



                SURENAOffilneTaskSpace.globalTime=SURENAOffilneTaskSpace.globalTime+SURENAOffilneTaskSpace._timeStep;

                if (round(SURENAOffilneTaskSpace.globalTime)<=round(SURENAOffilneTaskSpace.MotionTime)){
                     //// hip roll modification
                    if (SURENAOffilneTaskSpace.DoubleSupport!=true) {
                        //For modifying the angle of roll during single support
                        RollTime=RollTime+SURENAOffilneTaskSpace._timeStep;
                        MinimumJerkInterpolation Coef;
                        MatrixXd RollAngle(1,3);
                        RollAngle<<0,0.0,0;
                        MatrixXd RollAngleVelocity(1,3);
                        RollAngleVelocity<<0.000,INFINITY,0.000;
                        MatrixXd RollAngleAcceleration(1,3);
                        RollAngleAcceleration<<0,INFINITY,0;


                        MatrixXd Time22(1,3);
                        Time22<<0,(SURENAOffilneTaskSpace.TSS/2),SURENAOffilneTaskSpace.TSS;
                        MatrixXd CoefRoll =Coef.Coefficient(Time22,RollAngle,RollAngleVelocity,RollAngleAcceleration);


                        //StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

                        MatrixXd outputRollAngle;
                        if (RollTime<=(SURENAOffilneTaskSpace.TSS/2)) {
                           outputRollAngle= SURENAOffilneTaskSpace.GetAccVelPos(CoefRoll.row(0),RollTime,0,5);
                            hipRoll=outputRollAngle(0,0);
                        }
                        else {
                           outputRollAngle =SURENAOffilneTaskSpace.GetAccVelPos(CoefRoll.row(1),RollTime,SURENAOffilneTaskSpace.TSS/2,5);
                            hipRoll=outputRollAngle(0,0);
                        }


                    }
                    else {
                        hipRoll=0;
                        RollTime=0;
                    }

//cout<<SURENAOffilneTaskSpace.TSS<<endl;
//cout<<RollTime<<endl;

                    PoseRoot<<P(0,0),
                            P(1,0),
                            P(2,0),
                            0,
                            0,
                            0;

                    PoseRFoot<<m5,
                            m6,
                            m7,
                            0,
                            -1*m8*(M_PI/180),
                            0;

                    PoseLFoot<<m1,
                            m2,
                            m3,
                            0,
                            -1*m4*(M_PI/180),
                            0;
                    //// hip roll modification
//                    if (SURENAOffilneTaskSpace.LeftSupport==true && SURENAOffilneTaskSpace.HipRollModification==true){

//                        SURENA.doIKhipRollModify("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot,-1*hipRoll);
//                        SURENA.doIKhipRollModify("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot,0);

//                    }

//                    else if (SURENAOffilneTaskSpace.HipRollModification==true && SURENAOffilneTaskSpace.LeftSupport!=true )  {
//                        SURENA.doIKhipRollModify("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot,0);
//                        SURENA.doIKhipRollModify("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot,0*hipRoll);
//                    }
                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

                    //R1=AnkleRollRight
                    //R2=AnklePitchRight
                    //R3=KneePitchRight
                    //R4=HipPitchRight
                    //R5=HipRollRight
                    //R6=HipYawRight

                    //L1=AnkleRollLeft
                    //L2=AnklePitchLeft
                    //L3=KneePitchLeft
                    //L4=HipPitchLeft
                    //L5=HipRollLeft
                    //L6=HipYawLeft

                    //R1=0*(1/2Pi)*(2304)*100  R2=1*-1*(1/2Pi)*(2304)*100   R3=2(1/2Pi)*(2304)*50   R4=3*-1(1/2Pi)*(2304)*80    L2=4*(1/2Pi)*(2304)*100    L1=5*(1/2Pi)*(2304)*100   L3=6*-1*(1/2Pi)*(2304)*50     L4=7*(1/2Pi)*(2304)*80   R6=8*-1*(1/2Pi)*(2304)*120   R5=9*(1/2Pi)*(2304)*120    L5=10*(1/2Pi)*(2304)*120    L6=11*-1*(1/2Pi)*(2304)*120

                }
            }
        }



        if (endPhase==true &&  StartTime>=(DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime) && StartTime<=DurationOfendPhase+DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.860,0.95100;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime,DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime+DurationOfendPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpace.GetAccVelPos(CoefZStart,StartTime,DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime,5);
            zStart=outputZStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            PoseRFoot<<0,
                    -0.11500,
                    0.112000,
                    0,
                    0,
                    0;

            PoseLFoot<<0,
                    0.11500,
                    0.11200,
                    0,
                    0,
                    0;

            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }
        //   const Vector3 zmp;

        //      zmp[0] = 0.5;
        //      zmp[1] = 0.5;
        //   zmp[2] = 0;
        //mil->setZmp(zmp);
        //SurenaItemBody->setZmp(zmp);
        if (StartTime<=7){
            //SURENA.doIKHand("Body",StartTime*500,"RArm_WristP_J7");//Waist_P_J2
        }
        Vector3 dv = waistAccelSensor->dv();
        // cout << "dv(" << dv(0) <<"," << dv(1) <<","<< dv(2) << ")" << endl << flush;

        links = SURENA.GetLinks();
        for(int  i = 1;i < 29;i++)
        {
            qref[i-1] = links[i].JointAngle;
            //cout << qref[i-1] <<" , "<<flush;
        }
        //cout<<ioBody->numJoints();

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double u;
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];//note:the joint velocity should not be zero
            qold[i] = q;
            joint->u() = u;
        }

        return true;

    }


//Plot.show();

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SURENA4Offline)
