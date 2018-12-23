
#include <cnoid/SimpleController>
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Headers/Robot.h"
#include"Headers/TaskSpace.h"
#include"Headers/taskspaceonline1.h"
#include <qmath.h>
#include <cstring>
#include <cnoid/BodyLoader>
#include "src/BodyPlugin//BodyItem.h"
#include <cnoid/Sensor>
#include <QApplication>
#include<Headers/mainwindow.h>
#include <qwidget.h>
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

QApplication ab();



bool RFT;
bool LFT;
bool KRtemp;// for online
bool KLtemp;// for online
bool aState;
bool bState;
bool cState;
bool dState;
bool LeftFootLanded;
bool RightFootLanded;
bool FullContactDetected;

//bool indexLastDS;





MainWindow miladplot;
class SURENA4Online1 : public SimpleController
{


    BodyPtr ioBody;
    Robot SURENA;
    TaskSpaceOnline1 SURENAOnlineTaskSpace1;

    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;
    //BodyItemPtr SurenaItemBody;
    //BodyItem* mil;
    vector<double> qref;
    vector<double> qold;

    double dt;
    SimpleControllerIO* tempIO;

public:
    double StartTime=0;
    double WalkTime=0;
    double DurationOfStartPhase=6;
    double DurationOfendPhase=6;
    QList<LinkM> links;
  bool indexLastDS=true;

    BodyLoader bodyloader;



    virtual bool initialize(SimpleControllerIO* io) override
    {

        KRtemp=false;
        KLtemp=false;


        links=SURENA.GetLinks();


        ioBody = io->body();

        SURENAOnlineTaskSpace1.StepNumber=1;

        dt = io->timeStep();
        tempIO=io;

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
            StartTime=StartTime+SURENAOnlineTaskSpace1._timeStep;

            MatrixXd outputZStart= SURENAOnlineTaskSpace1.GetAccVelPos(CoefZStart,StartTime,0,5);
            zStart=outputZStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            SURENAOnlineTaskSpace1.CoMXVector.append(0);
            SURENAOnlineTaskSpace1.timeVector.append(SURENAOnlineTaskSpace1.globalTime);

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


        if (endPhase==true &&  StartTime>=(DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime) && StartTime<=DurationOfendPhase+DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.860,0.95100;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;

            MatrixXd Time(1,2);
            Time<<DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime,DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime+DurationOfendPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOnlineTaskSpace1._timeStep;

            MatrixXd outputZStart= SURENAOnlineTaskSpace1.GetAccVelPos(CoefZStart,StartTime,DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime,5);
            zStart=outputZStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            SURENAOnlineTaskSpace1.CoMXVector.append(0);
            SURENAOnlineTaskSpace1.timeVector.append(SURENAOnlineTaskSpace1.globalTime);

            PoseRFoot<<0,
                    -0.11500,
                    SURENAOnlineTaskSpace1.currentRightFootZ,
                    0,
                    0,
                    0;

            PoseLFoot<<0,
                    0.11500,
                    SURENAOnlineTaskSpace1.currentLeftFootZ,
                    0,
                    0,
                    0;

            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }


        int NumberOfTimeStep=(SURENAOnlineTaskSpace1.Tc/SURENAOnlineTaskSpace1._timeStep)+1;
        int NumberOfTimeStepDS=(SURENAOnlineTaskSpace1.TDs/SURENAOnlineTaskSpace1._timeStep)+1;
        int NumberOfTimeStepEnd=(SURENAOnlineTaskSpace1.TEnd/SURENAOnlineTaskSpace1._timeStep)+1;


        //---------------------------------------------------------------------------------//
        //--------------------------------main cycle of walking----------------------------//
        //---------------------------------------------------------------------------------//
        if (StartTime>DurationOfStartPhase && StartTime<(DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime)){

            bool walk=true;
            double m1;
            double m2;
            double m3;
            double m4;
            double m5;
            double m6;
            double m7;
            double m8;
            StartTime=StartTime+SURENAOnlineTaskSpace1._timeStep;

            MatrixXd P;
            if(walk==true){

                if ((SURENAOnlineTaskSpace1.StepNumber==1) && (SURENAOnlineTaskSpace1.localTiming>=SURENAOnlineTaskSpace1.TStart) ) {
                    SURENAOnlineTaskSpace1.localTiming=0.0020000000000;//0.001999999999000000;
                    SURENAOnlineTaskSpace1.localtimingInteger=1;
                    SURENAOnlineTaskSpace1.StepNumber=SURENAOnlineTaskSpace1.StepNumber+1;
                    KLtemp=false;
                }



                else if ((SURENAOnlineTaskSpace1.localtimingInteger>=NumberOfTimeStep) &&   (SURENAOnlineTaskSpace1.StepNumber>1    &&   SURENAOnlineTaskSpace1.StepNumber<(SURENAOnlineTaskSpace1.NStep+2))) {


                    SURENAOnlineTaskSpace1.StepNumber=SURENAOnlineTaskSpace1.StepNumber+1;

                    if (SURENAOnlineTaskSpace1.StepNumber==SURENAOnlineTaskSpace1.NStep+2) {
                        SURENAOnlineTaskSpace1.localTiming=0.0020000000000;
                        SURENAOnlineTaskSpace1.localtimingInteger=1;
                    }
                    else {
                        SURENAOnlineTaskSpace1.localtimingInteger=1;
                        SURENAOnlineTaskSpace1.localTiming=0.0020000000000;//0.00199999999;
                    }


                    if ((SURENAOnlineTaskSpace1.StepNumber%2)==0) {
                        KLtemp=false;
                    }
                    else {
                        KRtemp=false;
                    }


                }

                else if (indexLastDS==true && SURENAOnlineTaskSpace1.localTiming>=SURENAOnlineTaskSpace1.TDs && SURENAOnlineTaskSpace1.StepNumber==SURENAOnlineTaskSpace1.NStep+2) {
                    SURENAOnlineTaskSpace1.localTiming=0.00200000000000;
                    SURENAOnlineTaskSpace1.localtimingInteger=1;
                    indexLastDS=false;
                    KLtemp=false;
                }

                else if (indexLastDS==false && SURENAOnlineTaskSpace1.localTiming>(0.5*SURENAOnlineTaskSpace1.TEnd)) {
                    KRtemp=false;
                    //cout<<SURENAOnlineTaskSpace1.localTiming<<endl;
                    /*SURENAOnlineTaskSpace1.localTiming=0.00200000000000;
                    SURENAOnlineTaskSpace1.localtimingInteger=1*/;
                }
                //                links=SURENA.GetLinks();


                if (KLtemp==false) {
                    KLtemp=true;
                    SURENAOnlineTaskSpace1.currentLeftFootX1=links[12].PositionInWorldCoordinate(0);
                    SURENAOnlineTaskSpace1.currentLeftFootZ=links[12].PositionInWorldCoordinate(2);
                }




                if (KRtemp==false) {
                    KRtemp=true;
                    SURENAOnlineTaskSpace1.currentRightFootX1=links[6].PositionInWorldCoordinate(0);
                    SURENAOnlineTaskSpace1.currentRightFootZ=links[6].PositionInWorldCoordinate(2);
                }



                MatrixXd m=SURENAOnlineTaskSpace1.AnkleTrajectory(SURENAOnlineTaskSpace1.globalTime,SURENAOnlineTaskSpace1.StepNumber,SURENAOnlineTaskSpace1.localTiming,RFT,LFT,indexLastDS);
                m1=m(0,0);
                m2=m(1,0);
                m3=m(2,0);
                m4=m(3,0);
                m5=m(4,0);
                m6=m(5,0);
                m7=m(6,0);
                m8=m(7,0);

                P=SURENAOnlineTaskSpace1.PelvisTrajectory(SURENAOnlineTaskSpace1.globalTime,SURENAOnlineTaskSpace1.StepNumber,SURENAOnlineTaskSpace1.localTiming,indexLastDS);
                SURENAOnlineTaskSpace1.globalTime=SURENAOnlineTaskSpace1.globalTime+SURENAOnlineTaskSpace1._timeStep;
                SURENAOnlineTaskSpace1.localTiming=SURENAOnlineTaskSpace1.localTiming+SURENAOnlineTaskSpace1._timeStep;

                SURENAOnlineTaskSpace1.localtimingInteger= SURENAOnlineTaskSpace1.localtimingInteger+1;
                if (round(SURENAOnlineTaskSpace1.globalTime)<=round(SURENAOnlineTaskSpace1.MotionTime)){


                    PoseRoot<<P(0,0),
                            P(1,0),
                            P(2,0),
                            0,
                            0,
                            0;

                    SURENAOnlineTaskSpace1.CoMXVector.append(m5);
                    SURENAOnlineTaskSpace1.timeVector.append(SURENAOnlineTaskSpace1.globalTime);

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

                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.ForwardKinematic(1);
                }
            }
        }


       //cout<<SURENAOnlineTaskSpace1.MotionTime<<flush<<endl;

        links = SURENA.GetLinks();



        for(int  i = 1;i<29;i++)
        {
            qref[i-1] = links[i].JointAngle;
            //cout << qref[i-1] <<" , "<<flush;
        }

        if (SURENAOnlineTaskSpace1.globalTime>(SURENAOnlineTaskSpace1.MotionTime-2*SURENAOnlineTaskSpace1._timeStep) && SURENAOnlineTaskSpace1.globalTime<(SURENAOnlineTaskSpace1.MotionTime-SURENAOnlineTaskSpace1._timeStep)){
            doplot();
        }

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double u;
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];//note:the joint velocity should not be zero
            qold[i] = q;
            joint->u() = u;
            //cout<<SURENAOnlineTaskSpace1.globalTime<<endl;

        }

        return true;

    }

    void doplot(){
        miladplot.Plot(SURENAOnlineTaskSpace1);
    }

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SURENA4Online1)
