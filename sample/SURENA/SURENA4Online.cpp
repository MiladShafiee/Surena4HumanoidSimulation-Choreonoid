
#include <cnoid/SimpleController>
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Headers/Robot.h"
#include"Headers/TaskSpace.h"
#include"Headers/taskspaceoffline.h"
#include <qmath.h>
#include <qwidget.h>
#include <QApplication>
#include<Headers/mainwindow.h>
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
QApplication app();
MainWindow plot;
class SURENA : public SimpleController
{

    BodyPtr ioBody;
    Robot SURENA;
    TaskSpace SURENATaskSpace;
    TaskSpaceOffline SURENAOffilneTaskSpace;
    QList<LinkM> links;
    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;


    vector<double> qref;
    vector<double> qold;
    int NStep;
    int var;
    double dt;
    double dimeet;
    double dimeet1;
    SimpleControllerIO* tempIO;
public:

    double StartTime=0;
    double WalkTime=0;
    double FinalTime=StartTime+WalkTime;
    double  DurationOfStartPhase=6;
    //FinalTime=StartTime+WalkTime;
double kindex;
    virtual bool initialize(SimpleControllerIO* io) override
    {                    SURENATaskSpace.RightFootXVelocity.append(0);
                         SURENATaskSpace.RightFootYVelocity.append(0);

                         SURENATaskSpace.RightFootXTrajectory.append(0);
                         SURENATaskSpace.RightFootYTrajectory.append(0);

                         SURENATaskSpace.RightFootXAcceleration.append(0);
                         SURENATaskSpace.RightFootYAcceleration.append(0);


                         SURENATaskSpace.LeftFootXVelocity.append(0);
                         SURENATaskSpace.LeftFootYVelocity.append(0);

                         SURENATaskSpace.LeftFootXTrajectory.append(0);
                         SURENATaskSpace.LeftFootYTrajectory.append(0);

                         SURENATaskSpace.LeftFootXAcceleration.append(0);
                         SURENATaskSpace.LeftFootYAcceleration.append(0);

                         SURENATaskSpace.LeftFootZTrajectory.append(SURENATaskSpace._ankleLength);
                         SURENATaskSpace.LeftFootZVelocity.append(0);
                         SURENATaskSpace.LeftFootZAcceleration.append(0);

                         SURENATaskSpace.RightFootZTrajectory.append(SURENATaskSpace._ankleLength);
                         SURENATaskSpace.RightFootZVelocity.append(0);
                         SURENATaskSpace.RightFootZacceleration.append(0);
        ioBody = io->body();
        dt = io->timeStep();
                             kindex=0;
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
        if(var < NStep){
            var++;
        }


        bool startPhase=true;
        PoseRoot.resize(6,1);
        PoseRFoot.resize(6,1);
        PoseLFoot.resize(6,1);

        //*******************This part of code is for initialization of joints of the robot for walking**********************************
        if (startPhase==true && StartTime<=DurationOfStartPhase) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.951,SURENATaskSpace._PelvisHeight;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;

            MatrixXd YPosition(1,2);
            YPosition<<0.000,SURENATaskSpace._yDCMOffsetNom;
            MatrixXd YVelocity(1,2);
            YVelocity<<0.000,0.000;
            MatrixXd YAcceleration(1,2);
            YAcceleration<<0.000,0.000;


            MatrixXd XPosition(1,2);
            XPosition<<0.00,SURENATaskSpace._xDCMOffsetNom;
            MatrixXd XVelocity(1,2);
            XVelocity<<0.000,0.000;
            MatrixXd XAcceleration(1,2);
            XAcceleration<<0.000,0.000;

//cout<<SURENATaskSpace._xDCMOffsetNom;
            MatrixXd Time(1,2);
            Time<<0,DurationOfStartPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);
            MatrixXd CoefYStart =Coef.Coefficient(Time,YPosition,YVelocity,YAcceleration);
            MatrixXd CoefXStart =Coef.Coefficient(Time,XPosition,XVelocity,XAcceleration);


            double zStart=0.951;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENATaskSpace._timeStep;

                MatrixXd outputZStart= SURENATaskSpace.GetAccVelPos(CoefZStart.topRows(1),StartTime,0,5);
                zStart=outputZStart(0,0);

                MatrixXd outputYStart= SURENATaskSpace.GetAccVelPos(CoefYStart.topRows(1),StartTime,0,5);
                yStart=outputYStart(0,0);

                MatrixXd outputXStart= SURENATaskSpace.GetAccVelPos(CoefXStart.topRows(1),StartTime,0,5);
                xStart=outputXStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            PoseRFoot<<0,
                    -0.1150,
                    0.112000,
                    0,
                    0,
                    0;

            PoseLFoot<<0,
                    0.1150,
                    0.112000,
                    0,
                    0,
                    0;
            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }

      if ((StartTime>DurationOfStartPhase || startPhase==false ) && StartTime<(33)){
          bool walk=true;
//qDebug()<<2;
 StartTime=StartTime+SURENATaskSpace._timeStep;

        //this part of code is for walking
       MatrixXd DCM;

                if(walk==true){
                    // tempIO->currentTime() < FinalTime
                    DCM=SURENATaskSpace.CoMDynamics(SURENATaskSpace._stepNumber,SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0),SURENATaskSpace._CoPDisplacement);
                    SURENATaskSpace._n=SURENATaskSpace._stepNumber-1;

                    if ((SURENATaskSpace._n%2)==0){
                        SURENATaskSpace._MinStepWidth = SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1)-0.40;
                        SURENATaskSpace._MaxStepWidth = SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1)-.10;

                    }
                    else {
                        SURENATaskSpace._MinStepWidth = SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1)+0.10;
                        SURENATaskSpace._MaxStepWidth = SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1)+0.40;
                    }

                    SURENATaskSpace._yDCMOffsetNom=(pow(-1,SURENATaskSpace._n))*(SURENATaskSpace._pelvisLength/(1+exp(SURENATaskSpace._omega*SURENATaskSpace._desiredStepDuration)))-SURENATaskSpace._desiredStepWidth/(1-exp(SURENATaskSpace._omega*SURENATaskSpace._desiredStepDuration));

                    SURENATaskSpace._MinStepLength=SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)+SURENATaskSpace.MinStepLength;
                    SURENATaskSpace._MaxStepLength=SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)+SURENATaskSpace.MaxStepLength;
                    SURENATaskSpace.Input=SURENATaskSpace.QPController(SURENATaskSpace._stepNumber,SURENATaskSpace._CoPDisplacement);
                    SURENATaskSpace._gama.conservativeResize(SURENATaskSpace._gama.rows()+1,NoChange);
                    SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)=SURENATaskSpace.Input(3,0);
                    SURENATaskSpace._stepDuration=(1/SURENATaskSpace._omega)*log(SURENATaskSpace.Input(3,0));
                    SURENATaskSpace.timeVector.append(SURENATaskSpace.globalTime);
//                    SURENATaskSpace.DCMXVector.append(DCM(0,1));
//                    SURENATaskSpace.EndCoPYVector.append( SURENATaskSpace.InitialCoP( SURENATaskSpace.InitialCoP.rows()-1,1));


//                    timeVector.append(globalTime);
                    SURENATaskSpace.DCMdsXVector.append(DCM(0,4));
                    SURENATaskSpace.DCMdsYVector.append(DCM(0,5));
                    SURENATaskSpace.CoMdsXVector.append(DCM(0,6));
                    SURENATaskSpace.CoMdsYVector.append(DCM(0,7));
                    SURENATaskSpace.DCMXVector.append(DCM(0,0));
                    SURENATaskSpace.DCMYVector.append(DCM(0,1));
                    SURENATaskSpace.CoMXVector.append(DCM(0,2));
                    SURENATaskSpace.CoMYVector.append(DCM(0,3));
                    SURENATaskSpace.EndCoPYVector.append( SURENATaskSpace.InitialCoP( SURENATaskSpace.InitialCoP.rows()-1,1));
                    SURENATaskSpace.EndCoPXVector.append( SURENATaskSpace.InitialCoP( SURENATaskSpace.InitialCoP.rows()-1,0));




                    if (SURENATaskSpace.globalTime==0) {
                        SURENATaskSpace.RightFootXTrajectory.append(SURENATaskSpace.InitialCoP(0,0));
                        SURENATaskSpace.RightFootXVelocity.append(0);
                        SURENATaskSpace.RightFootXAcceleration.append(0);

                        SURENATaskSpace.RightFootYTrajectory.append(-1*SURENATaskSpace.InitialCoP(0,1));
                        SURENATaskSpace.RightFootYVelocity.append(0);
                        SURENATaskSpace.RightFootYAcceleration.append(0);


                        SURENATaskSpace.RightFootZTrajectory.append(SURENATaskSpace._ankleLength);
                        SURENATaskSpace.RightFootZVelocity.append(0);
                        SURENATaskSpace.RightFootZacceleration.append(0);




                        SURENATaskSpace.LeftFootXTrajectory.append(SURENATaskSpace.InitialCoP(0,0));
                        SURENATaskSpace.LeftFootXVelocity.append(0);
                        SURENATaskSpace.LeftFootXAcceleration.append(0);


                        SURENATaskSpace.LeftFootYTrajectory.append(1*SURENATaskSpace.InitialCoP(0,1));
                        SURENATaskSpace.LeftFootYVelocity.append(0);
                        SURENATaskSpace.LeftFootYAcceleration.append(0);


                        SURENATaskSpace.LeftFootZTrajectory.append(SURENATaskSpace._ankleLength);
                        SURENATaskSpace.LeftFootZVelocity.append(0);
                        SURENATaskSpace.LeftFootZAcceleration.append(0);
                    }
                    else {
                        kindex=kindex+1;
                        MatrixXd tempRightFoot= SURENATaskSpace.RightFoot();

                        SURENATaskSpace.RightFootXTrajectory.append(tempRightFoot(0,0));
                        SURENATaskSpace.RightFootXVelocity.append(tempRightFoot(0,1));
                        SURENATaskSpace.RightFootXAcceleration.append(tempRightFoot(0,2));

                        SURENATaskSpace.RightFootYTrajectory.append(tempRightFoot(0,3));
                        SURENATaskSpace.RightFootYVelocity.append(tempRightFoot(0,4));
                        SURENATaskSpace.RightFootYAcceleration.append(tempRightFoot(0,5));


                        SURENATaskSpace.RightFootZTrajectory.append(tempRightFoot(0,6));
                        SURENATaskSpace.RightFootZVelocity.append(tempRightFoot(0,7));
                        SURENATaskSpace.RightFootZacceleration.append(tempRightFoot(0,8));


                        MatrixXd tempLeftFoot= SURENATaskSpace.LeftFoot();

                        SURENATaskSpace.LeftFootXTrajectory.append(tempLeftFoot(0,0));
                        SURENATaskSpace.LeftFootXVelocity.append(tempLeftFoot(0,1));
                        SURENATaskSpace.LeftFootXAcceleration.append(tempLeftFoot(0,2));

                        SURENATaskSpace. LeftFootYTrajectory.append(tempLeftFoot(0,3));
                        SURENATaskSpace.LeftFootYVelocity.append(tempLeftFoot(0,4));
                        SURENATaskSpace.LeftFootYAcceleration.append(tempLeftFoot(0,5));


                        SURENATaskSpace.LeftFootZTrajectory.append(tempLeftFoot(0,6));
                        SURENATaskSpace.LeftFootZVelocity.append(tempLeftFoot(0,7));
                        SURENATaskSpace.LeftFootZAcceleration.append(tempLeftFoot(0,8));
                    }



                    SURENATaskSpace.time=SURENATaskSpace.time+SURENATaskSpace._timeStep;
                    SURENATaskSpace.globalTime=SURENATaskSpace.globalTime+SURENATaskSpace._timeStep;

                    //this part is for double support
                    if (SURENATaskSpace.time>=0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration && SURENATaskSpace.time<SURENATaskSpace._stepDuration-0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration){
                        SURENATaskSpace.timeDs=0;
                        SURENATaskSpace._DCMxInitialDoubleSupport=SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0)+exp(-0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration*SURENATaskSpace._omega)*(((SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)- SURENATaskSpace._CoPDisplacement(0,0))*(1-SURENATaskSpace._xdeltaNom)/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(0.500*exp(-SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-0.500*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+(SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)- SURENATaskSpace._CoPDisplacement(0,0))*((SURENATaskSpace._xdeltaNom-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)))/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(1-exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+SURENATaskSpace.InitialDCM(SURENATaskSpace._stepNumber-1,0)*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0));
                        SURENATaskSpace._DCMxFinalDoubleSupport=SURENATaskSpace.Input(0,0)+exp(0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration*SURENATaskSpace._omega)*(((SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)- SURENATaskSpace._CoPDisplacement(0,0))*(1-SURENATaskSpace._xdeltaNom)/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(0.500*exp(-SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-0.500*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+(SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)- SURENATaskSpace._CoPDisplacement(0,0))*((SURENATaskSpace._xdeltaNom-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)))/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(1-exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+SURENATaskSpace.InitialDCM(SURENATaskSpace._stepNumber-1,0)*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-SURENATaskSpace.Input(0,0));
                        SURENATaskSpace._DCMVelxInitialDoubleSupport=(SURENATaskSpace._DCMxInitialDoubleSupport-SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0))*SURENATaskSpace._omega;
                        SURENATaskSpace._DCMVelxFinalDoubleSupport=SURENATaskSpace._omega*exp(0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration*SURENATaskSpace._omega)*(((SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)- SURENATaskSpace._CoPDisplacement(0,0))*(1-SURENATaskSpace._xdeltaNom)/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(0.500*exp(-SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-0.500*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+(SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,0)- SURENATaskSpace._CoPDisplacement(0,0))*((SURENATaskSpace._xdeltaNom-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)))/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(1-exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+SURENATaskSpace.InitialDCM(SURENATaskSpace._stepNumber-1,0)*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-SURENATaskSpace.Input(0,0));

                        SURENATaskSpace._DCMyInitialDoubleSupport=SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1)+exp(-0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration*SURENATaskSpace._omega)*(((SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,1)- SURENATaskSpace._CoPDisplacement(1,0))*(1-SURENATaskSpace._ydeltaNom)/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,1))))*(0.500*exp(-SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-0.500*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+(SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,1)- SURENATaskSpace._CoPDisplacement(1,0))*((SURENATaskSpace._xdeltaNom-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)))/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(1-exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+SURENATaskSpace.InitialDCM(SURENATaskSpace._stepNumber-1,1)*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1));
                        SURENATaskSpace._DCMyFinalDoubleSupport=SURENATaskSpace.Input(4,0)+exp(0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration*SURENATaskSpace._omega)*(((SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,1)- SURENATaskSpace._CoPDisplacement(1,0))*(1-SURENATaskSpace._xdeltaNom)/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(0.500*exp(-SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-0.500*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+(SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,1)- SURENATaskSpace._CoPDisplacement(1,0))*((SURENATaskSpace._xdeltaNom-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)))/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(1-exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+SURENATaskSpace.InitialDCM(SURENATaskSpace._stepNumber-1,1)*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-SURENATaskSpace.Input(4,0));
                        SURENATaskSpace._DCMVelyInitialDoubleSupport=(SURENATaskSpace._DCMyInitialDoubleSupport-SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1))*SURENATaskSpace._omega;
                        SURENATaskSpace._DCMVelyFinalDoubleSupport=SURENATaskSpace._omega*exp(0.5*SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration*SURENATaskSpace._omega)*(((SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,1)- SURENATaskSpace._CoPDisplacement(1,0))*(1-SURENATaskSpace._xdeltaNom)/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(0.500*exp(-SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-0.500*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+(SURENATaskSpace.InitialCoP(SURENATaskSpace._stepNumber-1,1)- SURENATaskSpace._CoPDisplacement(1,0))*((SURENATaskSpace._xdeltaNom-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0)))/(1-(1/SURENATaskSpace._gama(SURENATaskSpace._gama.rows()-1,0))))*(1-exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration))+SURENATaskSpace.InitialDCM(SURENATaskSpace._stepNumber-1,1)*exp(SURENATaskSpace._omega*SURENATaskSpace._stepDuration)-SURENATaskSpace.Input(4,0));

                        MatrixXd ttimee(4 ,4);
                        double T=SURENATaskSpace._doubleSupportRatio*SURENATaskSpace._stepDuration;


                        ttimee<<2/pow(T,3), 1/pow(T,2), -2/pow(T,3), 1/pow(T,2),
                                -3/pow(T,2), -2/(T) , 3/pow(T,2) , -1/(T) ,
                                0      ,    1   ,       0 ,    0   ,
                                1      ,    0   ,       0 ,    0   ;
                        MatrixXd xdcm(4,1);
                        MatrixXd ydcm(4,1);
                        xdcm<<SURENATaskSpace._DCMxInitialDoubleSupport,SURENATaskSpace._DCMVelxInitialDoubleSupport,SURENATaskSpace._DCMxFinalDoubleSupport,SURENATaskSpace._DCMVelxFinalDoubleSupport;
                        SURENATaskSpace._coefXDCMDoubleSupport=ttimee*xdcm;

                        ydcm<<SURENATaskSpace._DCMyInitialDoubleSupport,SURENATaskSpace._DCMVelyInitialDoubleSupport,SURENATaskSpace._DCMyFinalDoubleSupport,SURENATaskSpace._DCMVelyFinalDoubleSupport;
                         SURENATaskSpace._coefYDCMDoubleSupport=ttimee*ydcm;
                    }


                    if (SURENATaskSpace.time>SURENATaskSpace._stepDuration) {
                        //if stepduration is nan add please
                        SURENATaskSpace.time=0;
                        SURENATaskSpace.InitialDCM.conservativeResize(SURENATaskSpace.InitialDCM.rows()+1,NoChange);
                        SURENATaskSpace.InitialDCM(SURENATaskSpace.InitialDCM.rows()-1,0)=DCM(0,0);
                        SURENATaskSpace.InitialDCM(SURENATaskSpace.InitialDCM.rows()-1,1)=DCM(0,1);


                        SURENATaskSpace.InitialCoM.conservativeResize(SURENATaskSpace.InitialCoM.rows()+1,NoChange);
                        SURENATaskSpace.InitialCoM(SURENATaskSpace.InitialCoM.rows()-1,0)=DCM(0,2);
                        SURENATaskSpace.InitialCoM(SURENATaskSpace.InitialCoM.rows()-1,1)=DCM(0,3);

                        SURENATaskSpace.InitialCoMds.conservativeResize(SURENATaskSpace.InitialCoMds.rows()+1,NoChange);
                        SURENATaskSpace.InitialCoMds(SURENATaskSpace.InitialCoMds.rows()-1,0)=DCM(0,6);
                        SURENATaskSpace.InitialCoMds(SURENATaskSpace.InitialCoMds.rows()-1,1)=DCM(0,7);


                        SURENATaskSpace.InitialCoP.conservativeResize(SURENATaskSpace.InitialCoP.rows()+1,NoChange);
                        SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0)=SURENATaskSpace.Input(0,0);
                        SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1)=SURENATaskSpace.Input(4,0);

                        SURENATaskSpace._stepNumber=SURENATaskSpace._stepNumber+1;
                        SURENATaskSpace._xdeltaMax =((2*SURENATaskSpace._XCoPDisplacementSS)/SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0))+1;
                        SURENATaskSpace._xdeltaMin =((2*SURENATaskSpace._XCoPDisplacementSS)/SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0))+1;
                        SURENATaskSpace._xdeltaNom =((2*SURENATaskSpace._XCoPDisplacementSS)/SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,0))+1;


                        SURENATaskSpace._ydeltaMax =((2*SURENATaskSpace._YCoPDisplacementSS)/SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1))+1;
                        SURENATaskSpace._ydeltaMin =((2*SURENATaskSpace._YCoPDisplacementSS)/SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1))+1;
                        SURENATaskSpace._ydeltaNom =((2*SURENATaskSpace._YCoPDisplacementSS)/SURENATaskSpace.InitialCoP(SURENATaskSpace.InitialCoP.rows()-1,1))+1;
                    }

                    PoseRoot.resize(6,1);
                    PoseRoot<<DCM(0,6),
                            DCM(0,7),
                            SURENATaskSpace._PelvisHeight,
                            0,
                            0,
                            0;



                    PoseRFoot.resize(6,1);
                    PoseRFoot<<SURENATaskSpace.RightFootXTrajectory[SURENATaskSpace.RightFootXTrajectory.length()-1],
                               SURENATaskSpace.RightFootYTrajectory[SURENATaskSpace.RightFootYTrajectory.length()-1],
                               SURENATaskSpace.RightFootZTrajectory[SURENATaskSpace.RightFootZTrajectory.length()-1],
                            0,
                            0,
                            0;
                    PoseLFoot.resize(6,1);
                    PoseLFoot<<SURENATaskSpace.LeftFootXTrajectory[SURENATaskSpace.LeftFootXTrajectory.length()-1],
                            SURENATaskSpace.LeftFootYTrajectory[SURENATaskSpace.LeftFootYTrajectory.length()-1],
                            SURENATaskSpace.LeftFootZTrajectory[SURENATaskSpace.LeftFootZTrajectory.length()-1],
                            0,
                            0,
                            0;

                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

        //            PoseRoot.resize(6,1);
        //            PoseRoot<<0,
        //                    0,
        //                    0.8,
        //                    0,
        //                    0,
        //                    0;



        //            PoseRFoot.resize(6,1);
        //            PoseRFoot<<0,
        //                       -0.115,
        //                       SURENATaskSpace.RightFootZTrajectory[SURENATaskSpace.RightFootZTrajectory.length()-1],
        //                    0,
        //                    0,
        //                    0;
        //            PoseLFoot.resize(6,1);
        //            PoseLFoot<<0,
        //                    0.115,
        //                    SURENATaskSpace.LeftFootZTrajectory[SURENATaskSpace.LeftFootZTrajectory.length()-1],
        //                    0,
        //                    0,
        //                    0;
                }
        }
                // Comment for deactive walking////////////////////////////////////////////////////





        links = SURENA.GetLinks();

        for(int  i = 1;i < 13;i++)
        {

            qref[i-1] = links[i].JointAngle;

        }

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double u;
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];//note:the joint velocity should not be zero
            qold[i] = q;
            joint->u() = u;
        }

        dimeet1= tempIO->currentTime();
       //out<<SURENATaskSpace.globalTime<<endl;
        if (SURENATaskSpace.globalTime>1 && SURENATaskSpace.globalTime<1.001){
         //cout<<SURENATaskSpace.globalTime<<endl;
doplot();
        }
        return true;

//doplot();
    }


    void doplot(){
  // plot.show();

  //  plot.Plot(SURENATaskSpace);
    //return app.exec();
}
    };

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SURENA)
