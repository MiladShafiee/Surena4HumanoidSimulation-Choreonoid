#include "Headers/TaskSpace.h"

TaskSpace::TaskSpace()
{

    _XDesiredVelocityKH=2;//km/h
    _XDesiredVelocity= _XDesiredVelocityKH/3.6;//m/s
    _YDesiredVelocity=0;// desired velocity in Y direction is not tested
    SetStepTimingGain(1);
    SetStepPositionXGain(1);
    SetDCMOffsetXGain(10000);
    SetDeltaXGain(1);
    _n;//number of step for finding that which foot moves and which foot is support
    //time=5.5   vel=0.027   step=0.15
    SetStepPositionYGain(1);
    SetDCMOffsetYGain(10000);
    SetDeltaYGain(1);
    time=0;


    _doubleSupportRatio=0.2;


    globalTime=0;//current time of simulation
    _timeStep=0.002;
    //note when the min step duration becomes zero, _maxXDCMOffset will become inf, however by increasing the dcm offset gain we will find some wrong plot

    _auxilliaryStepLength=0.3;//deiredstepLength that we aim to be a nominal value of step length
    _offsetOfStepLength=0.1;//For generating the constraint of step length
    _MinStepLength = _auxilliaryStepLength-_offsetOfStepLength;
    _MaxStepLength = _auxilliaryStepLength+_offsetOfStepLength;

    _auxilliaryStepDuration=_auxilliaryStepLength/(_XDesiredVelocity);
    _offsetOfStepDuration=1;//For generating the constraint of step duration
    _MinStepDuration = _auxilliaryStepDuration-_offsetOfStepDuration;
    _MaxStepDuration = _auxilliaryStepDuration+_offsetOfStepDuration;

     MinStepLength = _auxilliaryStepLength-_offsetOfStepLength;
    MaxStepLength = _auxilliaryStepLength+_offsetOfStepLength;

    _MinStepWidth=0.0;
    _MaxStepWidth=0.0;
    _xdeltaMax=1;//consider that now we don't have any double support or single support the zmp is constant
    _xdeltaMin=1;
    _xdeltaNom=1;
    _ydeltaMax=1;
    _ydeltaMin=1;
    _ydeltaNom=1;


    _mass=74.24;
    _gravity=9.80000;
    _PelvisHeight=0.83;
    _pelvisLength=0.23;
    _ankleLength=1*0.112;
    _omega=sqrt(_gravity/_PelvisHeight);
    //    _maxXDCMOffset= _MaxStepLength/(exp(_omega*_MinStepDuration)-1);
    //    _minXDCMOffset= _MinStepLength/(exp(_omega*_MaxStepDuration)-1);
    GetDesiredParameter();

    _desiredMAxFootHeight=0.03+_ankleLength;
    _boundMaxFootHeight=_desiredMAxFootHeight+0.05;

    _DCM.resize(1,2);
    _DCM.fill(0);

    _DCMds.resize(1,2);
    _DCMds.fill(0);

    _CoM.resize(1,2);
    _CoM.fill(0);

    _CoMds.resize(1,2);
    _CoMds.fill(0);


    InitialCoP.resize(1,2);
    InitialCoP<<0,0.115;
    _CoPHeel.resize(1,2);
    _CoPHeel<<0,0;

    _CoPToe.resize(1,2);
    _CoPToe<<0,0;

    _nSteps=6;
    _stepNumber=1;
    // _stepDuration=2;
    // _xDCMOffsetNom=0.1456;
    _xDCMOffsetNom=_desiredStepLength/(exp(_omega*_desiredStepDuration)-1);

    //double mmmm;
    //mmmm=InitialCoP(0,1)+-1*(_pelvisLength/(1+exp(_omega*_desiredStepDuration)));
    _yDCMOffsetNom=InitialCoP(0,1)+(-1)*(_pelvisLength/(1+exp(_omega*_desiredStepDuration)))-_desiredStepWidth/(1-exp(_omega*_desiredStepDuration));


    _XCoPDisplacementSS=0.00000;
    _YCoPDisplacementSS=0.00000;

    _XError=0;
    _YError=0;

    _error.resize(2,1);
    _error<<_XError,_YError;

    _gama.resize(1,2);
    _gama<<(exp(1*_omega*_desiredStepDuration)),0;

    _yDCMOffset=_yDCMOffsetNom;
    _xDCMOffset=_xDCMOffsetNom;

    InitialDCM.resize(1,2);
    InitialDCM<<_xDCMOffsetNom,_yDCMOffsetNom;

    InitialCoM.resize(1,2);
    InitialCoM<<_xDCMOffsetNom,_yDCMOffsetNom;

    InitialCoMds.resize(1,2);
    InitialCoMds<<_xDCMOffsetNom,_yDCMOffsetNom;

    _CoPDisplacement.resize(2,1);
    _CoPDisplacement<<_XCoPDisplacementSS,_YCoPDisplacementSS;



//    RightFootXVelocity.append(0);
//    RightFootYVelocity.append(0);
//    RightFootXTrajectory.append(0);
//    RightFootYTrajectory.append(0);
//    RightFootXAcceleration.append(0);
//    RightFootYAcceleration.append(0);
//    RightFootZTrajectory.append(_ankleLength);
//    RightFootZVelocity.append(0);
//    RightFootZacceleration.append(0);

//    LeftFootXVelocity.append(0);
//    LeftFootYVelocity.append(0);
//    LeftFootXTrajectory.append(0);
//    LeftFootYTrajectory.append(0);
//    LeftFootXAcceleration.append(0);
//    LeftFootYAcceleration.append(0);
//    LeftFootZTrajectory.append(_ankleLength);
//    LeftFootZVelocity.append(0);
//    LeftFootZAcceleration.append(0);

}






MatrixXd TaskSpace::QPController(int StepNumber,MatrixXd CoPDisplacementSS){
    MatrixXd H(7,7);
    MatrixXd F(1,7);
    MatrixXd Aiq(10,7);
    MatrixXd biq(10,1);
    MatrixXd Aeq(2,7);
    MatrixXd beq(1,2);
    double alphaa=0.000000;
    double XCoPDisplacementSS;
    double YCoPDisplacementSS;
    XCoPDisplacementSS=CoPDisplacementSS(0,0);
    YCoPDisplacementSS=CoPDisplacementSS(1,0);

    H<<_alpha1x,0,0,0,0,0,0,
            0,_alpha2x,0,0,0,0,0,
            0,0,_alpha3x,0,0,0,0,
            0,0,0,_alphaT,0,0,0,
            0,0,0,0,_alpha1y,0,0,
            0,0,0,0,0,_alpha2y,0,
            0,0,0,0,0,0,_alpha3y;

    F<<-1*_alpha1x*_desiredStepLength,
            -1*_alpha2x*_xdeltaNom,
            -1*_alpha3x*_xDCMOffsetNom,
            -1*_alphaT*_sigmaDesired,
            -1*_alpha1y*_desiredStepWidth,
            -1*_alpha2y*_ydeltaNom,
            -1*_alpha3y*_yDCMOffsetNom;

    Aiq<<1,0,0,0,0,0,0,
            -1,0,0,0,0,0,0,
            0,+1,0,0,0,0,0,
            0,-1,0,0,0,0,0,
            0,0,0,+1,0,0,0,
            0,0,0,-1,0,0,0,
            0,0,0,0,+1,0,0,
            0,0,0,0,-1,0,0,
            0,0,0,0,0,+1,0,
            0,0,0,0,0,-1,0;

    biq<<_MaxStepLength,
            -1*_MinStepLength,
            _xdeltaMax,
            -1*_xdeltaMin,
            exp(_omega*_MaxStepDuration),
            -1*exp(_omega*_MinStepDuration),
            1*_MaxStepWidth,
            -1*_MinStepWidth,
            _ydeltaMax,
            -1*_ydeltaMin;

    if (StepNumber==1){
        Aeq<<1,-(InitialCoP(StepNumber-1,0)-XCoPDisplacementSS),1,((InitialCoP(StepNumber-1,0)-XCoPDisplacementSS)-_XError*exp(-_omega*time)-InitialDCM(StepNumber-1,0)),0,0,0,
                0,0,0,((InitialCoP(StepNumber-1,1)-YCoPDisplacementSS)-_YError*exp(-_omega*time)-InitialDCM(StepNumber-1,1)),1,-(InitialCoP(StepNumber-1,1)-YCoPDisplacementSS),1;
        beq<<0,0;
    }
    else{
        Aeq<<1,-InitialCoP(StepNumber-1,0),1,((InitialCoP(StepNumber-1,0))-_XError*exp(-_omega*time)-InitialDCM(StepNumber-1,0)),0,0,0,
                0,0,0,(InitialCoP(StepNumber-1,1)-_YError*exp(-_omega*time)-InitialDCM(StepNumber-1,1)),1,-InitialCoP(StepNumber-1,1),1;
        beq<<0,0;
    }

    VectorXd sol(H.rows());
    MatrixXd HTemp2 = H;

    MatrixXd CE(7,2);
    VectorXd ce0(2);
    CE<<Aeq.transpose();
    ce0<<beq(0,0),beq(0,1);

    VectorXd g(Map<VectorXd>(F.data(),F.cols()*F.rows()));

    double optCosts;
    VectorXd  ci0=biq;
    MatrixXd CIT=Aiq.transpose();


    QElapsedTimer timer;
    qint64 nanoSec;
    timer.start();//----------------------------------------------------------------


    optCosts = solve_quadprog(HTemp2, g, -1*CE,ce0 ,-1*CIT , ci0, sol);
    if(optCosts==std::numeric_limits<double>::infinity()){
        qDebug("Quadratic Programming for DCM Planning failed!");
    }
    MatrixXd ux = sol;
    nanoSec = timer.nsecsElapsed();
    // qDebug()<< nanoSec*pow(10,-6);
    return ux;

    //                //min  0.5 * x H x + g x
    //                //s.t. CE^T x + ce0 = 0
    //                //     CI^T x + ci0 >= 0
    //                // Since you cannot explicitly express bilateral constraints
    //                // consider only lower bounds
}






MatrixXd TaskSpace::CoMDynamics(int stepNumber, double GamaX, MatrixXd CoPDisplacementSS){
    double XCoPDisplacementSS;
    double YCoPDisplacementSS;
    //double doubleSupportRatio=0.2;
    double stepDuration=(1/_omega)*log(GamaX);
    MatrixXd TempDCM;
    XCoPDisplacementSS=CoPDisplacementSS(0,0);
    YCoPDisplacementSS=CoPDisplacementSS(1,0);
    _CoM.conservativeResize(_CoM.rows()+1,NoChange);
    _DCM.conservativeResize(_DCM.rows()+1,NoChange);
    _DCMds.conservativeResize(_DCMds.rows()+1,NoChange);
    if (stepNumber==1){//in the start of walking we dont define the double support !!here!!!! ...
        if ( time>=(stepDuration-0.5*_doubleSupportRatio*stepDuration)) {
            timeDs=timeDs+_timeStep;
            //cout<<timeDs<<endl;
            //cout<<stepDuration<<endl;
            MatrixXd time2(1,4);
            time2<<pow(timeDs,3),pow(timeDs,2),timeDs,1;
            MatrixXd tempx=time2*_coefXDCMDoubleSupport;
            MatrixXd tempy=time2*_coefYDCMDoubleSupport;
            _DCMds(_DCMds.rows()-1,0)=tempx(0,0);
            _DCMds(_DCMds.rows()-1,1)=tempy(0,0);
            _DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);
            // cout<<tempy(0,0)<<endl;
        }
        else {
            _DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);
            _DCMds(_DCMds.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCMds(_DCMds.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);
        }
    }
    else {
        if (time<=0.5*_doubleSupportRatio*stepDuration ){
            timeDs=timeDs+_timeStep;
            MatrixXd time2(1,4);
            time2<<pow(timeDs,3),pow(timeDs,2),timeDs,1;
            MatrixXd tempx=time2*_coefXDCMDoubleSupport;
            MatrixXd tempy=time2*_coefYDCMDoubleSupport;
            _DCMds(_DCMds.rows()-1,0)=tempx(0,0);
            _DCMds(_DCMds.rows()-1,1)=tempy(0,0);;
            // cout<<timeDs<<endl;
            //  cout<<stepDuration<<endl;
           // cout<<tempy(0,0)<<endl;
            _DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);

        }
        else if ( time>=(stepDuration-0.5*_doubleSupportRatio*stepDuration)) {
            timeDs=timeDs+_timeStep;
            // cout<<timeDs<<endl;
            // cout<<stepDuration<<endl;
            MatrixXd time2(1,4);
            time2<<pow(timeDs,3),pow(timeDs,2),timeDs,1;
            MatrixXd tempx=time2*_coefXDCMDoubleSupport;
            MatrixXd tempy=time2*_coefYDCMDoubleSupport;
            _DCMds(_DCMds.rows()-1,0)=tempx(0,0);
            _DCMds(_DCMds.rows()-1,1)=tempy(0,0);
            _DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);
          //  cout<<tempy(0,0)<<endl;
        }
        else {
            _DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);
            _DCMds(_DCMds.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
            _DCMds(_DCMds.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);


        }

    }
    //_DCM(_DCM.rows()-1,0)=((InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*(1-_xdeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,0)- XCoPDisplacementSS)*((_xdeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,0)*exp(_omega*time);
    // _DCM(_DCM.rows()-1,1)=((InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*(1-_ydeltaNom)/(1-(1/GamaX)))*(0.500*exp(-_omega*time)-0.500*exp(_omega*time))+(InitialCoP(stepNumber-1,1)- YCoPDisplacementSS)*((_ydeltaNom-(1/GamaX))/(1-(1/GamaX)))*(1-exp(_omega*time))+InitialDCM(stepNumber-1,1)*exp(_omega*time);


    _CoM(_CoM.rows()-1,0)=(InitialCoM(stepNumber-1,0)-_DCM(_DCM.rows()-1,0))*exp(-_omega*time)+_DCM(_DCM.rows()-1,0);
    _CoM(_CoM.rows()-1,1)=(InitialCoM(stepNumber-1,1)-_DCM(_DCM.rows()-1,1))*exp(-_omega*time)+_DCM(_DCM.rows()-1,1);

    _CoMds(_CoMds.rows()-1,0)=(InitialCoMds(stepNumber-1,0)-_DCMds(_DCMds.rows()-1,0))*exp(-_omega*time)+_DCMds(_DCMds.rows()-1,0);
    _CoMds(_CoMds.rows()-1,1)=(InitialCoMds(stepNumber-1,1)-_DCMds(_DCMds.rows()-1,1))*exp(-_omega*time)+_DCMds(_DCMds.rows()-1,1);
    double _DCMx=_DCM(_DCM.rows()-1,0);
    double _DCMy=_DCM(_DCM.rows()-1,1);
    double _CoMy=_CoM(_CoM.rows()-1,1);
    double _CoMx=_CoM(_CoM.rows()-1,0);
    double _CoMdsy=_CoMds(_CoMds.rows()-1,1);
    double _CoMdsx=_CoMds(_CoMds.rows()-1,0);
    double _DCMdsx=_DCMds(_DCMds.rows()-1,0);
    double _DCMdsy=_DCMds(_DCMds.rows()-1,1);
    TempDCM.resize(1,8);
    TempDCM<<_DCMx,_DCMy,_CoMx,_CoMy,_DCMdsx,_DCMdsy,_CoMdsx,_CoMdsy;
    return TempDCM;

}



void TaskSpace::GetDesiredParameter(){
    double LowerBound1;
    double UpperBound1;

    double LowerBound2;
    double UpperBound2;

    double LowerBound3;
    double UpperBound3;

    double LowerBound;
    double UpperBound;
    LowerBound1=_MinStepLength/(std::abs(_XDesiredVelocity));
    UpperBound1=_MaxStepLength/(std::abs(_XDesiredVelocity));

    LowerBound2=_MinStepWidth/(std::abs(_YDesiredVelocity));
    UpperBound2=_MaxStepWidth/(std::abs(_YDesiredVelocity));

    LowerBound3=_MinStepDuration;
    UpperBound3=_MaxStepDuration;
    if (_XDesiredVelocity==0 && _YDesiredVelocity==0){
        LowerBound=LowerBound3;
        UpperBound=UpperBound3;
    }
    else if (_XDesiredVelocity==0) {

        LowerBound=max(LowerBound2,LowerBound3);
        UpperBound=min(UpperBound2,UpperBound3);
    }
    else if (_YDesiredVelocity==0) {


        LowerBound=max(LowerBound1,LowerBound3);
        UpperBound=min(UpperBound1,UpperBound3);


    }
    else {
        double tempL1=max(LowerBound1,LowerBound3);
        double tempL2=max(LowerBound1,LowerBound2);

        double tempU1=min(UpperBound1,UpperBound3);
        double tempU2=min(UpperBound1,UpperBound2);


        LowerBound=max(tempL1,tempL2);
        UpperBound=min(tempU1,tempU2);

    }
    _desiredStepDuration=(LowerBound+UpperBound)/2;
    _desiredStepLength=_XDesiredVelocity*(LowerBound+UpperBound)/2;
    _desiredStepWidth=_YDesiredVelocity*(LowerBound+UpperBound)/2;
    _sigmaDesired= exp(_omega*_desiredStepDuration);

}

MatrixXd TaskSpace::RightFoot(){
    double x;
    double xf;
    double dx;
    double ddx;
    double y;
    double dy;
    double ddy;
    double z;
    double dz;
    double ddz;
    double Yaw;
    double Roll;
    double Pitch;
    MatrixXd Time;
    MatrixXd xposition;
    MatrixXd xvelocity;
    MatrixXd xacceleration;
    double t0=0;
    MatrixXd yposition;
    MatrixXd yvelocity;
    MatrixXd yacceleration;
    double timeSS=time-_doubleSupportRatio*_stepDuration*0.5;// we define this time for single support
    Time.resize(1,2);
    Time<<timeSS-_timeStep,(1/_omega)*log(Input(3,0))-_doubleSupportRatio*_stepDuration ;//Time duration is stepDuration-DsDuration

    if (_n%2==0){

        if (time>0.5*_doubleSupportRatio*_stepDuration && time<_stepDuration-0.5*_doubleSupportRatio*_stepDuration){
            //left foot is support foot
            //        if (time==_timeStep){
            //            position.resize(1,2);
            //            velocity.resize(1,2);
            //            acceleration.resize(1,2);
            //            position(0,0)=RightFootXTrajectory[RightFootXTrajectory.size()-1];
            //            velocity(0,0)=0;
            //            acceleration(0,0)=0;
            //            position(0,1)=Input(0,0);
            //            velocity(0,1)=0;
            //            acceleration(0,1)=0;
            //        }


            //   else{


            xposition.resize(1,2);
            xvelocity.resize(1,2);
            xacceleration.resize(1,2);
            xposition(0,0)=RightFootXTrajectory[RightFootXTrajectory.size()-1];
            xvelocity(0,0)=RightFootXVelocity[RightFootXVelocity.size()-1];
            xacceleration(0,0)=RightFootXAcceleration[RightFootXAcceleration.size()-1];
            xposition(0,1)=Input(0,0);
            xvelocity(0,1)=0;
            xacceleration(0,1)=0;
            //  }


            yposition.resize(1,2);
            yvelocity.resize(1,2);
            yacceleration.resize(1,2);
            yposition(0,0)=RightFootYTrajectory[RightFootYTrajectory.size()-1];
            yvelocity(0,0)=RightFootYVelocity[RightFootYVelocity.size()-1];
            yacceleration(0,0)=RightFootYAcceleration[RightFootYAcceleration.size()-1];
            yposition(0,1)=Input(4,0);
            yvelocity(0,1)=0;
            yacceleration(0,1)=0;

            MatrixXd CoefX =Coef.Coefficient(Time,xposition,xvelocity,xacceleration);
            MatrixXd outputx= GetAccVelPos(CoefX.topRows(1),timeSS,timeSS-_timeStep,5);

            x=outputx(0,0);
            dx=outputx(0,1);
            ddx=outputx(0,2);

            MatrixXd CoefY =Coef.Coefficient(Time,yposition,yvelocity,yacceleration);
            MatrixXd outputy= GetAccVelPos(CoefY.topRows(1),timeSS,timeSS-_timeStep,5);

            y=outputy(0,0);
            dy=outputy(0,1);
            ddy=outputy(0,2);
        }

        else {
            x=RightFootXTrajectory[RightFootXTrajectory.size()-1];
            dx=0;
            ddx=0;

            y=RightFootYTrajectory[RightFootYTrajectory.size()-1];
            dy=0;
            ddy=0;
        }

    }


    else {
        x=RightFootXTrajectory[RightFootXTrajectory.size()-1];
        dx=0;
        ddx=0;

        y=RightFootYTrajectory[RightFootYTrajectory.size()-1];
        dy=0;
        ddy=0;
    }



    if (_n%2==0){
        MatrixXd H(10,10);
        MatrixXd F(1,10);
        MatrixXd Aiq(2,10);
        MatrixXd biq(2,1);
        MatrixXd Aeq;
        MatrixXd beq;
        if (time>0.5*_doubleSupportRatio*_stepDuration && time<_stepDuration-0.5*_doubleSupportRatio*_stepDuration){
            if(timeSS!=_timeStep){
                Aeq.resize(9,10);
                beq.resize(9,1);

            }

            else{

                Aeq.resize(6,10);
                beq.resize(6,1);


            }

            H<<2*pow((Time(0,1)/2-t0),18),2*pow((Time(0,1)/2-t0),17),2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),
                    2*pow((Time(0,1)/2-t0),17),2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),
                    2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),
                    2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),
                    2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),
                    2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),
                    2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),
                    2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),
                    2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),2*pow((Time(0,1)/2-t0),1),
                    2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),2*pow((Time(0,1)/2-t0),1),2*pow((Time(0,1)/2-t0),0);

            F<<-2*pow((Time(0,1)/2-t0),9)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),8)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),7)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),6)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),5)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),4)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),3)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),2)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),1)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),0)*_desiredMAxFootHeight;


            Aiq<<pow((timeSS-t0),9),pow((timeSS-t0),8),pow((timeSS-t0),7),pow((timeSS-t0),6),pow((timeSS-t0),5),pow((timeSS-t0),4),pow((timeSS-t0),3),pow((timeSS-t0),2),pow((timeSS-t0),1),pow((timeSS-t0),0),
                    -1*pow((timeSS-t0),9),-1*pow((timeSS-t0),8),-1*pow((timeSS-t0),7),-1*pow((timeSS-t0),6),-1*pow((timeSS-t0),5),-1*pow((timeSS-t0),4),-1*pow((timeSS-t0),3),-1*pow((timeSS-t0),2),-1*pow((timeSS-t0),1),-1*pow((timeSS-t0),0);
            biq<<_boundMaxFootHeight,-1*_ankleLength;
            if((timeSS)!=_timeStep){//at the fist timestep we dont have any data so the constraint on the position,velocity and acceleration of  last step will be removeed
                // we have changed above  time
                Aeq<<pow((0-t0),9),pow((0-t0),8),pow((0-t0),7),pow((0-t0),6),pow((0-t0),5),pow((0-t0),4),pow((0-t0),3),pow((0-t0),2),pow((0-t0),1),1,
                        pow((timeSS-_timeStep-t0),9),pow((timeSS-_timeStep-t0),8),pow((timeSS-_timeStep-t0),7),pow((timeSS-_timeStep-t0),6),pow((timeSS-_timeStep-t0),5),pow((timeSS-_timeStep-t0),4),pow((timeSS-_timeStep-t0),3),pow((timeSS-_timeStep-t0),2),pow((timeSS-_timeStep-t0),1),1,
                        pow((Time(0,1)-t0),9),pow((Time(0,1)-t0),8),pow((Time(0,1)-t0),7),pow((Time(0,1)-t0),6),pow((Time(0,1)-t0),5),pow((Time(0,1)-t0),4),pow((Time(0,1)-t0),3),pow((Time(0,1)-t0),2),pow((Time(0,1)-t0),1),1,
                        9*pow((0-t0),8),8*pow((0-t0),7),7*pow((0-t0),6),6*pow((0-t0),5),5*pow((0-t0),4),4*pow((0-t0),3),3*pow((0-t0),2),2*pow((0-t0),1),1*pow((0-t0),0),0*pow((0-t0),0),
                        9*pow((timeSS-_timeStep-t0),8),8*pow((timeSS-_timeStep-t0),7),7*pow((timeSS-_timeStep-t0),6),6*pow((timeSS-_timeStep-t0),5),5*pow((timeSS-_timeStep-t0),4),4*pow((timeSS-_timeStep-t0),3),3*pow((timeSS-_timeStep-t0),2),2*pow((timeSS-_timeStep-t0),1),1,0,
                        9*pow((Time(0,1)-t0),8),8*pow((Time(0,1)-t0),7),7*pow((Time(0,1)-t0),6),6*pow((Time(0,1)-t0),5),5*pow((Time(0,1)-t0),4),4*pow((Time(0,1)-t0),3),3*pow((Time(0,1)-t0),2),2*pow((Time(0,1)-t0),1),1*pow((Time(0,1)-t0),0),0*pow((Time(0,1)-t0),0),
                        9*8*pow((0-t0),7),8*7*pow((0-t0),6),7*6*pow((0-t0),5),6*5*pow((0-t0),4),5*4*pow((0-t0),3),4*3*pow((0-t0),2),3*2*pow((0-t0),1),2*1*pow((0-t0),0),0*pow((0-t0),0),0*pow((0-t0),0),
                        9*8*pow((timeSS-_timeStep-t0),7),8*7*pow((timeSS-_timeStep-t0),6),7*6*pow((timeSS-_timeStep-t0),5),6*5*pow((timeSS-_timeStep-t0),4),5*4*pow((timeSS-_timeStep-t0),3),4*3*pow((timeSS-_timeStep-t0),2),3*2*pow((timeSS-_timeStep-t0),1),2,0,0,
                        9*8*pow((Time(0,1)-t0),7),8*7*pow((Time(0,1)-t0),6),7*6*pow((Time(0,1)-t0),5),6*5*pow((Time(0,1)-t0),4),5*4*pow((Time(0,1)-t0),3),4*3*pow((Time(0,1)-t0),2),3*2*pow((Time(0,1)-t0),1),2*1*pow((Time(0,1)-t0),0),0,0;
                beq<<_ankleLength,RightFootZTrajectory[RightFootZTrajectory.size()-1],_ankleLength,0,RightFootZVelocity[RightFootZVelocity.size()-1],0,0,RightFootZacceleration[RightFootZacceleration.size()-1],0;
            }

            else{
                Aeq<<pow((0-t0),9),pow((0-t0),8),pow((0-t0),7),pow((0-t0),6),pow((0-t0),5),pow((0-t0),4),pow((0-t0),3),pow((0-t0),2),pow((0-t0),1),1,
                        pow((Time(0,1)-t0),9),pow((Time(0,1)-t0),8),pow((Time(0,1)-t0),7),pow((Time(0,1)-t0),6),pow((Time(0,1)-t0),5),pow((Time(0,1)-t0),4),pow((Time(0,1)-t0),3),pow((Time(0,1)-t0),2),pow((Time(0,1)-t0),1),1,
                        9*pow((0-t0),8),8*pow((0-t0),7),7*pow((0-t0),6),6*pow((0-t0),5),5*pow((0-t0),4),4*pow((0-t0),3),3*pow((0-t0),2),2*pow((0-t0),1),1*pow((0-t0),0),0*pow((0-t0),0),
                        9*pow((Time(0,1)-t0),8),8*pow((Time(0,1)-t0),7),7*pow((Time(0,1)-t0),6),6*pow((Time(0,1)-t0),5),5*pow((Time(0,1)-t0),4),4*pow((Time(0,1)-t0),3),3*pow((Time(0,1)-t0),2),2*pow((Time(0,1)-t0),1),1*pow((Time(0,1)-t0),0),0*pow((Time(0,1)-t0),0),
                        9*8*pow((0-t0),7),8*7*pow((0-t0),6),7*6*pow((0-t0),5),6*5*pow((0-t0),4),5*4*pow((0-t0),3),4*3*pow((0-t0),2),3*2*pow((0-t0),1),2*1*pow((0-t0),0),0*pow((0-t0),0),0*pow((0-t0),0),
                        9*8*pow((Time(0,1)-t0),7),8*7*pow((Time(0,1)-t0),6),7*6*pow((Time(0,1)-t0),5),6*5*pow((Time(0,1)-t0),4),5*4*pow((Time(0,1)-t0),3),4*3*pow((Time(0,1)-t0),2),3*2*pow((Time(0,1)-t0),1),2*1*pow((Time(0,1)-t0),0),0,0;

                beq<<1*_ankleLength,1*_ankleLength,0,0,0,0;

            }

            VectorXd sol(H.rows());
            MatrixXd HTemp2 = H;

            VectorXd  ci0=biq;
            MatrixXd CIT=-1*Aiq.transpose();


            MatrixXd CE=1*Aeq.transpose();
            VectorXd ce0=-1*beq;


            VectorXd g(Map<VectorXd>(F.data(),F.cols()*F.rows()));

            double optCost;

            optCost = solve_quadprog(HTemp2, g, CE,ce0 ,CIT , ci0, sol);
            if(optCost==std::numeric_limits<double>::infinity()){
                qDebug("Quadratic Programming for Foot Z Trajectory failed!");
            }

            MatrixXd CoefZ = sol;
            MatrixXd outputz= GetAccVelPos(CoefZ.transpose(),timeSS,0,9);
            if (outputz(0,0)<_ankleLength){
                //Whenever the equality constraint for previous timeStep does not satisfied this part will activate
                //we have sometime a confliction between equality and inequality
                //Always the equality constraint should be satisfiy then the inequality constraint will satisfy.
                //and therefore there is some situation that equality is satisfied but inequlaity is not satisfied
                //one situation is that the last timestep trajectory comes a little smaller than ankleHeight therefore in the next timestep the equality constraint use this value as a bound
                //therefore in a undesirable loop it deacrease and because the priority is on the equality constraint, the inequality constraint that say the heigh of foot should be larger the ankle height will be neglected
                //based on above sicussion we use following statement to correct this preocedure
                outputz(0,0)=_ankleLength;
                outputz(0,1)=-0*outputz(0,1);
                outputz(0,2)=-0*outputz(0,2);
            }
            z=outputz(0,0);
            dz=outputz(0,1);
            ddz=outputz(0,2);
        }

        else {
            z=RightFootZTrajectory[RightFootZTrajectory.size()-1];
            dz=0;
            ddz=0;
        }


    }
    else {
        z=RightFootZTrajectory[RightFootZTrajectory.size()-1];
        dz=0;
        ddz=0;
    }


    MatrixXd FootTrajectories(1,9);
    FootTrajectories<<x,dx,ddx,y,dy,ddy,z,dz,ddz;
    return FootTrajectories;
}


MatrixXd TaskSpace::LeftFoot(){
    double x;
    double xf;
    double dx;
    double ddx;
    double y;
    double dy;
    double ddy;
    double z;
    double dz;
    double ddz;
    double Yaw;
    double Roll;
    double Pitch;
    MatrixXd Time;
    MatrixXd xposition;
    MatrixXd xvelocity;
    MatrixXd xacceleration;
    double t0=0;
    MatrixXd yposition;
    MatrixXd yvelocity;
    MatrixXd yacceleration;
    double timeSS=time-_doubleSupportRatio*_stepDuration*0.5;// we define this time for single support
    Time.resize(1,2);
    Time<<timeSS-_timeStep,(1/_omega)*log(Input(3,0))-_doubleSupportRatio*_stepDuration ;//Time duration is stepDuration-DsDuration






    if (_n%2!=0){


        if (time>0.5*_doubleSupportRatio*_stepDuration && time<_stepDuration-0.5*_doubleSupportRatio*_stepDuration){
        //Right foot is support foot
        //        if (time==_timeStep){
        //            position.resize(1,2);
        //            velocity.resize(1,2);
        //            acceleration.resize(1,2);
        //            position(0,0)=RightFootXTrajectory[RightFootXTrajectory.size()-1];
        //            velocity(0,0)=0;
        //            acceleration(0,0)=0;
        //            position(0,1)=Input(0,0);
        //            velocity(0,1)=0;
        //            acceleration(0,1)=0;
        //        }


        //   else{

        xposition.resize(1,2);
        xvelocity.resize(1,2);
        xacceleration.resize(1,2);
        xposition(0,0)=LeftFootXTrajectory[LeftFootXTrajectory.size()-1];
        xvelocity(0,0)=LeftFootXVelocity[LeftFootXVelocity.size()-1];
        xacceleration(0,0)=LeftFootXAcceleration[LeftFootXAcceleration.size()-1];
        xposition(0,1)=Input(0,0);
        xvelocity(0,1)=0;
        xacceleration(0,1)=0;
        //  }


        yposition.resize(1,2);
        yvelocity.resize(1,2);
        yacceleration.resize(1,2);
        yposition(0,0)=LeftFootYTrajectory[LeftFootYTrajectory.size()-1];
        yvelocity(0,0)=LeftFootYVelocity[LeftFootYVelocity.size()-1];
        yacceleration(0,0)=LeftFootYAcceleration[LeftFootYAcceleration.size()-1];
        yposition(0,1)=Input(4,0);
        yvelocity(0,1)=0;
        yacceleration(0,1)=0;

        MatrixXd CoefX =Coef.Coefficient(Time,xposition,xvelocity,xacceleration);
        MatrixXd outputx= GetAccVelPos(CoefX.topRows(1),timeSS,timeSS-_timeStep,5);

        x=outputx(0,0);
        dx=outputx(0,1);
        ddx=outputx(0,2);

        MatrixXd CoefY =Coef.Coefficient(Time,yposition,yvelocity,yacceleration);
        MatrixXd outputy= GetAccVelPos(CoefY.topRows(1),timeSS,timeSS-_timeStep,5);

        y=outputy(0,0);
        dy=outputy(0,1);
        ddy=outputy(0,2);

    }

        else {
            x=LeftFootXTrajectory[LeftFootXTrajectory.size()-1];
            dx=0;
            ddx=0;

            y=LeftFootYTrajectory[LeftFootYTrajectory.size()-1];
            dy=0;
            ddy=0;
        }
}

    else {
        x=LeftFootXTrajectory[LeftFootXTrajectory.size()-1];
        dx=0;
        ddx=0;

        y=LeftFootYTrajectory[LeftFootYTrajectory.size()-1];
        dy=0;
        ddy=0;
    }



    if (_n%2!=0){
        MatrixXd H(10,10);
        MatrixXd F(1,10);
        MatrixXd Aiq(2,10);
        MatrixXd biq(2,1);
        MatrixXd Aeq;
        MatrixXd beq;
if (time>0.5*_doubleSupportRatio*_stepDuration && time<_stepDuration-0.5*_doubleSupportRatio*_stepDuration){
        if(timeSS!=_timeStep){
            Aeq.resize(9,10);
            beq.resize(9,1);

        }

        else{

            Aeq.resize(6,10);
            beq.resize(6,1);


        }

        H<<2*pow((Time(0,1)/2-t0),18),2*pow((Time(0,1)/2-t0),17),2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),
                2*pow((Time(0,1)/2-t0),17),2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),
                2*pow((Time(0,1)/2-t0),16),2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),
                2*pow((Time(0,1)/2-t0),15),2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),
                2*pow((Time(0,1)/2-t0),14),2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),
                2*pow((Time(0,1)/2-t0),13),2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),
                2*pow((Time(0,1)/2-t0),12),2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),
                2*pow((Time(0,1)/2-t0),11),2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),
                2*pow((Time(0,1)/2-t0),10),2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),2*pow((Time(0,1)/2-t0),1),
                2*pow((Time(0,1)/2-t0),9),2*pow((Time(0,1)/2-t0),8),2*pow((Time(0,1)/2-t0),7),2*pow((Time(0,1)/2-t0),6),2*pow((Time(0,1)/2-t0),5),2*pow((Time(0,1)/2-t0),4),2*pow((Time(0,1)/2-t0),3),2*pow((Time(0,1)/2-t0),2),2*pow((Time(0,1)/2-t0),1),2*pow((Time(0,1)/2-t0),0);

        F<<-2*pow((Time(0,1)/2-t0),9)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),8)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),7)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),6)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),5)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),4)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),3)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),2)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),1)*_desiredMAxFootHeight,-2*pow((Time(0,1)/2-t0),0)*_desiredMAxFootHeight;


        Aiq<<pow((timeSS-t0),9),pow((timeSS-t0),8),pow((timeSS-t0),7),pow((timeSS-t0),6),pow((timeSS-t0),5),pow((timeSS-t0),4),pow((timeSS-t0),3),pow((timeSS-t0),2),pow((timeSS-t0),1),pow((timeSS-t0),0),
                -1* pow((timeSS-t0),9),-1*pow((timeSS-t0),8),-1*pow((timeSS-t0),7),-1*pow((timeSS-t0),6),-1*pow((timeSS-t0),5),-1*pow((timeSS-t0),4),-1*pow((timeSS-t0),3),-1*pow((timeSS-t0),2),-1*pow((timeSS-t0),1),-1*pow((timeSS-t0),0);
        biq<<_boundMaxFootHeight,-1*_ankleLength;
        if(timeSS!=_timeStep){
            Aeq<<pow((0-t0),9),pow((0-t0),8),pow((0-t0),7),pow((0-t0),6),pow((0-t0),5),pow((0-t0),4),pow((0-t0),3),pow((0-t0),2),pow((0-t0),1),1,
                    pow((timeSS-_timeStep-t0),9),pow((timeSS-_timeStep-t0),8),pow((timeSS-_timeStep-t0),7),pow((timeSS-_timeStep-t0),6),pow((timeSS-_timeStep-t0),5),pow((timeSS-_timeStep-t0),4),pow((timeSS-_timeStep-t0),3),pow((timeSS-_timeStep-t0),2),pow((timeSS-_timeStep-t0),1),1,
                    pow((Time(0,1)-t0),9),pow((Time(0,1)-t0),8),pow((Time(0,1)-t0),7),pow((Time(0,1)-t0),6),pow((Time(0,1)-t0),5),pow((Time(0,1)-t0),4),pow((Time(0,1)-t0),3),pow((Time(0,1)-t0),2),pow((Time(0,1)-t0),1),1,
                    9*pow((0-t0),8),8*pow((0-t0),7),7*pow((0-t0),6),6*pow((0-t0),5),5*pow((0-t0),4),4*pow((0-t0),3),3*pow((0-t0),2),2*pow((0-t0),1),1*pow((0-t0),0),0*pow((0-t0),0),
                    9*pow((timeSS-_timeStep-t0),8),8*pow((timeSS-_timeStep-t0),7),7*pow((timeSS-_timeStep-t0),6),6*pow((timeSS-_timeStep-t0),5),5*pow((timeSS-_timeStep-t0),4),4*pow((timeSS-_timeStep-t0),3),3*pow((timeSS-_timeStep-t0),2),2*pow((timeSS-_timeStep-t0),1),1,0,
                    9*pow((Time(0,1)-t0),8),8*pow((Time(0,1)-t0),7),7*pow((Time(0,1)-t0),6),6*pow((Time(0,1)-t0),5),5*pow((Time(0,1)-t0),4),4*pow((Time(0,1)-t0),3),3*pow((Time(0,1)-t0),2),2*pow((Time(0,1)-t0),1),1*pow((Time(0,1)-t0),0),0*pow((Time(0,1)-t0),0),
                    9*8*pow((0-t0),7),8*7*pow((0-t0),6),7*6*pow((0-t0),5),6*5*pow((0-t0),4),5*4*pow((0-t0),3),4*3*pow((0-t0),2),3*2*pow((0-t0),1),2*1*pow((0-t0),0),0*pow((0-t0),0),0*pow((0-t0),0),
                    9*8*pow((timeSS-_timeStep-t0),7),8*7*pow((timeSS-_timeStep-t0),6),7*6*pow((timeSS-_timeStep-t0),5),6*5*pow((timeSS-_timeStep-t0),4),5*4*pow((timeSS-_timeStep-t0),3),4*3*pow((timeSS-_timeStep-t0),2),3*2*pow((timeSS-_timeStep-t0),1),2,0,0,
                    9*8*pow((Time(0,1)-t0),7),8*7*pow((Time(0,1)-t0),6),7*6*pow((Time(0,1)-t0),5),6*5*pow((Time(0,1)-t0),4),5*4*pow((Time(0,1)-t0),3),4*3*pow((Time(0,1)-t0),2),3*2*pow((Time(0,1)-t0),1),2*1*pow((Time(0,1)-t0),0),0,0;

            beq<<_ankleLength,LeftFootZTrajectory[LeftFootZTrajectory.size()-1],_ankleLength,0,LeftFootZVelocity[LeftFootZVelocity.size()-1],0,0,LeftFootZAcceleration[LeftFootZAcceleration.size()-1],0;
        }
        else{
            Aeq<<pow((0-t0),9),pow((0-t0),8),pow((0-t0),7),pow((0-t0),6),pow((0-t0),5),pow((0-t0),4),pow((0-t0),3),pow((0-t0),2),pow((0-t0),1),1,
                    pow((Time(0,1)-t0),9),pow((Time(0,1)-t0),8),pow((Time(0,1)-t0),7),pow((Time(0,1)-t0),6),pow((Time(0,1)-t0),5),pow((Time(0,1)-t0),4),pow((Time(0,1)-t0),3),pow((Time(0,1)-t0),2),pow((Time(0,1)-t0),1),1,
                    9*pow((0-t0),8),8*pow((0-t0),7),7*pow((0-t0),6),6*pow((0-t0),5),5*pow((0-t0),4),4*pow((0-t0),3),3*pow((0-t0),2),2*pow((0-t0),1),1*pow((0-t0),0),0*pow((0-t0),0),
                    9*pow((Time(0,1)-t0),8),8*pow((Time(0,1)-t0),7),7*pow((Time(0,1)-t0),6),6*pow((Time(0,1)-t0),5),5*pow((Time(0,1)-t0),4),4*pow((Time(0,1)-t0),3),3*pow((Time(0,1)-t0),2),2*pow((Time(0,1)-t0),1),1*pow((Time(0,1)-t0),0),0*pow((Time(0,1)-t0),0),
                    9*8*pow((0-t0),7),8*7*pow((0-t0),6),7*6*pow((0-t0),5),6*5*pow((0-t0),4),5*4*pow((0-t0),3),4*3*pow((0-t0),2),3*2*pow((0-t0),1),2*1*pow((0-t0),0),0*pow((0-t0),0),0*pow((0-t0),0),
                    9*8*pow((Time(0,1)-t0),7),8*7*pow((Time(0,1)-t0),6),7*6*pow((Time(0,1)-t0),5),6*5*pow((Time(0,1)-t0),4),5*4*pow((Time(0,1)-t0),3),4*3*pow((Time(0,1)-t0),2),3*2*pow((Time(0,1)-t0),1),2*1*pow((Time(0,1)-t0),0),0,0;



            beq<<_ankleLength,_ankleLength,0,0,0,0;

        }

        VectorXd sol(H.rows());
        MatrixXd HTemp2 = H;

        VectorXd  ci0=biq;
        MatrixXd CIT=-1*Aiq.transpose();


        MatrixXd CE=-1*Aeq.transpose();
        VectorXd ce0=beq;


        VectorXd g(Map<VectorXd>(F.data(),F.cols()*F.rows()));

        double optCost;

        optCost = solve_quadprog(HTemp2, g, CE,ce0 ,CIT , ci0, sol);
        if(optCost==std::numeric_limits<double>::infinity()){
            qDebug("Quadratic Programming for Foot Z Trajectory failed!");
        }

        MatrixXd CoefZ = sol;
        MatrixXd outputz= GetAccVelPos(CoefZ.transpose(),timeSS,0,9);

        if (outputz(0,0)<_ankleLength){
            outputz(0,0)=_ankleLength;
            outputz(0,1)=-0*outputz(0,1);
            outputz(0,2)=-0*outputz(0,2);
        }
        z=outputz(0,0);
        dz=outputz(0,1);
        ddz=outputz(0,2);
    }
else {
    z=LeftFootZTrajectory[LeftFootZTrajectory.size()-1];
    dz=0;
    ddz=0;
}
    }
    else {
        z=LeftFootZTrajectory[LeftFootZTrajectory.size()-1];
        dz=0;
        ddz=0;
    }
    MatrixXd FootTrajectories(1,9);
    FootTrajectories<<x,dx,ddx,y,dy,ddy,z,dz,ddz;
    return FootTrajectories;
}




MatrixXd TaskSpace::GetAccVelPos(MatrixXd Coef,double time,double ti,int PolynomialOrder)
{
    int PolyNomialDegree=PolynomialOrder;
    MatrixXd T(PolyNomialDegree+1,1);
    T.fill(0);
    MatrixXd Diag(PolyNomialDegree+1,PolyNomialDegree+1);
    Diag.fill(0);
    for (int var = 0; var < PolyNomialDegree+1; var++) {
        T(var,0)=pow((time-ti),PolyNomialDegree-var);
        if (var!=0) {
            Diag.diagonal(1)(var-1,0)=PolyNomialDegree-var+1;

        }
    }

    MatrixXd x=Coef*T;
    double X=x(0,0);

    MatrixXd v=Coef*Diag*T;
    double V=v(0,0);

    MatrixXd a=Coef*Diag*Diag*T;
    double A=a(0,0);

    MatrixXd Output(1,3);
    Output<<X,V,A;
    return Output;
}


void TaskSpace::SetWalkState(bool walkState)
{
    _walkstate=walkState;
}

void TaskSpace::SetStepTimingGain(double alphaT4)
{
    _alphaT=alphaT4;
}

void TaskSpace::SetStepPositionXGain(double alpha1)
{
    _alpha1x=alpha1;
}

void TaskSpace::SetStepPositionYGain(double alpha1)
{
    _alpha1y=alpha1;
}

void TaskSpace::SetDeltaXGain(double alpha3)
{
    _alpha2x=alpha3;
}

void TaskSpace::SetDeltaYGain(double alpha3)
{
    _alpha2y=alpha3;
}


void TaskSpace::SetDCMOffsetXGain(double alpha4)
{
    _alpha3x=alpha4;
}

void TaskSpace::SetDCMOffsetYGain(double alpha4)
{
    _alpha3y=alpha4;
}
void TaskSpace::SetDesiredMAxFootHeight(double desiredMaxFootHeight){

    _desiredMAxFootHeight=desiredMaxFootHeight;
}
