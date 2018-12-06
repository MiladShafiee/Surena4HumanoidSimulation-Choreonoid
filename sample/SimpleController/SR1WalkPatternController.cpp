///**
//   Sample walking motion controller for the SR1 robot model.
//   This program was ported from the "SamplePD" sample of OpenHRP3.
//   @author Shin'ichiro Nakaoka
//*/

//#include <cnoid/SimpleController>
//#include <cnoid/Link>
//#include <cnoid/BodyMotion>
//#include <cnoid/ExecutablePath>
//#include <cnoid/FileUtil>
//#include <cnoid/Sensor>
//#include <iostream>
//#include <fstream>

//using namespace std;
//using namespace cnoid;

//static double pgain[] = {
//    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
//    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
//    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
//    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
//    8000.0, 8000.0, 8000.0 };

//static double dgain[] = {
//    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
//    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
//    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
//    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
//    100.0, 100.0, 100.0 };


//class SR1WalkPatternController : public cnoid::SimpleController
//{
//    Body* ioBody;
//    int currentFrameIndex;
//    int lastFrameIndex;
//    MultiValueSeqPtr qseq;
//    MultiValueSeq::Frame qref0;
//    MultiValueSeq::Frame qref1;
//    vector<double> q0;
//    double dt;
//    Link::ActuationMode actuationMode;
//    RateGyroSensorPtr waistGyro;
//    ForceSensorPtr ankleForce;
//    AccelerationSensor* accelSensor;
    
//public:

//    virtual bool initialize(SimpleControllerIO* io) override
//    {
//        ioBody = io->body();
//        string patternFile;
//        string opt = io->optionString();

//        DeviceList<RateGyroSensor> sensors(ioBody->devices());
//        DeviceList<ForceSensor> force_sensors(ioBody->devices());

//        if(!sensors.empty()){
//            waistGyro = sensors.front();
//            waistGyro->clearState();
//        }

//        if(!force_sensors.empty()){
//            ankleForce = force_sensors.front();
//            ankleForce->clearState();
//        }
        
//        if(opt == "position"){
//            actuationMode = Link::JOINT_ANGLE;
//            patternFile = "SR1WalkPattern2.yaml";
//            io->os() << "SR1WalkPatternController: position control mode." << endl;
//        } else if(opt == "velocity"){
//            actuationMode = Link::JOINT_VELOCITY;
//            patternFile = "SR1WalkPattern2.yaml";
//            io->os() << "SR1WalkPatternController: velocity control mode." << endl;
//        } else {
//            actuationMode = Link::JOINT_TORQUE;
//            patternFile = "SR1WalkPattern.yaml";
//            io->os() << "SR1WalkPatternController: torque control mode." << endl;
//        }

//        for(auto joint : ioBody->joints()){
//            joint->setActuationMode(actuationMode);
//            io->enableIO(joint);
//        }

//        string filename = getNativePathString(
//            boost::filesystem::path(shareDirectory()) / "motion" / "SR1" / patternFile);
        
//        BodyMotion motion;
//        if(!motion.loadStandardYAMLformat(filename)){
//            io->os() << motion.seqMessage() << endl;
//            return false;
//        }
//        qseq = motion.jointPosSeq();
//        if(qseq->numFrames() == 0){
//            io->os() << "Empty motion data." << endl;
//            return false;
//        }

//        if(ioBody->numJoints() != qseq->numParts()){
//            io->os() << "The number of joints must be " << qseq->numParts() << endl;
//            return false;
//        }

//        dt = io->timeStep();
//        if(fabs(dt - qseq->timeStep()) > 1.0e-6){
//            io->os() << "Warning: the simulation time step is different from that of the motion data " << endl;
//        }
        
//        currentFrameIndex = 0;
//        lastFrameIndex = std::max(0, qseq->numFrames() - 1);
//        qref1 = qseq->frame(0);

//        q0.resize(qseq->numParts());
//        for(int i=0; i < ioBody->numJoints(); ++i){
//            q0[i] = ioBody->joint(i)->q();
//        }

//        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");

//        io->enableInput(accelSensor);

//        return true;
//    }

//    virtual bool control() override
//    {

//        Vector3 dv = accelSensor->dv();
//        cout << "dv(" << dv(0) <<"," << dv(1) <<","<< dv(2) << ")" << endl << flush;

//        //Vector3 ankleForceValue = ankleForce->f();
//        //Vector3 ankleTorqueValue = ankleForce->tau();
//        //Vector3 waistGyroValue = waistGyro->w();
//        //cout << "F(" << ankleForceValue(0) <<"," << ankleForceValue(1) <<","<< ankleForceValue(2) << ")" << endl;
//        //cout << "tau(" << ankleTorqueValue(0) <<"," << ankleTorqueValue(1) <<","<< ankleTorqueValue(2) << ")" << endl;
//        //cout << "w(" << waistGyroValue(0) <<"," << waistGyroValue(1) <<","<< waistGyroValue(2) << ")" << endl;
//        //waistGyro->
//        switch(actuationMode){

//        case Link::JOINT_TORQUE:
//            qref0 = qref1;
//            qref1 = qseq->frame(currentFrameIndex);
//            for(int i=0; i < ioBody->numJoints(); ++i){
//                Link* joint = ioBody->joint(i);
//                double q_ref = qref1[i];
//                double q = joint->q();
//                double dq_ref = (q_ref - qref0[i]) / dt;
//                double dq = (q - q0[i]) / dt;

//                joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
//                q0[i] = q;

//                //Vector3 waistGyroValue = waistGyro->w();
//                //cout << "F(" << ankleForceValue(0) <<"," << ankleForceValue(1) <<","<< ankleForceValue(2) << ")" << endl;
//                //cout << "tau(" << ankleTorqueValue(0) <<"," << ankleTorqueValue(1) <<","<< ankleTorqueValue(2) << ")" << endl;
//                //cout << "w(" << waistGyroValue(0) <<"," << waistGyroValue(1) <<","<< waistGyroValue(2) << ")" << endl;

//            }
//            break;

////        case Link::JOINT_ANGLE:
////            qref0 = qseq->frame(currentFrameIndex);
////            for(int i=0; i < ioBody->numJoints(); ++i){
////                ioBody->joint(i)->q() = qref0[i];
////            }
////            break;

////        case Link::JOINT_VELOCITY:
////            qref0 = qref1;
////            qref1 = qseq->frame(std::min(currentFrameIndex + 1, lastFrameIndex));
////            for(int i=0; i < ioBody->numJoints(); ++i){
////                ioBody->joint(i)->dq() = (qref1[i] - qref0[i]) / dt;
////            }
////            break;

//        default:
//            break;
//        }

//        if(currentFrameIndex < qseq->numFrames()){
//            ++currentFrameIndex;
//            return true;
//        } else {
//            return false;
//        }
//    }
//};

//CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1WalkPatternController)


/**
   Sample walking motion controller for the SR1 robot model.
   This program was ported from the "SamplePD" sample of OpenHRP3.
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

static double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0 };

static double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };


class SR1WalkPatternController : public cnoid::SimpleController
{
    Body* ioBody;
    int currentFrameIndex;
    int lastFrameIndex;
    MultiValueSeqPtr qseq;
    MultiValueSeq::Frame qref0;
    MultiValueSeq::Frame qref1;
    vector<double> q0;
    double dt;
    Link::ActuationMode actuationMode;

public:

    virtual bool initialize(SimpleControllerIO* io)
    {
        ioBody = io->body();
        string patternFile;
        string opt = io->optionString();

        if(opt == "position"){
            actuationMode = Link::JOINT_ANGLE;
            patternFile = "SR1WalkPattern2.yaml";
            io->os() << "SR1WalkPatternController: position control mode." << endl;
        } else if(opt == "velocity"){
            actuationMode = Link::JOINT_VELOCITY;
            patternFile = "SR1WalkPattern2.yaml";
            io->os() << "SR1WalkPatternController: velocity control mode." << endl;
        } else {
            actuationMode = Link::JOINT_TORQUE;
            patternFile = "SR1WalkPattern.yaml";
            io->setJointInput(JOINT_ANGLE);
            io->os() << "SR1WalkPatternController: torque control mode." << endl;
        }

        for(auto joint : ioBody->joints()){
            joint->setActuationMode(actuationMode);
            io->enableIO(joint);
        }

        string filename = getNativePathString(
            boost::filesystem::path(shareDirectory()) / "motion" / "SR1" / patternFile);

        BodyMotion motion;
        if(!motion.loadStandardYAMLformat(filename)){
            io->os() << motion.seqMessage() << endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if(qseq->numFrames() == 0){
            io->os() << "Empty motion data." << endl;
            return false;
        }

        if(ioBody->numJoints() != qseq->numParts()){
            io->os() << "The number of joints must be " << qseq->numParts() << endl;
            return false;
        }

        dt = io->timeStep();
        if(fabs(dt - qseq->timeStep()) > 1.0e-6){
            io->os() << "Warning: the simulation time step is different from that of the motion data " << endl;
        }

        currentFrameIndex = 0;
        lastFrameIndex = std::max(0, qseq->numFrames() - 1);
        qref1 = qseq->frame(0);

        q0.resize(qseq->numParts());
        for(int i=0; i < ioBody->numJoints(); ++i){
            q0[i] = ioBody->joint(i)->q();
        }

        return true;
    }

    virtual bool control()
    {
        switch(actuationMode){

        case Link::JOINT_TORQUE:
            qref0 = qref1;
            qref1 = qseq->frame(currentFrameIndex);
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                double q_ref = qref1[i];
                double q = joint->q();
                double dq_ref = (q_ref - qref0[i]) / dt;
                double dq = (q - q0[i]) / dt;
                joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
                q0[i] = q;
            }
            break;

        case Link::JOINT_ANGLE:
            qref0 = qseq->frame(currentFrameIndex);
            for(int i=0; i < ioBody->numJoints(); ++i){
                ioBody->joint(i)->q() = qref0[i];
            }
            break;

        case Link::JOINT_VELOCITY:
            qref0 = qref1;
            qref1 = qseq->frame(std::min(currentFrameIndex + 1, lastFrameIndex));
            for(int i=0; i < ioBody->numJoints(); ++i){
                ioBody->joint(i)->dq() = (qref1[i] - qref0[i]) / dt;
            }
            break;

        default:
            break;
        }

        if(currentFrameIndex < qseq->numFrames()){
            ++currentFrameIndex;
            return true;
        } else {
            return false;
        }
    }

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1WalkPatternController)
