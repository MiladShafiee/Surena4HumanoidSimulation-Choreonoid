/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/SimulatorItem>
#include <cnoid/RootItem>
#include <vector>
#include <iostream>

using namespace cnoid;
using namespace std;

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0, 8000.0,
    8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

class SR1MinimumController : public SimpleController
{
    BodyPtr ioBody;
    double dt;
    std::vector<double> qref;
    std::vector<double> qold;
    RateGyroSensorPtr waistGyro;
    double waistGyroOffset = 0;
    double K_shoulder = 0.01;
    double K_pelvis = 100;
    ItemPtr currentBodyItem;
    BodyItem* currentBodyItem1;

    SimulatorItem* activeSimulator;


public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        dt = io->timeStep();


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
        currentBodyItem = RootItem::instance()->findItem("World/SR1");
        currentBodyItem1 = RootItem::instance()->findItem<BodyItem>("World/SR1");// ::instance()->findItem("World/SR1");






        return true;
    }

    virtual bool control() override
    {
        Link* waistLink = ioBody->joint(12);
        Vector3 force;
        force << 200, 0.0, 0.0;
        Vector3 point;
        point << 0.0, 0.0, 0.0;
        activeSimulator = SimulatorItem::findActiveSimulatorItemFor(currentBodyItem);
        activeSimulator->setExternalForce(currentBodyItem1, waistLink, point, force, 1);
        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double u;
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            Vector3 waistGyroVal = waistGyro->w();


            double waistFeedback = waistGyroVal(1);
//            if(abs(waistFeedback) < 0.01)
//                waistFeedback = 0;
            if((i == 6) || (i == 19) || (i == 26)){
               // u =  (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
                qref[i] += K_shoulder*(waistFeedback) - (waistGyroOffset);
                //if(abs(u) < 0.1)
                    //u = 0;
                    cout << "(" << waistFeedback << "," << u << endl << flush;
               // cout << "u(" << u << endl << flush;
            }
//             else{
//              //  u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
//            }

            u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
               // u = K_pelvis*(waistGyroOffset - waistGyroVal(1));
            // if(i != 26)
                          //  u = -1*K_shoulder*(waistGyroVal(1) - waistGyroOffset);
            //            else
            //u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
            qold[i] = q;
            joint->u() = u;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)
