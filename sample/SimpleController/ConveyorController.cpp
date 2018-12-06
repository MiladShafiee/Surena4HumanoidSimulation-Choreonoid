/**
   Conveyor Controller Sample
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class ConveyorController : public SimpleController
{
    Link* conveyorJoint;
    
public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        conveyorJoint = io->body()->joint(0);
        conveyorJoint->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        io->enableOutput(conveyorJoint);
        return true;
    }

    virtual bool control() override
    {
        conveyorJoint->dq() = 1.0;
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ConveyorController)
