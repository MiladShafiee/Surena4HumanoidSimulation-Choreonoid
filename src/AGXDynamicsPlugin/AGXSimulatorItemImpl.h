#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H

#include <cnoid/SimulatorItem>
#include "AGXScene.h"
#include "AGXBody.h"
#include <iostream>

namespace cnoid {

class AGXSimulatorItem;
class AGXSimulatorItemImpl : public Referenced
{
public:
    AGXSimulatorItemImpl(AGXSimulatorItem* self);
    AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org);
    ~AGXSimulatorItemImpl();

    void initialize();                          // call from defualt constructor
    void doPutProperties(PutPropertyFunction& putProperty);     // add parameters to property panel
    bool store(Archive& archive);               // save simulation parameter to cnoid file
    bool restore(const Archive& archive);       // store simulation parameter from cnoid file

    // Function of create, step simulation
    SimulationBody* createSimulationBody(Body* orgBody);
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void createAGXMaterialTable();
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void stopSimulation();
    void pauseSimulation();
    void restartSimulation();

    void setGravity(const Vector3& g);
    Vector3 getGravity() const;
    bool saveSimulationToAGXFile();

private:
    ref_ptr<AGXSimulatorItem> self;
    AGXSceneRef agxScene = nullptr;
    Vector3 _p_gravity;
    int     _p_numThreads;
    bool    _p_enableContactReduction;
    int     _p_contactReductionBinResolution;
    int     _p_contactReductionThreshhold;
    bool    _p_enableAutoSleep;
    AGXScene* getAGXScene();
};
}
#endif