#include "AGXObjectFactory.h"

namespace cnoid{

////////////////////////////////////////////////////////////
// AGXPseudoContinuousTrackGeometry
void AGXPseudoContinuousTrackGeometry::setAxis(const agx::Vec3& axis)
{
    m_axis = axis;
}

agx::Vec3 AGXPseudoContinuousTrackGeometry::getAxis() const
{
    return m_axis;
}

agx::Vec3f AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity(const agxCollide::LocalContactPoint & point, size_t index) const
{
    agx::Vec3 dir = getAxis() ^ agx::Vec3(point.normal());
    dir.normalize();
    dir *= -1.0 * getSurfaceVelocity().x();
    return agx::Vec3f(dir);
}

////////////////////////////////////////////////////////////
// AGXObjectFactory

bool AGXObjectFactory::checkModuleEnalbled(const char* name)
{
    return agx::Runtime::instance()->isModuleEnabled(name);
}

agxSDK::SimulationRef AGXObjectFactory::createSimulation(const AGXSimulationDesc & desc)
{
    agxSDK::SimulationRef sim = new agxSDK::Simulation();
    agx::setNumThreads(desc.numThreads);
    sim->setTimeStep(desc.timeStep);
    sim->setUniformGravity(desc.gravity);
    sim->getSpace()->setEnableContactReduction(desc.enableContactReduction);
    sim->getSpace()->setContactReductionBinResolution(desc.contactReductionBinResolution);
    sim->getSpace()->setContactReductionThreshold(desc.contactReductionThreshhold);
    sim->getDynamicsSystem()->getAutoSleep()->setEnable(desc.enableAutoSleep);
    return sim;
}

agx::MaterialRef AGXObjectFactory::createMaterial(const AGXMaterialDesc & desc)
{
    agx::MaterialRef m = new agx::Material(desc.name);
    m->getBulkMaterial()->setDensity(desc.density);
    m->getBulkMaterial()->setYoungsModulus(desc.youngsModulus);
    m->getBulkMaterial()->setPoissonsRatio(desc.poissonRatio);

    // Below are overried when ContactMaterials are used.
    m->getBulkMaterial()->setViscosity(desc.viscosity);
    m->getBulkMaterial()->setDamping(desc.damping);
    m->getSurfaceMaterial()->setRoughness(desc.roughness);
    m->getSurfaceMaterial()->setViscosity(desc.surfaceViscosity);
    m->getSurfaceMaterial()->setAdhesion(desc.adhesionForce, desc.adhesivOverlap);
    return m;
}

agx::ContactMaterialRef AGXObjectFactory::createContactMaterial(agx::Material* const matA, agx::Material* const matB, const AGXContactMaterialDesc& desc)
{
    agx::ContactMaterialRef cm = new agx::ContactMaterial(matA, matB);
    setContactMaterialParam(cm, desc);
    return cm;
}

agx::ContactMaterialRef AGXObjectFactory::createContactMaterial(const AGXContactMaterialDesc& desc, agxSDK::MaterialManager* const mgr)
{
    if(!mgr) return nullptr;
    agx::MaterialRef mA = mgr->getMaterial(desc.nameA);
    agx::MaterialRef mB = mgr->getMaterial(desc.nameB);
    agx::ContactMaterialRef cm = mgr->getOrCreateContactMaterial(mA, mB);
    if(!cm) return nullptr;
    setContactMaterialParam(cm, desc);
    return cm;
}

agx::RigidBodyRef AGXObjectFactory::createRigidBody(const AGXRigidBodyDesc& desc)
{
    agx::RigidBodyRef rigid = new agx::RigidBody();
    rigid->setMotionControl(desc.control);
    rigid->setVelocity(desc.v);
    rigid->setAngularVelocity(desc.w);
    rigid->setPosition(desc.p);
    rigid->setRotation(desc.R);
    rigid->setName(desc.name);
    rigid->getAutoSleepProperties().setEnable(desc.enableAutoSleep);
    return rigid;
}

agxCollide::GeometryRef AGXObjectFactory::createGeometry(const AGXGeometryDesc& desc)
{
    agxCollide::GeometryRef geometry;
    if(desc.isPseudoContinuousTrack){
        AGXPseudoContinuousTrackGeometry* pg = new AGXPseudoContinuousTrackGeometry();
        pg->setAxis(desc.axis);
        pg->setSurfaceVelocity(agx::Vec3f());
        geometry = pg;
    }else{
        geometry = new agxCollide::Geometry();
    }
    geometry->addGroup(desc.selfCollsionGroupName);
    return geometry;
}

agxCollide::ShapeRef AGXObjectFactory::createShape(const AGXShapeDesc& desc)
{
    agxCollide::ShapeRef shape = nullptr;
    switch (desc.shapeType){
        case AGXShapeType::AGXCOLLIDE_BOX :{
            shape = createShapeBox(static_cast<const AGXBoxDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_SPHERE :{
            shape = createShapeSphere(static_cast<const AGXSphereDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_CAPSULE :{
            shape = createShapeCapsule(static_cast<const AGXCapsuleDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_CYLINDER :{
           shape = createShapeCylinder(static_cast<const AGXCylinderDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_TRIMESH :{
            shape = createShapeTrimesh(static_cast<const AGXTrimeshDesc&>(desc));
            break;
        }
        default:
            break;
    }
    return shape;
}

agx::ConstraintRef AGXObjectFactory::createConstraint(const AGXConstraintDesc& desc)
{
    agx::ConstraintRef constraint = nullptr;
    switch(desc.constraintType){
        case AGXConstraintType::AGXHINGE :
            constraint = createConstraintHinge(static_cast<const AGXHingeDesc&>(desc));
            break;
        case AGXConstraintType::AGXPRISMATIC :
            constraint = createConstraintPrismatic(static_cast<const AGXPrismaticDesc&>(desc));
            break;
        case AGXConstraintType::AGXLOCKJOINT :
            constraint = createConstraintLockJoint(static_cast<const AGXLockJointDesc&>(desc));
            break;
        case AGXConstraintType::AGXBALLJOINT :
            constraint = createConstraintBallJoint(static_cast<const AGXBallJointDesc&>(desc));
            break;
        default :
            break;
    }
    return constraint;
}

agx::FrameRef AGXObjectFactory::createFrame()
{
    return new agx::Frame();
}

agxSDK::AssemblyRef AGXObjectFactory::createAssembly()
{
    return new agxSDK::Assembly();
}

agx::Bool AGXObjectFactory::setContactMaterialParam(agx::ContactMaterial* const cm, const AGXContactMaterialDesc & desc)
{
    if(!cm) return false;
    cm->setYoungsModulus(desc.youngsModulus);
    cm->setRestitution(desc.restitution);
    cm->setDamping(desc.damping);
    cm->setFrictionCoefficient(desc.friction);
    cm->setAdhesion(desc.adhesionForce, desc.adhesivOverlap);
    cm->setSurfaceViscosity(desc.surfaceViscosity);
    cm->setContactReductionMode(desc.contactReductionMode);
    cm->setContactReductionBinResolution(desc.contactReductionBinResolution);

    // Create friction model
    if(desc.frictionModelType != AGXFrictionModelType::DEFAULT){
        agx::FrictionModelRef fm = nullptr;
        switch (desc.frictionModelType){
            case AGXFrictionModelType::BOX :
                fm = new agx::BoxFrictionModel();
                break;
            case AGXFrictionModelType::SCALED_BOX :
                fm = new agx::ScaleBoxFrictionModel();
                break;
            case AGXFrictionModelType::ITERATIVE_PROJECTED_CONE :
                fm = new agx::IterativeProjectedConeFriction();
                break;
            case AGXFrictionModelType::DEFAULT:
            default:
                break;
        }
        fm->setSolveType(desc.solveType);
        cm->setFrictionModel(fm);
    }
    return true;
}

agxCollide::BoxRef AGXObjectFactory::createShapeBox(const AGXBoxDesc& desc)
{
    return new agxCollide::Box(desc.halfExtents);
}

agxCollide::SphereRef AGXObjectFactory::createShapeSphere(const AGXSphereDesc & desc)
{
    return new agxCollide::Sphere(desc.radius);
}

agxCollide::CapsuleRef AGXObjectFactory::createShapeCapsule(const AGXCapsuleDesc & desc)
{
    return new agxCollide::Capsule(desc.radius, desc.height);
}

agxCollide::CylinderRef AGXObjectFactory::createShapeCylinder(const AGXCylinderDesc & desc)
{
    return new agxCollide::Cylinder(desc.radius, desc.height);
}

agxCollide::MeshRef AGXObjectFactory::createShapeTrimesh(const AGXTrimeshDesc& desc)
{
    return new agxCollide::Trimesh(&desc.vertices, &desc.indices, desc.name, desc.optionsMask, desc.bottomMargin);
}

agx::HingeRef AGXObjectFactory::createConstraintHinge(const AGXHingeDesc& desc)
{
    agx::HingeFrame hingeFrame;
    hingeFrame.setAxis(desc.frameAxis);
    hingeFrame.setCenter(desc.frameCenter);
    agx::HingeRef joint = new agx::Hinge(hingeFrame, desc.rigidBodyA, desc.rigidBodyB);
    setMotor1DParam(joint->getMotor1D(), desc.motor);
    setLock1DParam(joint->getLock1D(), desc.lock);
    setRange1DParam(joint->getRange1D(), desc.range);
    return joint;
}

agx::PrismaticRef AGXObjectFactory::createConstraintPrismatic(const AGXPrismaticDesc & desc)
{
    agx::PrismaticFrame prismaticFrame;
    prismaticFrame.setAxis(desc.frameAxis);
    prismaticFrame.setPoint(desc.framePoint);
    agx::PrismaticRef joint = new agx::Prismatic(prismaticFrame, desc.rigidBodyA, desc.rigidBodyB);
    setMotor1DParam(joint->getMotor1D(), desc.motor);
    setLock1DParam(joint->getLock1D(), desc.lock);
    setRange1DParam(joint->getRange1D(), desc.range);
    return joint;
}

agx::BallJointRef AGXObjectFactory::createConstraintBallJoint(const AGXBallJointDesc & desc)
{
    agx::BallJointFrame ballJointFrame;
    ballJointFrame.setCenter(desc.framePoint);
    return new agx::BallJoint(ballJointFrame, desc.rigidBodyA, desc.rigidBodyB);
}

agx::PlaneJointRef AGXObjectFactory::createConstraintPlaneJoint(const AGXPlaneJointDesc & desc)
{
    return new agx::PlaneJoint(desc.rigidBodyA, desc.frameA, desc.rigidBodyB, desc.frameB);
}

void AGXObjectFactory::setMotor1DParam(agx::Motor1D* controller, const AGXMotor1DDesc& desc)
{
    controller->setEnable(desc.enable);
    controller->setLocked(desc.enableLock);
    controller->setLockedAtZeroSpeed(desc.enableLockAtZeroSpeed);
    controller->setCompliance(desc.compliance);
    controller->setDamping(desc.damping);
}

void AGXObjectFactory::setLock1DParam(agx::Lock1D* controller, const AGXLock1DDesc& desc)
{
    controller->setEnable(desc.enable);
    controller->setCompliance(desc.compliance);
    controller->setDamping(desc.damping);
}

void AGXObjectFactory::setRange1DParam(agx::Range1D* controller, const AGXRange1DDesc& desc)
{
    controller->setEnable(desc.enable);
    controller->setRange(desc.range);
    controller->setCompliance(desc.compliance);
    controller->setDamping(desc.damping);
}

agx::LockJointRef AGXObjectFactory::createConstraintLockJoint(const AGXLockJointDesc & desc)
{
    return new agx::LockJoint(desc.rigidBodyA, desc.rigidBodyB);
}

agxVehicle::TrackWheelRef AGXObjectFactory::createVehicleTrackWheel(const AGXVehicleTrackWheelDesc& desc)
{
    return new agxVehicle::TrackWheel(desc.model, desc.radius, desc.rigidbody, desc.rbRelTransform);
}

agxVehicle::TrackRef AGXObjectFactory::createVehicleTrack(const AGXVehicleTrackDesc& desc)
{
    agxVehicle::TrackRef track = new agxVehicle::Track(desc.numberOfNodes, desc.nodeWidth, desc.nodeThickness, desc.nodeDistanceTension);
    for(int i = 0; i < desc.trackWheelRefs.size(); ++i){
        track->add(desc.trackWheelRefs[i]);
    }
    track->getProperties()->setHingeCompliance(desc.hingeCompliance);
    track->getProperties()->setHingeDamping(desc.hingeDamping);
    track->getProperties()->setMinStabilizingHingeNormalForce(desc.minStabilizingHingeNormalForce);
    track->getProperties()->setStabilizingHingeFrictionParameter(desc.stabilizingHingeFrictionParameter);
    track->getInternalMergeProperties()->setEnableMerge(desc.enableMerge);
    track->getInternalMergeProperties()->setNumNodesPerMergeSegment(desc.numNodesPerMergeSegment);
    track->getInternalMergeProperties()->setEnableLockToReachMergeCondition(desc.enableLockToReachMergeCondition);
    track->getInternalMergeProperties()->setLockToReachMergeConditionCompliance(desc.lockToReachMergeConditionCompliance);
    track->getInternalMergeProperties()->setLockToReachMergeConditionDamping(desc.lockToReachMergeConditionDamping);
    track->getInternalMergeProperties()->setMaxAngleMergeCondition(desc.maxAngleMergeCondition);
    track->getInternalMergeProperties()->setContactReduction(desc.contactReduction);


    if(desc.useThickerNodeEvery <= 0) return track;
    // Add shapes for create bumpy tracks
    agx::UInt counter = 0;
    track->initialize(
        [&]( const agxVehicle::TrackNode& node )
        {
        agx::Real heightOffset = 0.0;
        agx::Real thickness = desc.nodeThickness;
        // For every useThickerNodeEvery node we add a thicker box.
        if ( ( counter++ % desc.useThickerNodeEvery ) == 0 ) {
            thickness = desc.nodeThickerThickness;
            heightOffset = -0.5 * ( thickness - desc.nodeThickness );
        }
        node.getRigidBody()->add( new agxCollide::Geometry( new agxCollide::Box( 0.5 * thickness,
        0.5 * desc.nodeWidth,
        0.5 * node.getLength() ) ),
        agx::AffineMatrix4x4::translate( heightOffset, 0, node.getHalfExtents().z() ) );
        }
    );
    return track;
}

agxCollide::ConvexBuilderRef AGXObjectFactory::createConvexBuilder(){
    return new agxCollide::ConvexBuilder();
}

}