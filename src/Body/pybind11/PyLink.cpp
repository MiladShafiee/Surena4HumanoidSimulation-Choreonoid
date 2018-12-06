/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Link.h"
#include <cnoid/ValueTree>
#include <cnoid/SceneGraph>
#include <cnoid/PyReferenced>
#include <cnoid/PyEigenTypes>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

void Link_setPosition(Link& self, const Position& T) { self.setPosition(T); }
Vector3 Link_getTranslation(Link& self) { return self.translation(); }
void Link_setTranslation(Link& self, const Vector3& p) { self.setTranslation(p); }
Matrix3 Link_getRotation(Link& self) { return self.rotation(); }
void Link_setRotation(Link& self, const Matrix3& R) { self.setRotation(R); }
Vector3 Link_getOffsetTranslation(Link& self) { return self.offsetTranslation(); }
Matrix3 Link_getOffsetRotation(Link& self) { return self.offsetRotation(); }

}

namespace cnoid {

void exportPyLink(py::module& m)
{
    py::class_<Link, LinkPtr, Referenced> link(m, "Link");
    link
        .def("name", &Link::name)
        .def("index", &Link::index)
        .def("isValid", &Link::isValid)
        .def("parent", &Link::parent)
        .def("sibling", &Link::sibling)
        .def("child", &Link::child)
        .def("isRoot", &Link::isRoot)
        .def_property("T", (Position&(Link::*)())&Link::T, Link_setPosition)
        .def("position", (Position&(Link::*)())&Link::position)
        .def("setPosition", Link_setPosition)
        .def_property("p", Link_getTranslation, Link_setTranslation)
        .def("translation", Link_getTranslation)
        .def("setTranslation", Link_setTranslation)
        .def_property("R", Link_getRotation, Link_setRotation)
        .def("rotation", Link_getRotation)
        .def("setRotation", Link_setRotation)
        .def_property_readonly("Tb", (Position&(Link::*)())&Link::Tb)
        .def_property_readonly("b", Link_getOffsetTranslation)
        .def("offsetTranslation", Link_getOffsetTranslation)
        .def_property_readonly("Rb", Link_getOffsetRotation)
        .def("offsetRotation", Link_getOffsetRotation)
        .def("jointId", &Link::jointId)
        .def("jointType", &Link::jointType)
        .def("isFixedJoint", &Link::isFixedJoint)
        .def("isFreeJoint", &Link::isFreeJoint)
        .def("isRevoluteJoint", &Link::isRevoluteJoint)
        .def("isRotationalJoint", &Link::isRotationalJoint)
        .def("isPrismaticJoint", &Link::isPrismaticJoint)
        .def("isSlideJoint", &Link::isSlideJoint)
        .def_property_readonly("a", &Link::a)
        .def("jointAxis", &Link::jointAxis)
        .def_property_readonly("d", &Link::d)
        .def_property("q", (double&(Link::*)())&Link::q, [](Link& self, double q){ self.q() = q; })
        .def_property("dq", (double&(Link::*)())&Link::dq, [](Link& self, double dq){ self.dq() = dq; })
        .def_property("ddq", (double&(Link::*)())&Link::ddq, [](Link& self, double ddq){ self.ddq() = ddq; })
        .def_property("u", (double&(Link::*)())&Link::u, [](Link& self, double u){ self.u() = u; })
        .def_property_readonly("q_upper", (double(Link::*)()const)&Link::q_upper)
        .def_property_readonly("q_lower", (double(Link::*)()const)&Link::q_lower)
        .def_property_readonly("dq_upper", (double(Link::*)()const)&Link::dq_upper)
        .def_property_readonly("dq_lower", (double(Link::*)()const)&Link::dq_lower)
        .def_property("v", (Vector3&(Link::*)())&Link::v, [](Link& self, const Vector3& v){ self.v() = v; })
        .def_property("w", (Vector3&(Link::*)())&Link::w, [](Link& self, const Vector3& w){ self.w() = w; })
        .def_property("dv", (Vector3&(Link::*)())&Link::dv, [](Link& self, const Vector3& dv){ self.dv() = dv; })
        .def_property("dw", (Vector3&(Link::*)())&Link::dw, [](Link& self, const Vector3& dw){ self.dw() = dw; })
        .def_property_readonly("c", &Link::c)
        .def("centerOfMass", &Link::centerOfMass)
        .def_property("wc", (Vector3&(Link::*)())&Link::wc, [](Link& self, const Vector3& wc){ self.wc() = wc; })
        .def("centerOfMassGlobal", (Vector3&(Link::*)())&Link::centerOfMassGlobal)
        .def_property_readonly("m", &Link::m)
        .def("mass", &Link::mass)
        .def_property_readonly("I", &Link::I)
        .def_property_readonly("Jm2", &Link::Jm2)
        .def_property("F_ext", (Vector6&(Link::*)())&Link::F_ext, [](Link& self, const Vector6& F){ self.F_ext() = F; })
        .def_property("f_ext", [](Link& self) -> Vector3 { return self.f_ext(); },
                      [](Link& self, const Vector3& f){ self.f_ext() = f; })
        .def_property("tau_ext", [](Link& self) -> Vector3 { return self.tau_ext(); },
                      [](Link& self, const Vector3& tau){ self.tau_ext() = tau; })
        .def("materialId", &Link::materialId)
        .def("materialName", &Link::materialName)
        .def("shape", &Link::shape)
        .def("visualShape", &Link::visualShape)
        .def("collisionShape", &Link::collisionShape)
        .def("setIndex", &Link::setIndex)
        .def("prependChild", &Link::prependChild)
        .def("appendChild", &Link::appendChild)
        .def("removeChild", &Link::removeChild)
        .def("setJointType", &Link::setJointType)
        .def("setJointId", &Link::setJointId)
        .def("setJointAxis", &Link::setJointAxis)
        .def("setJointRange", &Link::setJointRange)
        .def("setJointVelocityRange", &Link::setJointVelocityRange)
        .def("setMass", &Link::setMass)
        .def("setInertia", &Link::setInertia)
        .def("setCenterOfMass", &Link::setCenterOfMass)
        .def("setEquivalentRotorInertia", &Link::setEquivalentRotorInertia)
        .def("setName", &Link::setName)
        .def("setMaterial", (void(Link::*)(int)) &Link::setMaterial)
        .def("setMaterial", (void(Link::*)(const std::string&)) &Link::setMaterial)
        .def("setShape", &Link::setShape)
        .def("setVisualShape", &Link::setVisualShape)
        .def("setCollisionShape", &Link::setCollisionShape)
        .def("attitude", &Link::attitude)
        .def("setAttitude", &Link::setAttitude)
        .def("calcRfromAttitude", &Link::calcRfromAttitude)
        .def("info", (Mapping*(Link::*)())&Link::info)
        .def("info", [](Link& self, const std::string& key, py::object defaultValue) {
            if(!PyFloat_Check(defaultValue.ptr())){
                PyErr_SetString(PyExc_TypeError, "The argument type is not supported");
                throw py::error_already_set();
            }
            double v = defaultValue.cast<double>();
            return py::cast(self.info(key, v));
        })
        .def("floatInfo", [](Link& self, const std::string& key) { return self.info<double>(key); })
        ;

    py::enum_<Link::JointType>(link, "JointType")
        .value("ROTATIONAL_JOINT", Link::JointType::ROTATIONAL_JOINT)
        .value("SLIDE_JOINT", Link::JointType::SLIDE_JOINT)
        .value("FREE_JOINT", Link::JointType::FREE_JOINT)
        .value("FIXED_JOINT", Link::JointType::FIXED_JOINT)
        .export_values();
}

}
