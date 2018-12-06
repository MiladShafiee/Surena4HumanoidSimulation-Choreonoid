/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyEigenTypes.h"
#include "PySignal.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyEigenTypes(py::module& m)
{
    m.def("rpyFromRot", &cnoid::rpyFromRot);
    m.def("rotFromRpy", (Matrix3 (*)(const Vector3&)) &cnoid::rotFromRpy);
    m.def("rotFromRpy", (Matrix3 (*)(double, double, double)) &cnoid::rotFromRpy);
    m.def("rotFromRpy44", [](const Vector3& v){ return Affine3(rotFromRpy(v)); });
    m.def("omegaFromRot", &cnoid::omegaFromRot);
    m.def("angleAxis", [](double angle, const Vector3& axis){ return Matrix3(AngleAxis(angle, axis)); });
    m.def("angleAxis44", [](double angle, const Vector3& axis){ return Affine3(AngleAxis(angle, axis)); });
    m.def("normalized", [](const Vector3& v){ return v.normalized(); });
    m.def("unitX", Vector3::UnitX);
    m.def("unitY", Vector3::UnitY);
    m.def("unitZ", Vector3::UnitZ);

    PySignal<void(const Vector3&)>(m, "Vector33Signal");
    PySignal<void(const Vector4&)>(m, "Vector43Signal");
    PySignal<void(const Matrix3&)>(m, "Matrix3Signal");
    PySignal<void(const Matrix4&)>(m, "Matrix4Signal");
    PySignal<void(const Affine3&)>(m, "Affine3Signal");
}

}
