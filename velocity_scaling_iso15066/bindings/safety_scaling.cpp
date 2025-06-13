#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <velocity_scaling_iso15066/ssm_fixed_areas.h>

namespace py = pybind11;
using namespace ssm15066;

PYBIND11_MODULE(ssm_safety, m) {
    py::class_<FixedAreasSSM>(m, "FixedAreasSSM")
        .def(py::init<>())
        .def("init", &FixedAreasSSM::init)
        .def("computeScaling", &FixedAreasSSM::computeScaling)
        .def("add_circle_area",
             py::overload_cast<const std::string&, const double&, const double&>(&FixedAreasSSM::addArea),
             "Add circular area", py::arg("name"), py::arg("radius"), py::arg("override"))
        .def("add_polygon_area",
             py::overload_cast<const std::string&, const std::vector<std::vector<double>>&, const double&>(&FixedAreasSSM::addArea),
             "Add polygon area", py::arg("name"), py::arg("corners"), py::arg("override"))
        .def("set_point_cloud", &FixedAreasSSM::setPointCloud)
        .def("get_distance", &FixedAreasSSM::getDistanceFromClosestPoint);
}
