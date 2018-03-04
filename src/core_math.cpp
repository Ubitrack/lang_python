/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the 
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */


/**
 * @ingroup python
 * @file
 * Ubitrack Python Bindings - Math Module.
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */

#include <string>
#include <sstream>

#include <boost/shared_ptr.hpp>

#include "ubitrack_python/pyubitrack.h"

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);

#include <utMath/Scalar.h>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/Quaternion.h>


template<typename T>
void bind_scalar(py::module &m, const std::string &type_name, const std::string &doc_txt)
{
    typedef Ubitrack::Math::Scalar<T> ScalarType;

    py::class_<ScalarType, boost::shared_ptr<ScalarType>>(m, type_name.c_str(), doc_txt.c_str())
        .def(py::init<>())
        .def(py::init<T>())
        .def_property("value", [](const ScalarType &s) { return s.m_value;}, [](ScalarType &s, T v){ s.m_value = v;})
        .def("__repr__", [type_name](ScalarType &s) -> std::string {
            std::ostringstream sout;
            sout << "<";
            sout << type_name;
            sout << " ";
            sout << s;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](ScalarType &s) -> std::string {
            std::ostringstream sout;
            sout << s;
            return sout.str();            
        })
        ;
}


template<typename T, std::size_t N>
void bind_vector(py::module &m, const std::string &type_name, const std::string &doc_txt)
{

    typedef Ubitrack::Math::Vector<T, N> VecType;

    py::class_<VecType, boost::shared_ptr<VecType>>(m, type_name.c_str(), doc_txt.c_str(), py::buffer_protocol())
        .def(py::init<>())
        .def("__init__", [](VecType &v, py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<T>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible array dimension!");

            if (info.shape[0] != N)
                throw std::runtime_error("Incompatible array size!");

            new (&v) VecType(static_cast<T *>(info.ptr));
        })
        .def_buffer([](VecType &v) -> py::buffer_info {
            return py::buffer_info(
                v.content(),                                /* Pointer to buffer */
                sizeof(T),                          /* Size of one scalar */
                py::format_descriptor<T>::format(), /* Python struct-style format descriptor */
                1,                                       /* Number of dimensions */
                { N },                  /* Buffer dimensions */
                { sizeof(T) }      /* Strides (in bytes) for each index */
            );
         })
        .def("array_view", [](py::object &obj) {
            VecType &v = obj.cast<VecType&>();
            return py::array_t<T>({N}, {sizeof(T) }, v.content(), obj);
        })
        .def("__getitem__", [](const VecType &v, long r) -> T {
            if (r >= N)
                throw pybind11::key_error("Invalid Key");
            if (r < 0)
                throw pybind11::key_error("Invalid Key");
            return v(r);
        })
        .def("__setitem__", [](VecType &v, long r, T s) {
            if (r >= N)
                throw pybind11::key_error("Invalid Key");
            if (r < 0)
                throw pybind11::key_error("Invalid Key");
            v(r) = s;
        })
        .def("__iter__", [](const VecType &v) { return py::make_iterator(v.begin(), v.end()); },
                         py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def("__repr__", [type_name](VecType &v) -> std::string {
            std::ostringstream sout;
            sout << "<";
            sout << type_name;
            sout << " ";
            sout << v;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](VecType &v) -> std::string {
            std::ostringstream sout;
            sout << v;
            return sout.str();            
        })
        .def("__len__", &VecType::size)
        ;

}


template<typename T, std::size_t N, std::size_t M>
void bind_matrix(py::module &m, const std::string &type_name, const std::string &doc_txt)
{

    typedef Ubitrack::Math::Matrix<T, N, M> MatType;

    py::class_<MatType, boost::shared_ptr<MatType>>(m, type_name.c_str(), doc_txt.c_str(), py::buffer_protocol())
        .def(py::init<>())
        .def("__init__", [](MatType &m, py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<T>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible array dimension!");

            if ((info.shape[0] != N) && (info.shape[1] != M))
                throw std::runtime_error("Incompatible array size!");

            new (&m) MatType(static_cast<T *>(info.ptr));
        })
        .def_buffer([](MatType &m) -> py::buffer_info {
            return py::buffer_info(
                m.content(),                                /* Pointer to buffer */
                sizeof(T),                          /* Size of one scalar */
                py::format_descriptor<T>::format(), /* Python struct-style format descriptor */
                2,                                       /* Number of dimensions */
                { N, M },                  /* Buffer dimensions */
                { sizeof(T), sizeof(T) * M }      /* Strides (in bytes) for each index */
            );
         })
        .def("array_view", [](py::object &obj) {
            MatType &m = obj.cast<MatType&>();
            return py::array_t<T>({N, M}, {sizeof(T), sizeof(T)*M}, m.content(), obj);
        })
        .def("__getitem__", [](const MatType &m, std::pair<size_t, size_t> i) -> T {
            if (i.first >= N)
                throw pybind11::key_error("Invalid Key");
            if (i.second >= M)
                throw pybind11::key_error("Invalid Key");
            return m(i.first, i.second);
        })
        .def("__setitem__", [](MatType &m, std::pair<size_t, size_t> i, T v) {
            if (i.first >= N)
                throw pybind11::key_error("Invalid Key");
            if (i.second >= M)
                throw pybind11::key_error("Invalid Key");
            m(i.first, i.second) = v;
        })
        // .def("__iter__", [](const MatType &m) { return py::make_iterator(m.begin(), m.end()); },
        //                  py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def("__repr__", [type_name](MatType &m) -> std::string {
            std::ostringstream sout;
            sout << "<";
            sout << type_name;
            sout << "\n";
            sout << m;
            sout << ">";
            return sout.str();            
        })
        .def("__str__", [](MatType &m) -> std::string {
            std::ostringstream sout;
            sout << m;
            return sout.str();            
        })
        .def("__len__", &MatType::size)
        ;

}


void bind_quaternion(py::module &m, const std::string &type_name, const std::string &doc_txt)
{
    typedef Ubitrack::Math::Quaternion QuatType;


    py::class_<QuatType, boost::shared_ptr<QuatType>> cls(m, type_name.c_str(), doc_txt.c_str());

    cls
        .def(py::init<>())
        // boost quaternion constructors
        // .def(py::init<std::complex<double>&, std::complex<double>&>())
        // ubitrack quaternion constructors
        .def(py::init<const Ubitrack::Math::Vector< double, 3 >&, const double>())
        .def(py::init<const Ubitrack::Math::Matrix< double, 3, 3 >&>())
        .def(py::init<const boost::math::quaternion<double>& >())
        .def(py::init<double, double, double>())
        .def(py::init<double, double, double, double>())
        // default accessors
        .def("x",&QuatType::x)
        .def("y", &QuatType::y)
        .def("z", &QuatType::z)
        .def("w",&QuatType::w)
        .def("normalize",(QuatType& (QuatType::*)())&QuatType::normalize, py::return_value_policy::reference_internal)
        .def("invert", (QuatType& (QuatType::*)())&QuatType::invert, py::return_value_policy::reference_internal)
        .def("inverted", (QuatType (QuatType::*)())&QuatType::operator~)

        .def(py::self += double())
        .def(py::self += std::complex<double>())
        .def(py::self += py::self)

        .def(py::self -= double())
        .def(py::self -= std::complex<double>())
        .def(py::self -= py::self)

        .def(py::self *= double())
        .def(py::self *= std::complex<double>())
        .def(py::self *= py::self)

        .def(py::self /= double())
        .def(py::self /= std::complex<double>())
        .def(py::self /= py::self)

        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)

        .def(py::self == double())
        .def(py::self == std::complex<double>())
        .def(py::self == py::self)

        .def(py::self != double())
        .def(py::self != std::complex<double>())
        .def(py::self != py::self)

        .def(py::self * Ubitrack::Math::Vector< double, 3 >())

        .def("real", (double (QuatType::*)())&QuatType::real)
        .def("unreal",(QuatType (QuatType::*)())&QuatType::unreal)

        .def("angle", &QuatType::angle)

        .def("negateIfCloser", &QuatType::negateIfCloser)

        .def("getEulerAngles", (Ubitrack::Math::Vector< double, 3 > (QuatType::*)(QuatType::t_EulerSequence) const)&QuatType::getEulerAngles)
        .def("toLogarithm",    (Ubitrack::Math::Vector< double, 3 > (QuatType::*)())&QuatType::toLogarithm)
        .def("toMatrix",       [](QuatType &q) {
            Ubitrack::Math::Matrix< double, 3, 3 > mat;
            q.toMatrix(mat);
            return mat;
        })
        .def("toAxisAngle",    [](QuatType &q) { 
            Ubitrack::Math::Vector< double, 3 > axis;
            double angle;
            q.toAxisAngle(axis, angle);
            return std::make_tuple(axis, angle);
        })
        .def("toVector",       [](QuatType &q) {
            Ubitrack::Math::Vector< double, 4 > vec;
            q.toVector(vec);
            return vec;
        })

        .def_static("fromLogarithm", &QuatType::fromLogarithm)
        .def_static("fromVector", &QuatType::fromVector<Ubitrack::Math::Vector< double, 4 >>)

        .def("__repr__", [type_name](QuatType &q) -> std::string {
            std::ostringstream sout;
            sout << "<";
            sout << type_name;
            sout << " ";
            sout << q;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](QuatType &q) -> std::string {
            std::ostringstream sout;
            sout << q;
            return sout.str();            
        })
        ;

    py::enum_<QuatType::t_EulerSequence>(cls, "EULER_SEQUENCE")
    .value("XYZ", QuatType::EULER_SEQUENCE_XYZ)
    .value("YZX", QuatType::EULER_SEQUENCE_YZX)
    .value("ZXY", QuatType::EULER_SEQUENCE_ZXY)
    .value("ZYX", QuatType::EULER_SEQUENCE_ZYX)
    .value("XZY", QuatType::EULER_SEQUENCE_XZY)
    .value("YXZ", QuatType::EULER_SEQUENCE_YXZ)
    ;

    m.def("slerp", &Ubitrack::Math::slerp);
    m.def("sup", &boost::math::sup<double>);
    m.def("l1", &boost::math::l1<double>);
    m.def("abs", &boost::math::abs<double>);
    m.def("conj", &boost::math::conj<double>);
    m.def("norm", &boost::math::norm<double>);
    m.def("spherical", &boost::math::spherical<double>);
    m.def("semipolar", &boost::math::semipolar<double>);
    m.def("multipolar", &boost::math::multipolar<double>);
    m.def("cylindrospherical", &boost::math::cylindrospherical<double>);
    m.def("cylindrical", &boost::math::cylindrical<double>);
    m.def("exp", &boost::math::exp<double>);
    m.def("cos", &boost::math::cos<double>);
    m.def("sin", &boost::math::sin<double>);
    m.def("tan", &boost::math::tan<double>);
    m.def("cosh", &boost::math::cosh<double>);
    m.def("sinh", &boost::math::sinh<double>);
    m.def("tanh", &boost::math::tanh<double>);
    m.def("pow", &boost::math::pow<double>);

}

void bind_utMath(py::module& m)
{
    bind_scalar<unsigned long>(m, "ScalarID", "Math::Scalar<unsigned long>");
    bind_scalar<int>(m, "ScalarInt", "Math::Scalar<int>");
    bind_scalar<double>(m, "ScalarDouble", "Math::Scalar<double>");

	bind_vector<double, 2>(m, "Vector2d", "Math::Vector<double,2>");
    bind_vector<double, 3>(m, "Vector3d", "Math::Vector<double,3>");
    bind_vector<double, 4>(m, "Vector4d", "Math::Vector<double,4>");
    bind_vector<double, 5>(m, "Vector5d", "Math::Vector<double,5>");
    bind_vector<double, 6>(m, "Vector6d", "Math::Vector<double,6>");
    bind_vector<double, 7>(m, "Vector7d", "Math::Vector<double,7>");
    bind_vector<double, 8>(m, "Vector8d", "Math::Vector<double,8>");

    bind_vector<float, 2>(m, "Vector2f", "Math::Vector<float,2>");
    bind_vector<float, 3>(m, "Vector3f", "Math::Vector<float,3>");
    bind_vector<float, 4>(m, "Vector4f", "Math::Vector<float,4>");
    bind_vector<float, 5>(m, "Vector5f", "Math::Vector<float,5>");
    bind_vector<float, 6>(m, "Vector6f", "Math::Vector<float,6>");
    bind_vector<float, 7>(m, "Vector7f", "Math::Vector<float,7>");
    bind_vector<float, 8>(m, "Vector8f", "Math::Vector<float,8>");

    bind_matrix<double, 2, 2>(m, "Matrix22d", "Math::Matrix<double,2,2>");
    bind_matrix<double, 3, 3>(m, "Matrix33d", "Math::Matrix<double,3,3>");
    bind_matrix<double, 3, 4>(m, "Matrix34d", "Math::Matrix<double,3,4>");
    bind_matrix<double, 4, 4>(m, "Matrix44d", "Math::Matrix<double,4,4>");
    bind_matrix<double, 6, 6>(m, "Matrix66d", "Math::Matrix<double,6,6>");

    bind_matrix<float, 2, 2>(m, "Matrix22f", "Math::Matrix<float,2,2>");
    bind_matrix<float, 3, 3>(m, "Matrix33f", "Math::Matrix<float,3,3>");
    bind_matrix<float, 3, 4>(m, "Matrix34f", "Math::Matrix<float,3,4>");
    bind_matrix<float, 4, 4>(m, "Matrix44f", "Math::Matrix<float,4,4>");
    bind_matrix<float, 6, 6>(m, "Matrix66f", "Math::Matrix<float,6,6>");

    bind_quaternion(m, "Quaternion", "Math::Quaternion");

}