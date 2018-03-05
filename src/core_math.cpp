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

#include "ubitrack_python/opaque_types.h"
#include "ubitrack_python/pyubitrack.h"

#include <string>
#include <sstream>
#include <complex>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <utMath/Stochastic/Average.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);


/*
 * Scalar Types
 */
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



/*
 * Vector Types
 */

template<typename T, std::size_t N>
struct vector_constructor_helper
{
    template<typename Class_>
    void bind(Class_ &cls) {}
};


template<typename T>
struct vector_constructor_helper<T, 2> 
{
    template<typename Class_>
    void bind(Class_ &cls) {
        cls.def(py::init<T, T>());
    }
};

template<typename T>
struct vector_constructor_helper<T, 3> 
{
    template<typename Class_>
    void bind(Class_ &cls) {
        cls.def(py::init<T, T, T>());
    }
};

template<typename T>
struct vector_constructor_helper<T, 4> 
{
    template<typename Class_>
    void bind(Class_ &cls) {
        cls.def(py::init<T, T, T, T>());
    }
};

template<typename T, std::size_t N>
void bind_vector(py::module &m, const std::string &type_name, const std::string &doc_txt)
{

    typedef Ubitrack::Math::Vector<T, N> VecType;
    py::class_<VecType, boost::shared_ptr<VecType>> vec_cls(m, type_name.c_str(), doc_txt.c_str(), py::buffer_protocol());

    vector_constructor_helper<T, N>().bind(vec_cls);

    vec_cls
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

/*
 * ErrorVector Types
 */
template<typename T, std::size_t N>
void bind_error_vector(py::module &m, const std::string &type_name, const std::string &doc_txt)
{

    typedef Ubitrack::Math::ErrorVector<T, N> VecType;

    py::class_<VecType, boost::shared_ptr<VecType>>(m, type_name.c_str(), doc_txt.c_str())
        .def(py::init<>())
        .def(py::init<const Ubitrack::Math::Vector< T, N>&, const Ubitrack::Math::Matrix<T, N, N>& >())
        .def("getRMS", &VecType::getRMS)
        .def_property("value", [](const VecType &ev) { return ev.value;}, [](VecType &ev, Ubitrack::Math::Vector< T, N>& v){ ev.value = v;})
        .def_property("covariance", [](const VecType &ev) { return ev.covariance;}, [](VecType &ev, Ubitrack::Math::Matrix<T, N, N>& m){ ev.covariance = m;})
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
        ;

}

/*
 * Matrix Types
 */
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

/*
 * Quaternion Types
 */
void bind_quaternion(py::module &m, const std::string &type_name, const std::string &doc_txt)
{
    typedef Ubitrack::Math::Quaternion QuatType;


    py::class_<QuatType, boost::shared_ptr<QuatType>> cls(m, type_name.c_str(), doc_txt.c_str());

    cls
        .def(py::init<>())
        // boost quaternion constructors
        .def("__init__", [](QuatType &q, std::complex<double>& c1, std::complex<double>& c2) {
            new (&q) QuatType(boost::math::quaternion<double>(c1, c2));
        })
        // ubitrack quaternion constructors
        .def(py::init<const Ubitrack::Math::Vector< double, 3 >&, const double>())
        .def("__init__", [](QuatType &q, py::buffer b, double angle) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<double>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible array dimension!");

            if (info.shape[0] != 3)
                throw std::runtime_error("Incompatible array size!");

            Ubitrack::Math::Vector<double, 3> v(static_cast<double *>(info.ptr));
            new (&q) QuatType(v, angle);
        })
        .def(py::init<const Ubitrack::Math::Matrix< double, 3, 3 >&>())
        .def("__init__", [](QuatType &q, py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<double>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible array dimension!");

            if ((info.shape[0] != 3) && (info.shape[1] != 3))
                throw std::runtime_error("Incompatible array size!");

            Ubitrack::Math::Matrix<double, 3, 3> m(static_cast<double *>(info.ptr));
            new (&q) QuatType(m);
        })
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

        // operator overloading is implemented explicitly to avoid fiddling with boost::math::quaternions in python
        // iadd: +=
        .def("__iadd__", [](const QuatType &l, double r) {
                return QuatType(boost::math::quaternion<double>(l) += r);
            }, py::is_operator())
        .def("__iadd__", [](const QuatType &l, std::complex<double> r) {
                return QuatType(boost::math::quaternion<double>(l) += r);
            }, py::is_operator())
        .def("__iadd__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) += r);
            }, py::is_operator())

        // isub: -=
        .def("__isub__", [](const QuatType &l, double r) {
                return QuatType(boost::math::quaternion<double>(l) -= r);
            }, py::is_operator())
        .def("__isub__", [](const QuatType &l, std::complex<double> r) {
                return QuatType(boost::math::quaternion<double>(l) -= r);
            }, py::is_operator())
        .def("__isub__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) -= r);
            }, py::is_operator())

        // imul: *=
        .def("__imul__", [](const QuatType &l, double r) {
                return QuatType(boost::math::quaternion<double>(l) *= r);
            }, py::is_operator())
        .def("__imul__", [](const QuatType &l, std::complex<double> r) {
                return QuatType(boost::math::quaternion<double>(l) *= r);
            }, py::is_operator())
        .def("__imul__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) *= r);
            }, py::is_operator())

        // idiv: /=
        .def("__idiv__", [](const QuatType &l, double r) {
                return QuatType(boost::math::quaternion<double>(l) /= r);
            }, py::is_operator())
        .def("__idiv__", [](const QuatType &l, std::complex<double> r) {
                return QuatType(boost::math::quaternion<double>(l) /= r);
            }, py::is_operator())
        .def("__idiv__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) /= r);
            }, py::is_operator())

        // add, sub, mul, div
        .def("__add__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) + r);
            }, py::is_operator())
        .def("__sub__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) - r);
            }, py::is_operator())
        .def("__mul__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) * r);
            }, py::is_operator())
        .def("__truediv__", [](const QuatType &l, const QuatType& r) {
                return QuatType(boost::math::quaternion<double>(l) / r);
            }, py::is_operator())

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
        .def_static("fromVector", [](py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<double>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible array dimension!");

            if (info.shape[0] != 4)
                throw std::runtime_error("Incompatible array size!");
            
            Ubitrack::Math::Vector<double, 4> v(static_cast<double *>(info.ptr));
            return QuatType(v( 0 ), v( 1 ), v( 2 ), v( 3 ));
        })

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

}

/*
 * Math Functions provided by Boost::math
 */
void bind_math_functions(py::module &m) 
{
    m.def("slerp", &Ubitrack::Math::slerp);

    m.def("sup", &boost::math::sup<double>);
    m.def("sup", [](const Ubitrack::Math::Quaternion &q){ return boost::math::sup(q);});

    m.def("l1", &boost::math::l1<double>);
    m.def("l1", [](const Ubitrack::Math::Quaternion &q){ return boost::math::l1(q);});

    m.def("abs", &boost::math::abs<double>);
    m.def("abs", [](const Ubitrack::Math::Quaternion &q){ return boost::math::abs(q);});

    m.def("norm", &boost::math::norm<double>);
    m.def("norm", [](const Ubitrack::Math::Quaternion &q){ return boost::math::norm(q);});

    m.def("conj", &boost::math::conj<double>);
    m.def("conj", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::conj(q));});

    m.def("spherical", [](const double &rho, const double &theta, const double &phi1, const double &phi2){ 
        return Ubitrack::Math::Quaternion(boost::math::spherical(rho, theta, phi1, phi2));
    });

    m.def("semipolar", [](const double &rho, const double &alpha, const double &theta1, const double &theta2){ 
        return Ubitrack::Math::Quaternion(boost::math::semipolar(rho, alpha, theta1, theta2));
    });

    m.def("multipolar", [](const double &rho1, const double &theta1, const double &rho2, const double &theta2){ 
        return Ubitrack::Math::Quaternion(boost::math::multipolar(rho1, theta1, rho2, theta2));
    });

    m.def("cylindrospherical", [](const double &t, const double &radius, const double &longitude, const double &latitude){ 
        return Ubitrack::Math::Quaternion(boost::math::cylindrospherical(t, radius, longitude, latitude));
    });

    m.def("cylindrical", [](const double &r, const double &angle, const double &h1, const double &h2){ 
        return Ubitrack::Math::Quaternion(boost::math::cylindrical(r, angle, h1, h2));
    });

    m.def("exp", &boost::math::exp<double>);
    m.def("exp", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::exp(q));});

    m.def("cos", &boost::math::cos<double>);
    m.def("cos", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::cos(q));});

    m.def("sin", &boost::math::sin<double>);
    m.def("sin", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::sin(q));});

    m.def("tan", &boost::math::tan<double>);
    m.def("tan", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::tan(q));});

    m.def("cosh", &boost::math::cosh<double>);
    m.def("cosh", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::cosh(q));});

    m.def("sinh", &boost::math::sinh<double>);
    m.def("sinh", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::sinh(q));});

    m.def("tanh", &boost::math::tanh<double>);
    m.def("tanh", [](const Ubitrack::Math::Quaternion &q){ return Ubitrack::Math::Quaternion(boost::math::tanh(q));});

    m.def("pow", &boost::math::pow<double>);
    m.def("pow", [](const Ubitrack::Math::Quaternion &q, int n){ return Ubitrack::Math::Quaternion(boost::math::pow(q, n));});

}

/*
 * Interpolation functions
 */
void bind_interpolation_functions(py::module &m)
{
 m.def("linearInterpolatePose",       (Ubitrack::Math::Pose (*)(const Ubitrack::Math::Pose&, const Ubitrack::Math::Pose&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateErrorPose",  (Ubitrack::Math::ErrorPose (*)(const Ubitrack::Math::ErrorPose&, const Ubitrack::Math::ErrorPose&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateQuaternion", (Ubitrack::Math::Quaternion (*)(const Ubitrack::Math::Quaternion&, const Ubitrack::Math::Quaternion&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector2",    (Ubitrack::Math::Vector< double, 2 > (*)(const Ubitrack::Math::Vector< double, 2 >&, const Ubitrack::Math::Vector< double, 2 >&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector3",    (Ubitrack::Math::Vector< double, 3 > (*)(const Ubitrack::Math::Vector< double, 3 >&, const Ubitrack::Math::Vector< double, 3 >&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector4",    (Ubitrack::Math::Vector< double, 4 > (*)(const Ubitrack::Math::Vector< double, 4 >&, const Ubitrack::Math::Vector< double, 4 >&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector5",    (Ubitrack::Math::Vector< double, 5 > (*)(const Ubitrack::Math::Vector< double, 5 >&, const Ubitrack::Math::Vector< double, 5 >&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector6",    (Ubitrack::Math::Vector< double, 6 > (*)(const Ubitrack::Math::Vector< double, 6 >&, const Ubitrack::Math::Vector< double, 6 >&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector7",    (Ubitrack::Math::Vector< double, 7 > (*)(const Ubitrack::Math::Vector< double, 7 >&, const Ubitrack::Math::Vector< double, 7 >&, double)) &Ubitrack::Math::linearInterpolate);
 m.def("linearInterpolateVector8",    (Ubitrack::Math::Vector< double, 8 > (*)(const Ubitrack::Math::Vector< double, 8 >&, const Ubitrack::Math::Vector< double, 8 >&, double)) &Ubitrack::Math::linearInterpolate);

}

/*
 * STL Vector Helpers
 */
template<typename T>
void bind_stl_vector(py::module &m, const std::string &type_name, const std::string &doc_txt)
{
    typedef std::vector<T> VType;

    py::class_<VType>(m, type_name.c_str(), doc_txt.c_str())
        .def(py::init<>())
        .def("pop_back", &VType::pop_back)
        /* There are multiple versions of push_back(), etc. Select the right ones. */
        .def("push_back", (void (VType::*)(const T &)) &VType::push_back)
        .def("back", (T &(VType::*)()) &VType::back)
        .def("__len__", [](const VType &v) { return v.size(); })
        .def("__iter__", [](VType &v) {
           return py::make_iterator(v.begin(), v.end());
        }, py::keep_alive<0, 1>());
}


void bind_utMath(py::module& m)
{
    bind_math_functions(m);

    bind_scalar<unsigned long>(m, "ScalarID", "Math::Scalar<unsigned long>");
    bind_scalar<int>(m, "ScalarInt", "Math::Scalar<int>");
    bind_scalar<double>(m, "ScalarDouble", "Math::Scalar<double>");

    bind_vector<std::size_t, 2>(m, "Vector2ui", "Math::Vector<std::size_t,2>");

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

    bind_error_vector<double, 2>(m, "ErrorVector2d", "Math::ErrorVector<double, 2>");
    bind_error_vector<double, 3>(m, "ErrorVector3d", "Math::ErrorVector<double, 3>");
    bind_error_vector<double, 7>(m, "ErrorVector7d", "Math::ErrorVector<double, 7>");

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

    bind_interpolation_functions(m);


    py::class_<Ubitrack::Math::Pose, boost::shared_ptr<Ubitrack::Math::Pose> > pose_cls(m, "Pose", "Math::Pose");
    pose_cls
        .def(py::init<const Ubitrack::Math::Quaternion&, const Ubitrack::Math::Vector< double, 3 >&>())
        .def("__init__", [](Ubitrack::Math::Pose &p, const Ubitrack::Math::Quaternion& q, py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<double>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 1)
                throw std::runtime_error("Incompatible array dimension!");

            if (info.shape[0] != 3)
                throw std::runtime_error("Incompatible array size!");

            Ubitrack::Math::Vector<double, 3> v(static_cast<double *>(info.ptr));
            new (&p) Ubitrack::Math::Pose(q, v);
        })
        .def(py::init<const Ubitrack::Math::Matrix< double, 4, 4 >&>())
        .def("__init__", [](Ubitrack::Math::Pose &p, py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some sanity checks ... */
            if (info.format != py::format_descriptor<double>::format())
                throw std::runtime_error("Incompatible array datatype!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible array dimension!");

            if ((info.shape[0] != 4) && (info.shape[1] != 4))
                throw std::runtime_error("Incompatible array size!");

            Ubitrack::Math::Matrix<double, 4, 4> m(static_cast<double *>(info.ptr));
            new (&p) Ubitrack::Math::Pose(m);
        })
        .def("rotation", &Ubitrack::Math::Pose::rotation, py::return_value_policy::reference_internal)
        .def("translation", &Ubitrack::Math::Pose::translation, py::return_value_policy::reference_internal)
        .def("scalePose", &Ubitrack::Math::Pose::scalePose)

        .def("toVector", [](Ubitrack::Math::Pose &p) {
            Ubitrack::Math::Vector< double, 7 > vec;
            p.toVector(vec);
            return vec;
        })
        .def_static("fromVector", &Ubitrack::Math::Pose::fromVector<Ubitrack::Math::Vector< double, 7 >>)

        .def("toMatrix", [](Ubitrack::Math::Pose &p) {
            Ubitrack::Math::Matrix< double, 4, 4 > mat(p);
            return mat;
        })

        .def(py::self * Ubitrack::Math::Vector< double, 3 >())
        .def(py::self * py::self)
        .def(py::self == py::self)

        .def("invert", (Ubitrack::Math::Pose (Ubitrack::Math::Pose::*)())&Ubitrack::Math::Pose::operator~)

        .def("__repr__", [](Ubitrack::Math::Pose &p) -> std::string {
            std::ostringstream sout;
            sout << "<Pose ";
            sout << p;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](Ubitrack::Math::Pose &p) -> std::string {
            std::ostringstream sout;
            sout << p;
            return sout.str();            
        })
        ;

    py::class_<Ubitrack::Math::ErrorPose, boost::shared_ptr<Ubitrack::Math::ErrorPose> >(m, "ErrorPose", "Math::ErrorPose", pose_cls)
        .def(py::init<const Ubitrack::Math::Quaternion&, const Ubitrack::Math::Vector< double, 3 >&, const Ubitrack::Math::Matrix< double, 6, 6 >& >())
        .def(py::init<const Ubitrack::Math::Pose&, const Ubitrack::Math::Matrix< double, 6, 6 >& >())
        .def("covariance", &Ubitrack::Math::ErrorPose::covariance)

        .def("toAdditiveErrorVector", [](Ubitrack::Math::ErrorPose &ep){
            Ubitrack::Math::ErrorVector< double, 7 > ev;
            ep.toAdditiveErrorVector(ev);
            return ev;
        })
        .def_static("fromAdditiveErrorVector", &Ubitrack::Math::ErrorPose::fromAdditiveErrorVector)

        .def("invert", (Ubitrack::Math::ErrorPose (Ubitrack::Math::ErrorPose::*)())&Ubitrack::Math::ErrorPose::operator~)

        .def("__repr__", [](Ubitrack::Math::ErrorPose &ep) -> std::string {
            std::ostringstream sout;
            sout << "<ErrorPose ";
            sout << ep;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](Ubitrack::Math::ErrorPose &ep) -> std::string {
            std::ostringstream sout;
            sout << ep;
            return sout.str();            
        })
        ;

    py::class_<Ubitrack::Math::RotationVelocity, boost::shared_ptr<Ubitrack::Math::RotationVelocity> >(m, "RotationVelocity", "Math::RotationVelocity")
        .def(py::init<double, double, double>())
        .def("integrate", &Ubitrack::Math::RotationVelocity::integrate)
        .def("angularVelocity", &Ubitrack::Math::RotationVelocity::angularVelocity)
        .def("axis", &Ubitrack::Math::RotationVelocity::axis)
        .def("toVector", [](Ubitrack::Math::RotationVelocity &rv){
            Ubitrack::Math::Vector< double, 3 > v(rv);
            return v;
        })
 
        .def("__repr__", [](Ubitrack::Math::RotationVelocity &rv) -> std::string {
            std::ostringstream sout;
            sout << "<RotationVelocity ";
            sout << rv;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](Ubitrack::Math::RotationVelocity &rv) -> std::string {
            std::ostringstream sout;
            sout << rv;
            return sout.str();            
        })
        ;


    py::class_<Ubitrack::Math::CameraIntrinsics<double>, boost::shared_ptr<Ubitrack::Math::CameraIntrinsics<double>> > ci_cls(m, "CameraIntrinsics", "Math::CameraIntrinsics<double>");
    ci_cls
        .def(py::init<const Ubitrack::Math::CameraIntrinsics<double> &>())
        .def_property("intrinsics", 
            [](const Ubitrack::Math::CameraIntrinsics<double> &ci) { return ci.matrix;}, 
            [](Ubitrack::Math::CameraIntrinsics<double> &ci, Ubitrack::Math::Matrix< double, 3, 3>& m){ ci.matrix = m;}
            )
        .def_property("dimension", 
            [](const Ubitrack::Math::CameraIntrinsics<double> &ci) { return ci.dimension;}, 
            [](Ubitrack::Math::CameraIntrinsics<double> &ci, Ubitrack::Math::Vector< std::size_t, 2 >& d){ ci.dimension = d;}
            )
        .def_property("tangential_params", 
            [](const Ubitrack::Math::CameraIntrinsics<double> &ci) { return ci.tangential_params;}, 
            [](Ubitrack::Math::CameraIntrinsics<double> &ci, Ubitrack::Math::Vector< double, 2 >& v){ ci.tangential_params = v;}
            )
        .def_property("radial_params", 
            [](const Ubitrack::Math::CameraIntrinsics<double> &ci) { return ci.radial_params;}, 
            [](Ubitrack::Math::CameraIntrinsics<double> &ci, Ubitrack::Math::Vector< double, 6 >& v){ ci.radial_params = v;}
            )
 
        .def("angleVertical", &Ubitrack::Math::CameraIntrinsics<double>::angleVertical)
        .def("angleHorizontal", &Ubitrack::Math::CameraIntrinsics<double>::angleHorizontal)
        .def("angleDiagonal", &Ubitrack::Math::CameraIntrinsics<double>::angleDiagonal)
 
        .def("__repr__", [](Ubitrack::Math::CameraIntrinsics<double> &ci) -> std::string {
            std::ostringstream sout;
            sout << "<CameraIntrinsics ";
            sout << ci;
            sout << " >";
            return sout.str();            
        })
        .def("__str__", [](Ubitrack::Math::CameraIntrinsics<double> &ci) -> std::string {
            std::ostringstream sout;
            sout << ci;
            return sout.str();            
        })
        ;

    bind_stl_vector<Ubitrack::Math::Scalar<unsigned long>>(m, "ScalarIDList", "Math::Scalar<unsigned long>");
    bind_stl_vector<Ubitrack::Math::Scalar<int>>(m, "ScalarIntList", "Math::Scalar<int>");
    bind_stl_vector<Ubitrack::Math::Scalar<double>>(m, "ScalarDoubleList", "Math::Scalar<double>");
    bind_stl_vector<UbitrackMathTypes::Vector2d>(m, "Vector2dList", "Math::Vector<double,2>");
    bind_stl_vector<UbitrackMathTypes::Vector3d>(m, "Vector3dList", "Math::Vector<double,3>");
    bind_stl_vector<UbitrackMathTypes::Vector4d>(m, "Vector4dList", "Math::Vector<double,4>");
    bind_stl_vector<UbitrackMathTypes::Vector5d>(m, "Vector5dList", "Math::Vector<double,5>");
    bind_stl_vector<UbitrackMathTypes::Vector6d>(m, "Vector6dList", "Math::Vector<double,6>");
    bind_stl_vector<UbitrackMathTypes::Vector7d>(m, "Vector7dList", "Math::Vector<double,7>");
    bind_stl_vector<UbitrackMathTypes::Vector8d>(m, "Vector8dList", "Math::Vector<double,8>");
    bind_stl_vector<UbitrackMathTypes::Vector2f>(m, "Vector2fList", "Math::Vector<float,2>");
    bind_stl_vector<UbitrackMathTypes::Vector3f>(m, "Vector3fList", "Math::Vector<float,3>");
    bind_stl_vector<UbitrackMathTypes::Vector4f>(m, "Vector4fList", "Math::Vector<float,4>");
    bind_stl_vector<UbitrackMathTypes::Vector5f>(m, "Vector5fList", "Math::Vector<float,5>");
    bind_stl_vector<UbitrackMathTypes::Vector6f>(m, "Vector6fList", "Math::Vector<float,6>");
    bind_stl_vector<UbitrackMathTypes::Vector7f>(m, "Vector7fList", "Math::Vector<float,7>");
    bind_stl_vector<UbitrackMathTypes::Vector8f>(m, "Vector8fList", "Math::Vector<float,8>");
    bind_stl_vector<UbitrackMathTypes::ErrorVector2d>(m, "ErrorVector2dList", "Math::ErrorVector<double,2>");
    bind_stl_vector<UbitrackMathTypes::ErrorVector3d>(m, "ErrorVector3dList", "Math::ErrorVector<double,3>");
    bind_stl_vector<UbitrackMathTypes::ErrorVector7d>(m, "ErrorVector7dList", "Math::ErrorVector<double,7>");
    bind_stl_vector<UbitrackMathTypes::Matrix22d>(m, "Matrix22dList", "Math::Matrix<double,2,2>");
    bind_stl_vector<UbitrackMathTypes::Matrix33d>(m, "Matrix33dList", "Math::Matrix<double,3,3>");
    bind_stl_vector<UbitrackMathTypes::Matrix34d>(m, "Matrix34dList", "Math::Matrix<double,3,4>");
    bind_stl_vector<UbitrackMathTypes::Matrix44d>(m, "Matrix44dList", "Math::Matrix<double,4,4>");
    bind_stl_vector<UbitrackMathTypes::Matrix66d>(m, "Matrix66dList", "Math::Matrix<double,6,6>");
    bind_stl_vector<UbitrackMathTypes::Matrix22f>(m, "Matrix22fList", "Math::Matrix<float,2,2>");
    bind_stl_vector<UbitrackMathTypes::Matrix33f>(m, "Matrix33fList", "Math::Matrix<float,3,3>");
    bind_stl_vector<UbitrackMathTypes::Matrix34f>(m, "Matrix34fList", "Math::Matrix<float,3,4>");
    bind_stl_vector<UbitrackMathTypes::Matrix44f>(m, "Matrix44fList", "Math::Matrix<float,4,4>");
    bind_stl_vector<UbitrackMathTypes::Matrix66f>(m, "Matrix66fList", "Math::Matrix<float,6,6>");
    bind_stl_vector<Ubitrack::Math::Pose>(m, "PoseList", "Math::Pose");
    bind_stl_vector<Ubitrack::Math::ErrorPose>(m, "ErrorPoseList", "Math::ErrorPose");
    bind_stl_vector<Ubitrack::Math::RotationVelocity>(m, "RotationVelocityList", "Math::RotationVelocity");
    bind_stl_vector<Ubitrack::Math::CameraIntrinsics<double>>(m, "CameraIntrinsicsList", "Math::CameraIntrinsics");


}