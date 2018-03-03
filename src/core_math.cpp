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

#include <utMath/Vector.h>
#include <utMath/Matrix.h>

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
                { sizeof(T)/* * N */ }      /* Strides (in bytes) for each index */
            );
         })
        .def("array_view", [](py::object &obj) {
            VecType &v = obj.cast<VecType&>();
            return py::array_t<T>({N}, {sizeof(T)/* * N */}, v.content(), obj);
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

void bind_utMath(py::module& m)
{
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

}