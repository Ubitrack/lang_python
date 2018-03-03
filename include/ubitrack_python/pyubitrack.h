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
 * Ubitrack Python Bindings - python helpers TOP.
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */

#ifndef UBITRACK_PYTHON_TOP_
#define UBITRACK_PYTHON_TOP_ 

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);
//PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Matrix4d>);

// some helper functions
namespace pybind11 {
namespace detail {

template <typename T, typename Class_>
void bind_default_constructor(Class_ &cl) {
	cl.def(py::init([]() {
		return new T();
	}), "Default constructor");
}

template <typename T, typename Class_>
void bind_copy_functions(Class_ &cl) {
	cl.def("__init__", [](T &t, const T &cp) {
		new (&t)T(cp);
	}, "Copy constructor");
	cl.def("__copy__", [](T &v) {
		return T(v);
	});
	cl.def("__deepcopy__", [](T &v, py::dict &memo) {
		return T(v);
	});
}

}	// namespace pybind11::detail
}	// namespace pybind11


#endif // UBITRACK_PYTHON_TOP_

