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
 * Ubitrack Python Bindings - Main Module.
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */


#include "ubitrack_python/opaque_types.h"
#include "ubitrack_python/pyubitrack.h"
#include <string>

void bind_utMath(py::module& m);
void bind_utMeasurement(py::module& m);
// void bind_utUtil(py::module& m);
// void bind_utCalibration(py::module& m);
// void bind_utDataflow(py::module& m);
// void bind_utGraph(py::module& m);
// void bind_utFacade(py::module& m);
// void bind_utHaptics(py::module& m);
// void bind_utVision(py::module& m);
// void bind_utVisualization(py::module& m);
// void bind_utRenderAPI(py::module& m);

PYBIND11_MODULE(ubitrack, m)
{
//     warn_about_unavailable_but_used_cpu_instructions();


// #define DLIB_QUOTE_STRING(x) DLIB_QUOTE_STRING2(x)
// #define DLIB_QUOTE_STRING2(x) #x
//     m.attr("__version__") = DLIB_QUOTE_STRING(DLIB_VERSION);
//     m.attr("__time_compiled__") = std::string(__DATE__) + " " + std::string(__TIME__);

// #ifdef DLIB_USE_CUDA
//     m.attr("DLIB_USE_CUDA") = true;
// #else
//     m.attr("DLIB_USE_CUDA") = false;
// #endif
// #ifdef DLIB_USE_BLAS 
//     m.attr("DLIB_USE_BLAS") = true;
// #else
//     m.attr("DLIB_USE_BLAS") = false;
// #endif
// #ifdef DLIB_USE_LAPACK
//     m.attr("DLIB_USE_LAPACK") = true;
// #else
//     m.attr("DLIB_USE_LAPACK") = false;
// #endif
// #ifdef DLIB_HAVE_AVX
//     m.attr("USE_AVX_INSTRUCTIONS") = true;
// #else
//     m.attr("USE_AVX_INSTRUCTIONS") = false;
// #endif
// #ifdef DLIB_HAVE_NEON 
//     m.attr("USE_NEON_INSTRUCTIONS") = true;
// #else
//     m.attr("USE_NEON_INSTRUCTIONS") = false;
// #endif



    // Note that the order here matters.  We need to do the basic types first.  If we don't 
    // then what happens is the documentation created by sphinx will use horrible big
    // template names to refer to C++ objects rather than the python names python users
    // will expect.  For instance, if bind_basic_types() isn't called early then when
    // routines take a std::vector<double>, rather than saying dlib.array in the python
    // docs it will say "std::vector<double, std::allocator<double> >" which is awful and
    // confusing to python users.
    //
    // So when adding new things always add them to the end of the list.
    auto m_core = m.def_submodule("core", "Core Functionality");

    auto m_core_math = m_core.def_submodule("math", "Math Types");
    bind_utMath(m_core_math);

    auto m_core_measurement = m_core.def_submodule("measurement", "Measurement Types");
    bind_utMeasurement(m_core_measurement);


}