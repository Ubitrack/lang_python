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

#include "core_math.hpp"
#include "core_measurement.hpp"
#include "core_util.hpp"
#include "core_calibration.hpp"
#include "core_serialization.hpp"

PYBIND11_MODULE(ubitrack, m)
{
    auto m_core = m.def_submodule("core", "Core Functionality");

    auto m_core_math = m_core.def_submodule("math", "Math Types");
    bind_utMath(m_core_math);

    auto m_core_measurement = m_core.def_submodule("measurement", "Measurement Types");
    bind_utMeasurement(m_core_measurement);

    auto m_core_util = m_core.def_submodule("util", "Utility Functions");
    bind_utUtil(m_core_util);

    auto m_core_calibration = m_core.def_submodule("calibration", "Calibration Functions");
    bind_utCalibration(m_core_calibration);

    auto m_core_serialization = m_core.def_submodule("serialization", "Serialization Functions");
    bind_utSerialization(m_core_serialization);

}