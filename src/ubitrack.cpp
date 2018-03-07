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

#include "vision_main.hpp"

#include "visualization_main.hpp"

#include "dataflow_dataflow.hpp"
#include "dataflow_graph.hpp"

#include "facade_main.hpp"

PYBIND11_MODULE(ubitrack, m)
{
    // utcore wrapper
    auto m_core = m.def_submodule("core", "Core Module");

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

    // utdataflow wrapper
    auto m_dataflow = m.def_submodule("dataflow", "Dataflow Module");

    auto m_dataflow_dataflow = m_dataflow.def_submodule("dataflow", "Dataflow Functions");
    bind_utDataflowMain(m_dataflow_dataflow);

    auto m_dataflow_graph = m_dataflow.def_submodule("graph", "Graph Functions");
    bind_utDataflowGraph(m_dataflow_graph);

    // utvision wrapper
    auto m_vision = m.def_submodule("vision", "Vision Module");
    bind_utVisionMain(m_vision);
    
    // utvisualization wrapper
    auto m_visualization = m.def_submodule("visualization", "Visualization Module");
    bind_utVisualizationMain(m_visualization);

    // utfacade wrapper
    auto m_facade = m.def_submodule("facade", "Facade Module");
    bind_utFacadeMain(m_facade);

}